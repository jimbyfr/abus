/**
 * Synopsys DesignWare 8250 driver adapted for Honeywell/ILC ABUS
 *
 * Copyright 2011 Picochip, Jamie Iles.
 * Copyright 2013 Intel Corporation
 *
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy.  If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 *
 * NOTE on Honeywell ABUS implementation for STIP+ device
 * ======================================================
 *
 * this device uses a AllWinner A64 SoC which includes 4 DesignWare UARTs
 * ABUs was implemented in the UART driver as it requires strict timing :
 * - 256000 bauds, 8 bits, no parity, 1 stop
 * - in MASTER mode, 64bytes sent continously every 8ms
 * - in SLAVE mode same packet as above, but sent after a silence of between 650uS & 1ms
 *      following a packet received from the MASTER.
 *
 * When SLAVE FIFO RX threshold is reduced to avoid a reception delay of more than 1.2ms
 * which would not be acceptable for ABus.
 * ABus is working with RS485. Normally RX is disabled while TX is enabled by hardware.
 * If the driver detects it is not the case, it tries to scrap any received
 * bytes while transmitting. FIFO threshold is as well reduced for that purpose.
 * The mode (MASTER or SLAVE) is passed through termios c_cc[NCCS-1].
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>

#include <asm/byteorder.h>

#include "8250_dwlib.h"

#include <linux/hrtimer.h>

#define     FUNCTION() { printk("ABUS Function entry %s\n", __func__); }

#define     ABUS_MODE_PASSTHROUGH   0
#define     ABUS_MODE_MASTER        1
#define     ABUS_MODE_ACTIVESLAVE   2

#define     ABUS_STATE_RECEIVED     3

#define     ABUS_KEEP_RTS           2700000
#define     ABUS_MASTER_TIMER       8000000
#define     ABUS_SLAVE_TIMER         330000

#define     ABUS_FRAME_LENGTH       64
#define     ABUS_WRITE_GPIO         100
#define     ABUS_WRITE_ON           1
#define     ABUS_WRITE_OFF          0

#define     ABUS_MAX_ECHOS          64

#define     ABUS_FLAG_MODE_MASK     0x000F
#define     ABUS_FLAG_MASTER        0x0001
#define     ABUS_FLAG_SLAVE         0x0002
#define     ABUS_FLAG_MEMO          0x0010
#define     ABUS_FLAG_ECHOING       0x0020
#define     ABUS_FLAG_OPENED        0x0040
#define     ABUS_FLAG_TIO_ABUS      0x0080
#define     ABUS_FLAG_FIFO          0x0100
#define     ABUS_FLAG_ECHOED        0x0200
#define     ABUS_FLAG_SENDING       0x1000

struct abus_stats {
    unsigned long   bytes_sent;
    unsigned long   bytes_received;
    unsigned long   echos;
    unsigned long   ticks;
    unsigned long   slots;
    unsigned long   irq[8];
};

/* Offsets for the DesignWare specific registers */
#define DW_UART_USR 0x1f /* UART Status Register */

/* DesignWare specific register fields */
#define DW_UART_MCR_SIRE        BIT(6)

struct dw8250_data {
    struct dw8250_port_data data;

    u8          usr_reg;
    int         msr_mask_on;
    int         msr_mask_off;
    struct clk      *clk;
    struct clk      *pclk;
    struct notifier_block   clk_notifier;
    struct work_struct  clk_work;
    struct reset_control    *rst;

    unsigned int        skip_autocfg:1;
    unsigned int        uart_16550_compatible:1;

    // ABUS specific information
    unsigned long       state;                  // Transmit state
    unsigned            sending;                // Bytes to send in a frame
    struct hrtimer      check_sending_timer;    // When to Send
    struct hrtimer      stop_sending_timer;     // RS485 control timer
    struct abus_stats   stats;                  // Statistics
    struct mutex        lock;
    unsigned            echos;                  // Consecutive slots with echo
    unsigned            tx_loadsz_memo;         // Memo max transmit for UART
    unsigned            fcr_memo;               // Memo FCR register value
    unsigned            flags;                  // Status bits
};

static inline struct dw8250_data *to_dw8250_data(struct dw8250_port_data *data)
{
    return container_of(data, struct dw8250_data, data);
}

static inline struct dw8250_data *clk_to_dw8250_data(struct notifier_block *nb)
{
    return container_of(nb, struct dw8250_data, clk_notifier);
}

static inline struct dw8250_data *work_to_dw8250_data(struct work_struct *work)
{
    return container_of(work, struct dw8250_data, clk_work);
}

static inline int dw8250_modify_msr(struct uart_port *p, int offset, int value)
{
    struct dw8250_data *d = to_dw8250_data(p->private_data);

    /* Override any modem control signals if needed */
    if (offset == UART_MSR) {
        value |= d->msr_mask_on;
        value &= ~d->msr_mask_off;
    }

    return value;
}

static void dw8250_force_idle(struct uart_port *p)
{
    struct uart_8250_port *up = up_to_u8250p(p);

    serial8250_clear_and_reinit_fifos(up);
    (void)p->serial_in(p, UART_RX);
}

static void dw8250_check_lcr(struct uart_port *p, int value)
{
    void __iomem *offset = p->membase + (UART_LCR << p->regshift);
    int tries = 1000;

    /* Make sure LCR write wasn't ignored */
    while (tries--) {
        unsigned int lcr = p->serial_in(p, UART_LCR);

        if ((value & ~UART_LCR_SPAR) == (lcr & ~UART_LCR_SPAR))
            return;

        dw8250_force_idle(p);

        if (p->iotype == UPIO_MEM32)
            writel(value, offset);
        else if (p->iotype == UPIO_MEM32BE)
            iowrite32be(value, offset);
        else
            writeb(value, offset);
    }
    /*
     * FIXME: this deadlocks if port->lock is already held
     * dev_err(p->dev, "Couldn't set LCR to %d\n", value);
     */
}

static void dw8250_serial_out32(struct uart_port *p, int offset, int value)
{
    struct dw8250_data *d = to_dw8250_data(p->private_data);

    writel(value, p->membase + (offset << p->regshift));

    if (offset == UART_LCR && !d->uart_16550_compatible)
        dw8250_check_lcr(p, value);
}

static unsigned int dw8250_serial_in32(struct uart_port *p, int offset)
{
    unsigned int value = readl(p->membase + (offset << p->regshift));

    return dw8250_modify_msr(p, offset, value);
}

/**
 *  @brief abus_transceiver_tx() - sets the RS485 in transmit state
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static inline void abus_transceiver_tx(struct dw8250_data *d) {
    struct uart_8250_port   *up = serial8250_get_port(d->data.line);
    struct uart_port        *p = &up->port;

    spin_lock_irq(&p->lock);
    gpio_set_value(ABUS_WRITE_GPIO, ABUS_WRITE_ON);
    d->flags |= ABUS_FLAG_SENDING;
    spin_unlock_irq(&p->lock);
}

/**
 *  @brief abus_transceiver_rx() - sets the RS485 in receive state
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static inline void abus_transceiver_rx(struct dw8250_data *d) {
    struct uart_8250_port   *up = serial8250_get_port(d->data.line);
    struct uart_port        *p = &up->port;

    spin_lock_irq(&p->lock);
    d->flags &= ~ ABUS_FLAG_SENDING;
    gpio_set_value(ABUS_WRITE_GPIO, ABUS_WRITE_OFF);
    spin_unlock_irq(&p->lock);
}

/**
 *  @brief abus_adapt_fifo() - reduce the FIFO receive threshold
 *
 *  The normal FIFO receive threshold if 32 bytes, so 1.24mS
 *  it is too high ifor slave mode or when echo is detected, therefore in these
 *  cases the threshold is set to 16 (so 624ms).
 *  We do so only if the default threshold was memorized, so we can revert later.
 *  The answer timeout in case of slave must therefore be higher to that value.
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static void abus_adapt_fifo(struct dw8250_data *d) {
    struct uart_8250_port   *up = serial8250_get_port(d->data.line);
    struct uart_port        *p = &up->port;

    if ((d->flags & ABUS_FLAG_MEMO)) {
        serial8250_clear_and_reinit_fifos(up);
        up->fcr &= ~UART_FCR_TRIGGER_MASK;
        up->fcr |= UART_FCR_R_TRIG_01;
        p->serial_out(p, UART_FCR, up->fcr);
        d->flags |= ABUS_FLAG_FIFO;
    };
};

/**
 *  @brief abus_reset_fifo() - return to default FIFO receive threshold
 *
 *  Revert to default FIFO receive threshold if the FIFO was changed
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static void abus_reset_fifo(struct dw8250_data *d) {
    struct uart_8250_port   *up = serial8250_get_port(d->data.line);
    struct uart_port        *p = &up->port;

    if ((d->flags & ABUS_FLAG_FIFO)) {
        serial8250_clear_and_reinit_fifos(up);
        up->fcr &= ~UART_FCR_TRIGGER_MASK;
        up->fcr |= d->fcr_memo;
        p->serial_out(p, UART_FCR, up->fcr);
        d->flags &= ~ABUS_FLAG_FIFO;
    };
};

/**
 *  @brief abus_start_operations() - Starting abus communication
 *
 *  Memo the default FIFO size if not done before
 *  if MASTER : set the send timer to 8ms cycle
 *  if SLAVE : set the check timer (when it's check if we can answer)
 *
 *  if SLAVE or MASTER+ECHO : reduce the FIFO threshold to help operations
 *
 *  @param d pointer to the driver private data
 *  @return 0 if OK <0 if error
 */
static int abus_start_operations(struct dw8250_data *d) {
    struct uart_8250_port *up = serial8250_get_port(d->data.line);

    FUNCTION();

    if (!(d->flags & ABUS_FLAG_MEMO)) {
        d->tx_loadsz_memo = up->tx_loadsz;
        d->fcr_memo = up->fcr & UART_FCR_TRIGGER_MASK;
        d->flags |= ABUS_FLAG_MEMO;
    };

    d->sending = 0;
    printk(KERN_INFO "ABUS Start flags : %x \n", d->flags);

    d->echos = 0;

    if ((d->flags & ABUS_FLAG_MASTER)) {
        if ((d->flags & ABUS_FLAG_ECHOING))
            abus_adapt_fifo(d);
        hrtimer_start(&d->check_sending_timer, ABUS_MASTER_TIMER, HRTIMER_MODE_REL);
    };

    if ((d->flags & ABUS_FLAG_SLAVE)) {
        abus_adapt_fifo(d);
        hrtimer_start(&d->check_sending_timer, ABUS_SLAVE_TIMER, HRTIMER_MODE_REL);
    };

    return 0;
}

/**
 *  @brief abus_stop_operations() - Stoping abus communication
 *
 *  Stops the timers to prevent future sending
 *  Set the RS485 in reception mode to avoid locking the line
 *  Reset the FIFO to default threshold
 *  Indicates that we are no more in MASTER or SLAVE mode
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static void abus_stop_operations(struct dw8250_data *d) {

    FUNCTION();

    hrtimer_cancel(&d->stop_sending_timer);
    hrtimer_cancel(&d->check_sending_timer);

    abus_transceiver_rx(d);

    abus_reset_fifo(d);

    d->flags &= ~(ABUS_FLAG_MODE_MASK);
    printk(KERN_INFO "ABUS exit stop\n");
}

/**
 *  @brief abus_send_packet() - Stoping abus communication
 *
 *  Sending a packet, the maximum that the UART allows while staying in
 *  the limit of a ABUS frame of 64 bytes
 *  This can be called either from the send timer of from the TX ready IRQ
 *
 *  @param d pointer to the driver private data
 *  @param p pointer to the uart port
 *  @param up pointer to the uart 8250 port
 *  @return void
 */
void abus_send_packet(struct dw8250_data *d, struct uart_port *p, struct uart_8250_port *up) {
    unsigned long   sent;
    unsigned long   count;

    count = p->icount.tx;

    // Trick : limit the transmit size to the remainder of a ABUS frame
    up->tx_loadsz = (d->sending < d->tx_loadsz_memo) ? d->sending : d->tx_loadsz_memo;

    serial8250_tx_chars(up);
    up->tx_loadsz = d->tx_loadsz_memo;

    // Updates the stats and the remaining to send in a frame
    sent = (p->icount.tx - count);
    d->stats.bytes_sent += sent;
    d->sending = (d->sending > sent) ? d->sending - sent : 0;
}

/**
 *  @brief abus_ignore_rx_chars() - Ignoring received chars
 *
 *  This replaces the serial8250_rx_chars() function
 *  It just reads the received chars out of the UART and keeps LSR updated
 *  Received bytes are just ignored
 *
 *  @param p pointer to the uart port
 *  @param lsr value of the LSR register while entering
 *  @return value of the LRS register on exit
 */
unsigned char abus_ignore_rx_chars(struct uart_port *p, unsigned char lsr) {
    do {
        (void)p->serial_in(p, UART_RX);
        lsr = p->serial_in(p, UART_LSR);
    } while (lsr & (UART_LSR_DR|UART_LSR_BI));
    return lsr;
}

/**
 *  @brief abus_handle_irq_override() - Handling specific ABUS IRQ
 *
 *  This replaces the standard serial8250_handle_irq() function
 *  This adds the echo detection (receive during sending), the rx chars count
 *  and calls the abus_send_packet() only if ABUS frame not completed before.
 *  This prevents the 8250 driver to continously sending.
 *  This is in an interrupt context
 *
 *  @param d pointer to the driver private data
 *  @param p pointer to the uart port
 *  @param up pointer to the uart 8250 port
 *  @return void
 */
int abus_handle_irq_override(struct dw8250_data *d, struct uart_port *p, unsigned int iir)
{
    unsigned char   status;
    unsigned long   flags;
    unsigned long   count;
    struct uart_8250_port *up = up_to_u8250p(p);

    if (iir & UART_IIR_NO_INT)
        return 0;

    spin_lock_irqsave(&p->lock, flags);

    // Reading LSR clears TX interrupt
    status = serial_port_in(p, UART_LSR);

    if (status & (UART_LSR_DR | UART_LSR_BI)) {
        if ((d->flags & ABUS_FLAG_SENDING)) {
            d->stats.echos++;
            d->flags |= ABUS_FLAG_ECHOED;
        };
        count = p->icount.rx;
        if ((d->flags & (ABUS_FLAG_SENDING|ABUS_FLAG_ECHOING)) != (ABUS_FLAG_SENDING|ABUS_FLAG_ECHOING))
            status = serial8250_rx_chars(up, status);
        else
            status = abus_ignore_rx_chars(p, status);
        d->stats.bytes_received += (p->icount.rx - count);
    }
    serial8250_modem_status(up);

    if ((status & UART_LSR_THRE) && (up->ier & UART_IER_THRI) && (d->sending))
        abus_send_packet(d, p, up);

    uart_unlock_and_check_sysrq(p, flags);
    return 1;
}

/**
 *  @brief abus_handle_irq() - Driver IRQ callback
 *
 *  Rewrite of the dw8250_handle_irq() with ABUS specific adds.
 *  Extends the waiting time if chars are received while in SLAVE mode
 *  Calls abus_handle_irq_override() instead of serial8250_handle_irq
 *  if in ABUS communication
 *
 *  @param p pointer to the uart port
 *  @return 1 if irq processed, 0 if not
 */
static int abus_handle_irq(struct uart_port *p)
{
    struct uart_8250_port *up = up_to_u8250p(p);
    struct dw8250_data *d = to_dw8250_data(p->private_data);
    unsigned int iir = p->serial_in(p, UART_IIR);
    unsigned int status;
    unsigned long flags;

    /*
     * There are ways to get Designware-based UARTs into a state where
     * they are asserting UART_IIR_RX_TIMEOUT but there is no actual
     * data available.  If we see such a case then we'll do a bogus
     * read.  If we don't do this then the "RX TIMEOUT" interrupt will
     * fire forever.
     *
     * This problem has only been observed so far when not in DMA mode
     * so we limit the workaround only to non-DMA mode.
     */
    if (!up->dma && ((iir & 0x3f) == UART_IIR_RX_TIMEOUT)) {
        spin_lock_irqsave(&p->lock, flags);
        status = p->serial_in(p, UART_LSR);

        if (!(status & (UART_LSR_DR | UART_LSR_BI)))
            (void) p->serial_in(p, UART_RX);

        spin_unlock_irqrestore(&p->lock, flags);
    }

    d->stats.irq[iir & 0x07]++;

    // if received characters in slave mode, just reset the state
    if (((iir & UART_IIR_RDI) == UART_IIR_RDI) && (d->flags & ABUS_FLAG_SLAVE))
        d->state = ABUS_STATE_RECEIVED;

    if ((d->flags & (ABUS_FLAG_MASTER|ABUS_FLAG_SLAVE))) {
        if (abus_handle_irq_override(d, p, iir))
            return 1;
    } else if (serial8250_handle_irq(p, iir))
        return 1;

    if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
        /* Clear the USR */
        (void)p->serial_in(p, d->usr_reg);

        return 1;
    }

    return 0;
}

static void dw8250_clk_work_cb(struct work_struct *work)
{
    struct dw8250_data *d = work_to_dw8250_data(work);
    struct uart_8250_port *up;
    unsigned long rate;

    rate = clk_get_rate(d->clk);
    if (rate <= 0)
        return;

    up = serial8250_get_port(d->data.line);

    serial8250_update_uartclk(&up->port, rate);
}

static int dw8250_clk_notifier_cb(struct notifier_block *nb,
                  unsigned long event, void *data)
{
    struct dw8250_data *d = clk_to_dw8250_data(nb);

    /*
     * We have no choice but to defer the uartclk update due to two
     * deadlocks. First one is caused by a recursive mutex lock which
     * happens when clk_set_rate() is called from dw8250_set_termios().
     * Second deadlock is more tricky and is caused by an inverted order of
     * the clk and tty-port mutexes lock. It happens if clock rate change
     * is requested asynchronously while set_termios() is executed between
     * tty-port mutex lock and clk_set_rate() function invocation and
     * vise-versa. Anyway if we didn't have the reference clock alteration
     * in the dw8250_set_termios() method we wouldn't have needed this
     * deferred event handling complication.
     */
    if (event == POST_RATE_CHANGE) {
        queue_work(system_unbound_wq, &d->clk_work);
        return NOTIFY_OK;
    }

    return NOTIFY_DONE;
}

static void
dw8250_do_pm(struct uart_port *port, unsigned int state, unsigned int old)
{
    if (!state)
        pm_runtime_get_sync(port->dev);

    serial8250_do_pm(port, state, old);

    if (state)
        pm_runtime_put_sync_suspend(port->dev);
}

/**
 *  @brief abus_set_termios() - Driver termios callback
 *
 *  Extends the standard dw8250_set_termios()
 *  Stops all ABUS operations
 *  Mark the port ready for ABUS if TIO characteristics match with ABUS
 *
 *  We do a custom use of c_cc[NCC-1] passing the mode (0, 1 or 2)
 *
 *  @param p pointer to the uart port
 *  @param termios pointer to the new termios data
 *  @param old pointer to the old termios data
 *  @return void
 */
static void abus_set_termios(struct uart_port *p, struct ktermios *termios,
                   struct ktermios *old)
{
    unsigned long newrate = tty_termios_baud_rate(termios) * 16;
    struct dw8250_data *d = to_dw8250_data(p->private_data);
    long rate;
    int ret;

    FUNCTION();

    mutex_lock(&d->lock);

    if ((d-> flags & (ABUS_FLAG_MASTER | ABUS_FLAG_SLAVE)))
        abus_stop_operations(d);

    d->flags &= ~ABUS_FLAG_TIO_ABUS;

    clk_disable_unprepare(d->clk);
    rate = clk_round_rate(d->clk, newrate);
    if (rate > 0) {
        /*
         * Premilinary set the uartclk to the new clock rate so the
         * clock update event handler caused by the clk_set_rate()
         * calling wouldn't actually update the UART divisor since
         * we about to do this anyway.
         */
        swap(p->uartclk, rate);
        ret = clk_set_rate(d->clk, newrate);
        if (ret)
            swap(p->uartclk, rate);
    }
    clk_prepare_enable(d->clk);

    p->status &= ~UPSTAT_AUTOCTS;
    if (termios->c_cflag & CRTSCTS)
        p->status |= UPSTAT_AUTOCTS;

    serial8250_do_set_termios(p, termios, old);

    if ((termios->c_ispeed == 256000) && (termios->c_ospeed == 256000)) {
        printk(KERN_INFO "ABUS Termios set to ABUS characteristics, mode:%hhu\n", termios->c_cc[NCCS-1]);
        d->flags |= ABUS_FLAG_TIO_ABUS;

        d->flags &= ~(ABUS_FLAG_MODE_MASK);
        if (termios->c_cc[NCCS-1] == ABUS_MODE_MASTER)
            d->flags |= ABUS_FLAG_MASTER;
        else if (termios->c_cc[NCCS-1] == ABUS_MODE_ACTIVESLAVE)
            d->flags |= ABUS_FLAG_SLAVE;

        if ((d->flags & (ABUS_FLAG_MASTER | ABUS_FLAG_SLAVE)))
            abus_start_operations(d);
    };


    mutex_unlock(&d->lock);
}

static void dw8250_set_ldisc(struct uart_port *p, struct ktermios *termios)
{
    struct uart_8250_port *up = up_to_u8250p(p);
    unsigned int mcr = p->serial_in(p, UART_MCR);

    FUNCTION();

    if (up->capabilities & UART_CAP_IRDA) {
        if (termios->c_line == N_IRDA)
            mcr |= DW_UART_MCR_SIRE;
        else
            mcr &= ~DW_UART_MCR_SIRE;

        p->serial_out(p, UART_MCR, mcr);
    }
    serial8250_do_set_ldisc(p, termios);
}

/*
 * dw8250_fallback_dma_filter will prevent the UART from getting just any free
 * channel on platforms that have DMA engines, but don't have any channels
 * assigned to the UART.
 *
 * REVISIT: This is a work around for limitation in the DMA Engine API. Once the
 * core problem is fixed, this function is no longer needed.
 */
static bool dw8250_fallback_dma_filter(struct dma_chan *chan, void *param)
{
    return false;
}

static void dw8250_quirks(struct uart_port *p, struct dw8250_data *data)
{
    FUNCTION();

    if (p->dev->of_node) {
        struct device_node *np = p->dev->of_node;
        int id;

        /* get index of serial line, if found in DT aliases */
        id = of_alias_get_id(np, "serial");
        if (id >= 0)
            p->line = id;
    };
}

/**
 *  @brief abus_reset_stats() - Reset statistics
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static inline void abus_reset_stats(struct dw8250_data *d) {
    memset(&d->stats, 0, sizeof(struct abus_stats));
}

/**
 *  @brief stats_show() - SYSFS show statistics
 *
 *  Called to show /sys/class/tty/tty.../device/abus/stats
 *
 *  @param dev device driver
 *  @param attr attribute to be read (don't care, we have only 1 attr)
 *  @param buf return data buffer
 *  @return size of returned data in the buffer
 */
static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct dw8250_data      *d = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE - 1, "flags:%04x state:%lu sent:%lu received:%lu ticks:%lu slots:%lu echos:%lu-%u\niir => 0:%lu 1:%lu 2:%lu 3:%lu 4:%lu 5:%lu 6:%lu 7:%lu\n",
        d->flags,
        d->state,
        d->stats.bytes_sent,
        d->stats.bytes_received,
        d->stats.ticks,
        d->stats.slots,
        d->stats.echos,
        d->echos,
        d->stats.irq[0],
        d->stats.irq[1],
        d->stats.irq[2],
        d->stats.irq[3],
        d->stats.irq[4],
        d->stats.irq[5],
        d->stats.irq[6],
        d->stats.irq[7]
    );

}

/**
 *  @brief stats_store() - SYSFS store statistics
 *
 *  Called to change /sys/class/tty/tty.../device/abus/stats
 *  Reset the statistics regardless of what is written
 *
 *  @param dev device driver
 *  @param attr attribute to be read (don't care, we have only 1 attr)
 *  @param buf return data buffer
 *  @param count size of data in the buffer
 *  @return size of written data. < 0 if error (mode non numeric)
 */
static ssize_t stats_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct dw8250_data *d = dev_get_drvdata(dev);
    abus_reset_stats(d);
    return count;
}

static DEVICE_ATTR_RW(stats);

/**
 *  @brief echo_show() - SYSFS show echo
 *
 *  Called to show /sys/class/tty/tty.../device/abus/echo
 *
 *  @param dev device driver
 *  @param attr attribute to be read (don't care, we have only 1 attr)
 *  @param buf return data buffer
 *  @return size of returned data in the buffer
 */
static ssize_t echo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct dw8250_data *d = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE - 1, "%u\n", ((d->flags & ABUS_FLAG_ECHOING) ? 1 : 0));
}

static DEVICE_ATTR_RO(echo);

static struct attribute *abus_attrs[] = {
    &dev_attr_stats.attr,
    &dev_attr_echo.attr,
    NULL,
};

static const struct attribute_group abus_attr_group = {
    .attrs = abus_attrs,
    .name = "abus",
};

/**
 *  @brief abus_stop_sending() - Stop sending
 *
 *  TIMER context
 *  Will reset the RS485 to receive mode because the timer has expired
 *  The timer is not restarted (it will be restarted with next frame)
 *
 *  @param timer
 *  @return HRTIMER_NORESTART
 */
static enum hrtimer_restart abus_stop_sending(struct hrtimer *timer) {
    struct dw8250_data      *d = container_of(timer, struct dw8250_data, stop_sending_timer);
    struct uart_8250_port   *up = serial8250_get_port(d->data.line);
    struct uart_port        *p = &up->port;
    unsigned long           flags;

    abus_transceiver_rx(d);

    spin_lock_irqsave(&p->lock, flags);
    d->sending = 0;
    if ((d->flags & ABUS_FLAG_ECHOED)) {
        if ((++d->echos >= ABUS_MAX_ECHOS) && !(d->flags & ABUS_FLAG_ECHOING)) {
            printk(KERN_ERR "ABUS detected %u consecutive echos. Issue with SCBIP\n", ABUS_MAX_ECHOS);
            d->flags |= ABUS_FLAG_ECHOING;
        };
    };
    spin_unlock_irqrestore(&p->lock, flags);

    return HRTIMER_NORESTART;
};

/**
 *  @brief abus_send_frame() - Sending a new frame
 *
 *  Forces RS485 to transmit, and starts its timer
 *  Send the first packet
 *
 *  @param d pointer to the driver private data
 *  @return void
 */
static void abus_send_frame(struct dw8250_data *d) {
    struct uart_8250_port   *up = serial8250_get_port(d->data.line);
    struct uart_port        *p = &up->port;
    unsigned long           flags;

    d->stats.slots++;
    abus_transceiver_tx(d);
    hrtimer_start(&d->stop_sending_timer, ABUS_KEEP_RTS, HRTIMER_MODE_REL);

    spin_lock_irqsave(&p->lock, flags);
    d->sending = ABUS_FRAME_LENGTH;
    d->flags &= ~ ABUS_FLAG_ECHOED;
    abus_send_packet(d, p, up);
    spin_unlock_irqrestore(&p->lock, flags);
};

/**
 *  @brief abus_check_sending() - Check for sending a new frame
 *
 *  TIMER context
 *  if MASTER : send a new frame
 *  if SLAVE : check is long enought was waited after the previous receive
 *
 *  Normally called only if MASTER or SLAVE, so timer is restarted
 *  In case a race competition happend and the mode is no more in ABUS
 *  the timer is not restarted
 *
 *  @param timer
 *  @return HRTIMER_RESTART
 */
static enum hrtimer_restart abus_check_sending(struct hrtimer *timer) {
    struct dw8250_data *d = container_of(timer, struct dw8250_data, check_sending_timer);

    d->stats.ticks++;

    if ((d->flags & ABUS_FLAG_MASTER)) {
        abus_send_frame(d);
        hrtimer_forward_now(timer, ABUS_MASTER_TIMER);
        return HRTIMER_RESTART;
    };

    if ((d->flags & ABUS_FLAG_SLAVE)) {
        if (d->state) {
            // TODO : protect
            d->state--;
            if (d->state == 0) {
                abus_send_frame(d);
            }
        };
        hrtimer_forward_now(timer, ABUS_SLAVE_TIMER);
        return HRTIMER_RESTART;
    };

    return HRTIMER_NORESTART;
};

/**
 *  @brief abus_startup() - Driver startup callback
 *
 *  Does the standard 8250 startup
 *  Indicates the port is opened
 *
 *  @param p pointer to the uart port
 *  @return the return value of the standard 8250 startup
 */
static int abus_startup(struct uart_port *p) {
    struct dw8250_data  *d = to_dw8250_data(p->private_data);
    int                 ret;

    FUNCTION();

    mutex_lock(&d->lock);

    d->flags |= ABUS_FLAG_OPENED;

    ret = serial8250_do_startup(p);

    mutex_unlock(&d->lock);

    return ret;
}

/**
 *  @brief abus_shutdown() - Driver shutdown callback
 *
 *  Stops ABUS communications
 *  Does the standard 8250 shutdown
 *  Indicates the port is not opened and will reneed a correct TIO later
 *
 *  @param p pointer to the uart port
 *  @return the return value of the standard 8250 startup
 */
static void abus_shutdown(struct uart_port *p) {
    struct dw8250_data *d = to_dw8250_data(p->private_data);

    FUNCTION();

    mutex_lock(&d->lock);

    if ((d->flags & (ABUS_FLAG_MASTER | ABUS_FLAG_SLAVE)))
        abus_stop_operations(d);

    d->flags &= ~(ABUS_FLAG_OPENED | ABUS_FLAG_TIO_ABUS);

    serial8250_do_shutdown(p);

    mutex_unlock(&d->lock);
}

/**
 *  @brief abus_probe() - Driver probe
 *
 *  Superseeds dw8250_probe()
 *  Adds special abus callbacks
 *  Disables DMA
 *  Create SYSFS group and file (stats + mode)
 *  define ABUS objects (timers, mutex, GPIO)
 *
 *  @param pdev pointer to platform driver
 *  @return the return value of the standard 8250 startup
 */
static int abus_probe(struct platform_device *pdev)
{
    struct uart_8250_port uart = {}, *up = &uart;
    struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    struct uart_port *p = &up->port;
    struct device *dev = &pdev->dev;
    struct dw8250_data *d;
    int irq;
    int err;
    u32 val;

    FUNCTION();

    if (!regs) {
        dev_err(dev, "no registers defined\n");
        return -EINVAL;
    }

    irq = platform_get_irq(pdev, 0);
    if (irq < 0)
        return irq;

    spin_lock_init(&p->lock);
    p->mapbase      = regs->start;
    p->irq          = irq;
    p->handle_irq   = abus_handle_irq;
    p->pm           = dw8250_do_pm;
    p->type         = PORT_8250;
    p->flags        = UPF_SHARE_IRQ | UPF_FIXED_PORT;
    p->dev          = dev;
    p->iotype       = UPIO_MEM;
    p->set_ldisc    = dw8250_set_ldisc;
    p->set_termios  = abus_set_termios;
    p->iotype       = UPIO_MEM32;
    p->serial_in    = dw8250_serial_in32;
    p->serial_out   = dw8250_serial_out32;
    p->startup      = abus_startup;
    p->shutdown     = abus_shutdown;

    p->membase = devm_ioremap(dev, regs->start, resource_size(regs));
    if (!p->membase)
        return -ENOMEM;

    d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
    if (!d)
        return -ENOMEM;

    d->data.dma.fn = dw8250_fallback_dma_filter;
    d->usr_reg = DW_UART_USR;
    p->private_data = &d->data;

    d->uart_16550_compatible = device_property_read_bool(dev,
                        "snps,uart-16550-compatible");

    err = device_property_read_u32(dev, "reg-shift", &val);
    if (!err)
        p->regshift = val;

    /* Always ask for fixed clock rate from a property. */
    device_property_read_u32(dev, "clock-frequency", &p->uartclk);

    /* If there is separate baudclk, get the rate from it. */
    d->clk = devm_clk_get_optional(dev, "baudclk");
    if (d->clk == NULL)
        d->clk = devm_clk_get_optional(dev, NULL);
    if (IS_ERR(d->clk))
        return PTR_ERR(d->clk);

    INIT_WORK(&d->clk_work, dw8250_clk_work_cb);
    d->clk_notifier.notifier_call = dw8250_clk_notifier_cb;

    err = clk_prepare_enable(d->clk);
    if (err)
        dev_warn(dev, "could not enable optional baudclk: %d\n", err);

    if (d->clk)
        p->uartclk = clk_get_rate(d->clk);

    /* If no clock rate is defined, fail. */
    if (!p->uartclk) {
        dev_err(dev, "clock rate not defined\n");
        err = -EINVAL;
        goto err_clk;
    }

    d->pclk = devm_clk_get_optional(dev, "apb_pclk");
    if (IS_ERR(d->pclk)) {
        err = PTR_ERR(d->pclk);
        goto err_clk;
    }

    err = clk_prepare_enable(d->pclk);
    if (err) {
        dev_err(dev, "could not enable apb_pclk\n");
        goto err_clk;
    }

    d->rst = devm_reset_control_get_optional_exclusive(dev, NULL);
    if (IS_ERR(d->rst)) {
        err = PTR_ERR(d->rst);
        goto err_pclk;
    }
    reset_control_deassert(d->rst);

    dw8250_quirks(p, d);

    if (!d->skip_autocfg)
        dw8250_setup_port(p);


    up->dma = NULL;

    d->data.line = serial8250_register_8250_port(up);
    if (d->data.line < 0) {
        err = d->data.line;
        goto err_reset;
    }

    /*
     * Some platforms may provide a reference clock shared between several
     * devices. In this case any clock state change must be known to the
     * UART port at least post factum.
     */
    if (d->clk) {
        err = clk_notifier_register(d->clk, &d->clk_notifier);
        if (err)
            dev_warn(p->dev, "Failed to set the clock notifier\n");
        else
            queue_work(system_unbound_wq, &d->clk_work);
    }

    err = sysfs_create_group(&dev->kobj, &abus_attr_group);
    if (err < 0)
        return err;

    d->flags = 0x0000;

    mutex_init(&d->lock);

    hrtimer_init(&d->check_sending_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    d->check_sending_timer.function = abus_check_sending;

    hrtimer_init(&d->stop_sending_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    d->stop_sending_timer.function = abus_stop_sending;

    gpio_request(ABUS_WRITE_GPIO, "UART4 TX/RX");
    gpio_direction_output(ABUS_WRITE_GPIO, ABUS_WRITE_OFF);

    platform_set_drvdata(pdev, d);

    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);

    return 0;

err_reset:
    reset_control_assert(d->rst);

err_pclk:
    clk_disable_unprepare(d->pclk);

err_clk:
    clk_disable_unprepare(d->clk);

    return err;
}

/**
 *  @brief abus_remove() - Driver remove
 *
 *  Superseeds dw8250_remove()
 *  Delete SYSFS group and file (stats + mode)
 *  Deletes/Frees ABUS objects (timers, mutex, GPIO)
 *
 *  @param pdev pointer to platform driver
 *  @return the return value of the standard 8250 startup
 */
static int abus_remove(struct platform_device *pdev)
{
    struct dw8250_data *d = platform_get_drvdata(pdev);
    struct device *dev = &pdev->dev;

    pm_runtime_get_sync(dev);

    if (d->clk) {
        clk_notifier_unregister(d->clk, &d->clk_notifier);

        flush_work(&d->clk_work);
    }

    serial8250_unregister_port(d->data.line);

    gpio_free(ABUS_WRITE_GPIO);

    hrtimer_cancel(&d->stop_sending_timer);
    hrtimer_cancel(&d->check_sending_timer);

    sysfs_remove_group(&dev->kobj, &abus_attr_group);

    reset_control_assert(d->rst);

    clk_disable_unprepare(d->pclk);

    clk_disable_unprepare(d->clk);

    pm_runtime_disable(dev);
    pm_runtime_put_noidle(dev);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dw8250_suspend(struct device *dev)
{
    struct dw8250_data *d = dev_get_drvdata(dev);

    serial8250_suspend_port(d->data.line);

    return 0;
}

static int dw8250_resume(struct device *dev)
{
    struct dw8250_data *d = dev_get_drvdata(dev);

    serial8250_resume_port(d->data.line);

    return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int dw8250_runtime_suspend(struct device *dev)
{
    struct dw8250_data *d = dev_get_drvdata(dev);

    clk_disable_unprepare(d->clk);

    clk_disable_unprepare(d->pclk);

    return 0;
}

static int dw8250_runtime_resume(struct device *dev)
{
    struct dw8250_data *d = dev_get_drvdata(dev);

    clk_prepare_enable(d->pclk);

    clk_prepare_enable(d->clk);

    return 0;
}
#endif

static const struct dev_pm_ops dw8250_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(dw8250_suspend, dw8250_resume)
    SET_RUNTIME_PM_OPS(dw8250_runtime_suspend, dw8250_runtime_resume, NULL)
};

static const struct of_device_id dw8250_of_match[] = {
    { .compatible = "snps,dw-hon-abus-uart" },
    { /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw8250_of_match);

static const struct acpi_device_id dw8250_acpi_match[] = {
    { "INT33C4", 0 },
    { "INT33C5", 0 },
    { "INT3434", 0 },
    { "INT3435", 0 },
    { "80860F0A", 0 },
    { "8086228A", 0 },
    { "APMC0D08", 0},
    { "AMD0020", 0 },
    { "AMDI0020", 0 },
    { "BRCM2032", 0 },
    { "HISI0031", 0 },
    { },
};
MODULE_DEVICE_TABLE(acpi, dw8250_acpi_match);

static struct platform_driver dw8250_hon_abus_platform_driver = {
    .driver = {
        .name       = "dw-hon-abus-uart",
        .pm     = &dw8250_pm_ops,
        .of_match_table = dw8250_of_match,
        .acpi_match_table = ACPI_PTR(dw8250_acpi_match),
    },
    .probe          = abus_probe,
    .remove         = abus_remove,
};

module_platform_driver(dw8250_hon_abus_platform_driver);

MODULE_AUTHOR("Jean-Marc Bonzom");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Honewell 8250 serial port driver with ABus support");
MODULE_ALIAS("platform:dw-hon-abus-uart");
