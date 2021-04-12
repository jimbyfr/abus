# abus

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

