#include <xc.h>
#include "uart.h"
#include "cbuffer.h"

// Interrupt-driven UART1 with separate RX and TX circular buffers
#define UART1_BRG_VALUE 467

// Separate RX and TX buffer sizes
#define TX_BUFFER_SIZE 256
#define RX_BUFFER_SIZE 64

static char tx_storage[TX_BUFFER_SIZE];
static char rx_storage[RX_BUFFER_SIZE];

static circular_buffer rx_buffer[1];
static circular_buffer tx_buffer[1];

// Set when the RX buffer overflows
static volatile int rx_overflow_flag = 0;

// Fill the TX FIFO from the circular buffer
static void uart1_kick_tx(void) {
    while ((cb_count(tx_buffer) > 0) && (U1STAbits.UTXBF == 0)) {
        U1TXREG = cb_pop(tx_buffer);
    }

    if (cb_count(tx_buffer) > 0) {
        IEC0bits.U1TXIE = 1;
    }
}

void uart1_init(void) {
    cb_init(rx_buffer, rx_storage, RX_BUFFER_SIZE);
    cb_init(tx_buffer, tx_storage, TX_BUFFER_SIZE);

    RPINR18bits.U1RXR = 75;
    RPOR0bits.RP64R = 1;

    U1MODE = 0;
    U1STA = 0;
    U1BRG = UART1_BRG_VALUE;

    // 8-bit data, no parity, 1 stop bit, standard baud, TX/RX only.
    U1MODEbits.STSEL = 0;
    U1MODEbits.PDSEL = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.UEN = 0;

    // Interrupt when a character is received / when there is room to transmit
    U1STAbits.URXISEL = 0;
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;

    // Flush any stale received bytes
    while (U1STAbits.URXDA) {
        volatile char dummy = U1RXREG;
        (void)dummy;
    }
    U1STAbits.OERR = 0;

    // Clear flags, enable RX interrupt
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
    IEC0bits.U1RXIE = 1;
    IEC0bits.U1TXIE = 0;

    // Enable the module and the transmitter.
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
}

// Queue one byte safely with respect to the TX ISR
void uart1_send_char(char c) {
    int tx_was_enabled = IEC0bits.U1TXIE;

    IEC0bits.U1TXIE = 0;

    if (cb_push(tx_buffer, c) == 0) {
        // Drop byte if the TX buffer is full
        IEC0bits.U1TXIE = tx_was_enabled;
        return;
    }

    uart1_kick_tx();
}

void uart1_send_string(const char *s) {
    while (*s != '\0') {
        uart1_send_char(*s);
        s++;
    }
}

// Number of bytes waiting in the RX buffer (0 means nothing to read)
int uart1_available(void) {
    return cb_count(rx_buffer);
}

char uart1_read(void) {
    return cb_pop(rx_buffer);
}

// Buffer occupancy, used for the $MBUF report.
int uart1_tx_count(void) {
    return cb_count(tx_buffer);
}

int uart1_rx_count(void) {
    return cb_count(rx_buffer);
}

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;

    while (U1STAbits.URXDA) {
        if (cb_push(rx_buffer, U1RXREG) == 0) {
            rx_overflow_flag = 1;   // buffer full, byte lost
        }
    }

    // An overrun stops reception until cleared, so always clear it
    if (U1STAbits.OERR) {
        U1STAbits.OERR = 0;
    }
}

void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;

    while ((U1STAbits.UTXBF == 0) && (cb_count(tx_buffer) > 0)) {
        U1TXREG = cb_pop(tx_buffer);
    }

    if (cb_count(tx_buffer) == 0) {
        IEC0bits.U1TXIE = 0;
    }
}
