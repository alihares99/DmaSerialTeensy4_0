//
// Created by Ali on ۱۲/۱۱/۲۰۱۹.
//

#include "DmaSerialTeensy.h"
#include "Arduino.h"
#include <cstring>
#include <cmath>

#define MIN(x, y)  ((x) > (y) ? (y) : (x))
#define MAX(x, y)  ((x) > (y) ? (x) : (y))

#ifdef __IMXRT1062__  // teensy 4.0

#define IRQ_PRIORITY  64  // 0 = highest priority, 255 = lowest
#define UART_CLOCK 24000000

#define CTRL_ENABLE 		(LPUART_CTRL_TE | LPUART_CTRL_RE)

// *********************************************************************************************************
// ** There is a fatal bug in imxrt.h that DMAMUX_SOURCE_LPUARTx_RX and DMAMUX_SOURCE_LPUARTx_TX values
// ** are incorrect. They should swap their values in imxrt.h file. This bug took me 6 hours to find!
// ** This is probably because this library was originally written for imxrt1052 micro and they forgot
// ** to add the necessary changes to support imxrt1062 which is the micro on teensy 4.0.
// *********************************************************************************************************

// teensy 4.0 specific board information:
const DmaSerialTeensy::Base_t DmaSerialTeensy::serial1Base = {
        0,
        &IMXRT_LPUART6,
        IRQ_LPUART6,
        #ifdef __IMXRT1062__  // teensy 4.0
        71, // DMAMUX_SOURCE_LPUART6_RX, // this value is incorrect in imxrt.h file
        70, // DMAMUX_SOURCE_LPUART6_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR3,
        CCM_CCGR3_LPUART6(CCM_CCGR_ON),
        {{0,2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
        {{1,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        0xff, // No CTS pin
        0, // No CTS
        IRQ_PRIORITY,
        38,
        24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t DmaSerialTeensy::serial2Base = {
        1,
        &IMXRT_LPUART4,
        IRQ_LPUART4,
        #ifdef __IMXRT1062__  // teensy 4.0
        69, //DMAMUX_SOURCE_LPUART4_RX, // this value is incorrect in imxrt.h file
        68, // DMAMUX_SOURCE_LPUART4_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR1,
        CCM_CCGR1_LPUART4(CCM_CCGR_ON),
        #if defined(__IMXRT1052__)
        {{6,2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}},
	    {{7,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        #elif defined(__IMXRT1062__)
        {{7,2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}},
        {{8,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        #endif
        0xff, // No CTS pin
        0, // No CTS
        IRQ_PRIORITY,
        38,
        24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t DmaSerialTeensy::serial3Base = {
        2,
        &IMXRT_LPUART2,
        IRQ_LPUART2,
        #ifdef __IMXRT1062__  // teensy 4.0
        67, // DMAMUX_SOURCE_LPUART2_RX, // this value is incorrect in imxrt.h file
        66, // DMAMUX_SOURCE_LPUART2_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR0,
        CCM_CCGR0_LPUART2(CCM_CCGR_ON),
        {{15,2, &IOMUXC_LPUART2_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
        {{14,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        18, //IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01, // 18
        2, // page 473
        IRQ_PRIORITY,
        38,
        24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t DmaSerialTeensy::serial4Base = {
        3,
        &IMXRT_LPUART3,
        IRQ_LPUART3,
        #ifdef __IMXRT1062__  // teensy 4.0
        5, // DMAMUX_SOURCE_LPUART3_RX, // this value is incorrect in imxrt.h file
        4, // DMAMUX_SOURCE_LPUART3_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR0,
        CCM_CCGR0_LPUART3(CCM_CCGR_ON),
        {{16,2, &IOMUXC_LPUART3_RX_SELECT_INPUT, 0}, {0xff, 0xff, nullptr, 0}},
        {{17,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        0xff, // No CTS pin
        0, // No CTS
        IRQ_PRIORITY,
        38,
        24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t DmaSerialTeensy::serial5Base = {
        4,
        &IMXRT_LPUART8,
        IRQ_LPUART8,
        #ifdef __IMXRT1062__  // teensy 4.0
        73, // DMAMUX_SOURCE_LPUART8_RX, // this value is incorrect in imxrt.h file
        72, // DMAMUX_SOURCE_LPUART8_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR6,
        CCM_CCGR6_LPUART8(CCM_CCGR_ON),
        {{21,2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 1}, {38, 2, &IOMUXC_LPUART8_RX_SELECT_INPUT, 0}},
        {{20,2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 1}, {39, 2, &IOMUXC_LPUART8_TX_SELECT_INPUT, 0}},
        0xff, // No CTS pin
        0, // No CTS
        IRQ_PRIORITY,
        38,
        24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t DmaSerialTeensy::serial6Base = {
        5,
        &IMXRT_LPUART1,
        IRQ_LPUART1,
        #ifdef __IMXRT1062__  // teensy 4.0
        3, //DMAMUX_SOURCE_LPUART1_RX, // this value is incorrect in imxrt.h file
        2, // DMAMUX_SOURCE_LPUART1_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR5,
        CCM_CCGR5_LPUART1(CCM_CCGR_ON),
        {{25,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        {{24,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        0xff, // No CTS pin
        0, // No CTS
        IRQ_PRIORITY,
        38,
        24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t DmaSerialTeensy::serial7Base = {
        6,
        &IMXRT_LPUART7,
        IRQ_LPUART7,
        #ifdef __IMXRT1062__  // teensy 4.0
        9, // DMAMUX_SOURCE_LPUART7_RX, // this value is incorrect in imxrt.h file
        8, // DMAMUX_SOURCE_LPUART7_TX, // this value is incorrect in imxrt.h file
        #endif
        CCM_CCGR5,
        CCM_CCGR5_LPUART7(CCM_CCGR_ON),
        {{28,2, &IOMUXC_LPUART7_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
        {{29,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
        0xff, // No CTS pin
        0, // No CTS
        IRQ_PRIORITY, 38, 24, // IRQ, rts_low_watermark, rts_high_watermark
};

const DmaSerialTeensy::Base_t* DmaSerialTeensy::allSerialBases[7] = {
        &serial1Base,
        &serial2Base,
        &serial3Base,
        &serial4Base,
        &serial5Base,
        &serial6Base,
        &serial7Base,
};

DmaSerialTeensy dmaSerial1(1);
DmaSerialTeensy dmaSerial2(2);
DmaSerialTeensy dmaSerial3(3);
DmaSerialTeensy dmaSerial4(4);
DmaSerialTeensy dmaSerial5(5);
DmaSerialTeensy dmaSerial6(6);
DmaSerialTeensy dmaSerial7(7);
const DmaSerialTeensy* DmaSerialTeensy::dmaSerials[7] = {
        &dmaSerial1,
        &dmaSerial2,
        &dmaSerial3,
        &dmaSerial4,
        &dmaSerial5,
        &dmaSerial6,
        &dmaSerial7,
};
#else
#error "no supported board"
#endif

void DmaSerialTeensy::txCompleteCallback1() {dmaSerial1.txIsr();}
void DmaSerialTeensy::txCompleteCallback2() {dmaSerial2.txIsr();}
void DmaSerialTeensy::txCompleteCallback3() {dmaSerial3.txIsr();}
void DmaSerialTeensy::txCompleteCallback4() {dmaSerial4.txIsr();}
void DmaSerialTeensy::txCompleteCallback5() {dmaSerial5.txIsr();}
void DmaSerialTeensy::txCompleteCallback6() {dmaSerial6.txIsr();}
void DmaSerialTeensy::txCompleteCallback7() {dmaSerial7.txIsr();}
void (* const DmaSerialTeensy::allTxIsr[7])() = {
        txCompleteCallback1,
        txCompleteCallback2,
        txCompleteCallback3,
        txCompleteCallback4,
        txCompleteCallback5,
        txCompleteCallback6,
        txCompleteCallback7,
};

void DmaSerialTeensy::rxCompleteCallback1() {dmaSerial1.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback2() {dmaSerial2.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback3() {dmaSerial3.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback4() {dmaSerial4.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback5() {dmaSerial5.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback6() {dmaSerial6.rxIsr();}
void DmaSerialTeensy::rxCompleteCallback7() {dmaSerial7.rxIsr();}
void (* const DmaSerialTeensy::allRxIsr[7])() = {
        rxCompleteCallback1,
        rxCompleteCallback2,
        rxCompleteCallback3,
        rxCompleteCallback4,
        rxCompleteCallback5,
        rxCompleteCallback6,
        rxCompleteCallback7,
};

DmaSerialTeensy::DmaSerialTeensy(int serialNo)
    : serialNo(serialNo)
{
    serialBase = allSerialBases[serialNo - 1];
    rxPinIndex = 0; // default pin = first pin
    txPinIndex = 0; // default pin = first pin

    txBufferTail = 0;
    txBufferHead = 0;
    txBufferCount = 0;
    rxBufferTail = 0;
    // rxBufferHead = 0; // no need for this
}


void DmaSerialTeensy::begin(uint32_t baud, uint16_t format) {

    // configure DMA channels:
    if (!dmaChannelSend) {
        dmaChannelSend = new DMAChannel();
        dmaChannelSend->destination(*(uint8_t*)&serialBase->port->DATA);
        // source is not configured here
        dmaChannelSend->triggerAtHardwareEvent(serialBase->dmaMuxSourceTx);
        dmaChannelSend->attachInterrupt(allTxIsr[serialNo - 1]);
        dmaChannelSend->interruptAtCompletion();
        dmaChannelSend->disableOnCompletion();
        // not enabled here
    }
    if (!dmaChannelReceive) {
        dmaChannelReceive = new DMAChannel();
        dmaChannelReceive->source(*(uint8_t*)&serialBase->port->DATA);
        dmaChannelReceive->destinationBuffer(rxBuffer, DMA_RX_BUFFER_SIZE);
        dmaChannelReceive->triggerAtHardwareEvent(serialBase->dmaMuxSourceRx);
        // no need for Rx interrupt:
        // dmaChannelReceive->attachInterrupt(allRxIsr[serialNo - 1]);
        // dmaChannelReceive->interruptAtCompletion();
        dmaChannelReceive->enable();
    }

    txBufferTail = 0;
    txBufferHead = 0;
    txBufferCount = 0;
    rxBufferTail = 0;
    // rxBufferHead = 0; // no need for this

    // HARES calculate baudrate:
    float base = (float)UART_CLOCK / (float)baud;
    float besterr = 1e20;
    int bestdiv = 1;
    int bestosr = 4;
    for (int osr=4; osr <= 32; osr++) {
        float div = base / (float)osr;
        int divint = lroundf(div);
        if (divint < 1) divint = 1;
        else if (divint > 8191) divint = 8191;
        float err = ((float)divint - div) / div;
        if (err < 0.0f) err = -err;
        if (err <= besterr) {
            besterr = err;
            bestdiv = divint;
            bestosr = osr;
        }
    }

    // Hardcoded values for dmaSerial1. Just for test.
    /*
    CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);
    IMXRT_LPUART6.CTRL = 0;
    *(portControlRegister(0)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(0)) = 2;
    IOMUXC_LPUART6_RX_SELECT_INPUT = 1;

    // config Tx Pin:
    *(portControlRegister(1)) =  IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(1)) = 2;
    // IOMUXC_LPUART6_TX_SELECT_INPUT = 1;

    serial1Base.port->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv);
    serial1Base.port->PINCFG = 0;
    serial1Base.port->BAUD |= (LPUART_BAUD_TDMAE | LPUART_BAUD_RDMAE | (1 << 20));
    serial1Base.port->FIFO &= ~(LPUART_FIFO_TXFE | LPUART_FIFO_RXFE);

    uint32_t ctrl = CTRL_ENABLE;// | LPUART_CTRL_RIE | LPUART_CTRL_TIE | LPUART_CTRL_ILIE;
    serial1Base.port->CTRL = ctrl;
    */

    // turn on clock for UART:
    serialBase->ccm_register |= serialBase->ccm_value;

    // disable UART:
    serialBase->port->CTRL = 0;

    // config Rx Pin:
    *(portControlRegister(serialBase->rx_pins[rxPinIndex].pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(serialBase->rx_pins[rxPinIndex].pin)) = serialBase->rx_pins[rxPinIndex].mux_val;
    if (serialBase->rx_pins[rxPinIndex].select_input_register) {
        *(serialBase->rx_pins[rxPinIndex].select_input_register) =  serialBase->rx_pins[rxPinIndex].select_val;
    }

    // config Tx Pin:
    *(portControlRegister(serialBase->tx_pins[txPinIndex].pin)) =  IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(serialBase->tx_pins[txPinIndex].pin)) = serialBase->tx_pins[txPinIndex].mux_val;

    serialBase->port->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv);
    serialBase->port->PINCFG = 0;

    // HARES: disabling the interrupt parts:
    // Enable the transmitter, receiver and enable receiver interrupt
    // attachInterruptVector(serialBase->irq, serialBase->irq_handler);
    // NVIC_SET_PRIORITY(serialBase->irq, serialBase->irq_priority);	// maybe should put into hardware...
    // NVIC_ENABLE_IRQ(serialBase->irq);

    // HARES: enabling DMA instead:
    serialBase->port->BAUD |= (LPUART_BAUD_TDMAE | LPUART_BAUD_RDMAE);

    // HARES: it seems that the rx_fifo_size and tx_fifo_size differs from the actual size based on the datasheet
    // uint16_t tx_fifo_size = (((serialBase->port->FIFO >> 4) & 0x7) << 2);
    // uint8_t tx_water = (tx_fifo_size < 16) ? tx_fifo_size >> 1 : 7;
    // uint16_t rx_fifo_size = (((serialBase->port->FIFO >> 0) & 0x7) << 2);
    // uint8_t rx_water = (rx_fifo_size < 16) ? rx_fifo_size >> 1 : 7;
    // serialBase->port->WATER = LPUART_WATER_RXWATER(rx_water) | LPUART_WATER_TXWATER(tx_water);
    // serialBase->port->FIFO |= LPUART_FIFO_TXFE | LPUART_FIFO_RXFE;

    // HARES: disabling FIFO:
    serialBase->port->FIFO &= ~(LPUART_FIFO_TXFE | LPUART_FIFO_RXFE);

    // lets configure up our CTRL register value
    uint32_t ctrl = CTRL_ENABLE | LPUART_CTRL_RIE | LPUART_CTRL_TIE | LPUART_CTRL_ILIE;

    // Now process the bits in the Format value passed in
    // Bits 0-2 - Parity plus 9  bit.
    ctrl |= (format & (LPUART_CTRL_PT | LPUART_CTRL_PE) );	// configure parity - turn off PT, PE, M and configure PT, PE
    if (format & 0x04) ctrl |= LPUART_CTRL_M;		// 9 bits (might include parity)
    if ((format & 0x0F) == 0x04) ctrl |=  LPUART_CTRL_R9T8; // 8N2 is 9 bit with 9th bit always 1

    // Bit 5 TXINVERT
    if (format & 0x20) ctrl |= LPUART_CTRL_TXINV;		// tx invert

    // Bit 3 10 bit - Will assume that begin already cleared it.
    // process some other bits which change other registers.
    if (format & 0x08) 	serialBase->port->BAUD |= LPUART_BAUD_M10;

    // Bit 4 RXINVERT
    uint32_t c = serialBase->port->STAT & ~LPUART_STAT_RXINV;
    if (format & 0x10) c |= LPUART_STAT_RXINV;		// rx invert
    serialBase->port->STAT = c;

    // bit 8 can turn on 2 stop bit mote
    if ( format & 0x100) serialBase->port->BAUD |= LPUART_BAUD_SBNS;

    // write out computed CTRL and turn on UART transmit and receive
    serialBase->port->CTRL = ctrl;

}

/**
 * Number of bytes in the buffer
 * @return 0 to 2047
 */
int DmaSerialTeensy::available() {
    int biter = dmaChannelReceive->TCD->BITER;
    int citer = dmaChannelReceive->TCD->CITER;
    int csr = dmaChannelReceive->TCD->CSR;
    if (csr & 0x80) { // done so Rx buffer is full
        if (rxBufferTail == 0) return 0;
        else return DMA_RX_BUFFER_SIZE - rxBufferTail;
    }
    else {
        // our version of buffer indexes are not update
        int head = biter - citer;
        if (head >= rxBufferTail) return head - rxBufferTail;
        else return head - rxBufferTail + DMA_RX_BUFFER_SIZE;
    }
}

int DmaSerialTeensy::read() {
    uint8_t c = rxBuffer[rxBufferTail++];
    if (rxBufferTail >= DMA_RX_BUFFER_SIZE)
        rxBufferTail -= DMA_RX_BUFFER_SIZE;
    return c;
}

int DmaSerialTeensy::peek() {
    return rxBuffer[rxBufferTail];
}

size_t DmaSerialTeensy::write(uint8_t c) {
    write(&c, 1);
    return 1;
}

size_t DmaSerialTeensy::write(char c) {
    return write((uint8_t *)&c, 1);
}

size_t DmaSerialTeensy::write(const uint8_t *p, size_t len) {

    int index = 0;
    while (index < len) {

        // wait until there is free space in the buffer:
        while (DMA_TX_BUFFER_SIZE - txBufferCount == 0); //

        // get a chunk of data to add to the buffer
        int chunkSize = MIN(len - index, DMA_TX_BUFFER_SIZE - txBufferCount);

        // copy the data to the buffer:
        int s1 = MIN(chunkSize, DMA_TX_BUFFER_SIZE - txBufferHead);
        int s2 = chunkSize - s1;
        memcpy(&txBuffer[txBufferHead], &p[index], s1);
        if (s2 > 0)
            memcpy(&txBuffer[0], &p[index + s1], s2);
        index += chunkSize;

        // move the head:
        txBufferCount += chunkSize;
        txBufferHead += chunkSize;
        if (txBufferHead >= DMA_TX_BUFFER_SIZE)
            txBufferHead -= DMA_TX_BUFFER_SIZE;

        // start transmitting from the tail:
        if (!transmitting) {
            transmitting = true;
            __disable_irq()
            int count = MIN(DMA_TX_BUFFER_SIZE - txBufferTail, chunkSize);
            count = MIN(count, DMA_MAX_BURST_DATA_TRANSFER); // MIN(remaining in the buffer, len_truncate, max_burst)
            dmaChannelSend->sourceBuffer(&txBuffer[txBufferTail], count);
            dmaChannelSend->enable();
            __enable_irq();
        }
    }
    return len;

}

void DmaSerialTeensy::rxIsr() {
    dmaChannelReceive->clearInterrupt();

    // no need for the Rx interrupt
    /*
    // move the head:
    int count = dmaChannelReceive->TCD->BITER;
    rxBufferHead += count;
    if (rxBufferHead >= DMA_RX_BUFFER_SIZE)
        rxBufferHead -= DMA_RX_BUFFER_SIZE;
    */

}

void DmaSerialTeensy::txIsr() {
    dmaChannelSend->clearInterrupt();

    // move the tail:
    {
        int count = dmaChannelSend->TCD->BITER;

        txBufferTail += count;
        if (txBufferTail >= DMA_TX_BUFFER_SIZE)
            txBufferTail -= DMA_TX_BUFFER_SIZE;
        txBufferCount -= count;
    }

    if (txBufferCount > 0) {
        transmitting = true;
        __disable_irq()
        int count = MIN(DMA_TX_BUFFER_SIZE - txBufferTail, txBufferCount);
        count = MIN(count, DMA_MAX_BURST_DATA_TRANSFER); // MIN(remaining in the buffer, txBufferCount, max_burst)
        dmaChannelSend->sourceBuffer(&txBuffer[txBufferTail], count);
        dmaChannelSend->enable();
        __enable_irq();
    }
    else {
        transmitting = false;
    }
}


