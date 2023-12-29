/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#pragma once

#include "core_pins.h"
#include "HardwareSerial.h"
#include "DMAChannel.h"

// ------------------- change the following if you want ---------------------- //
#define DMA_TX_BUFFER_SIZE          128
#define DMA_RX_BUFFER_SIZE          4096

// ------------------- do not change the rest ------------------------//

#define DMA_MAX_BURST_DATA_TRANSFER 511         // This is the maximum data we are putting into DMA at once

class DmaSerialTeensy : public Stream {

private:

    static const uint8_t cnt_tx_pins = 2;
    static const uint8_t cnt_rx_pins = 2;
    typedef struct {
        const uint8_t 		pin;		// The pin number
        const uint32_t 		mux_val;	// Value to set for mux;
        volatile uint32_t	*select_input_register; // Which register controls the selection
        const uint32_t		select_val;	// Value for that selection
    } pin_info_t;

    typedef struct {
        IMXRT_LPUART_t* port;
        uint8_t dmaMuxSourceRx;
        uint8_t dmaMuxSourceTx;
        volatile uint32_t &ccm_register;
        const uint32_t ccm_value;
        pin_info_t rx_pins[cnt_rx_pins];
        pin_info_t tx_pins[cnt_tx_pins];
    } Base_t;

    const static Base_t serial1Base;
    const static Base_t serial2Base;
    const static Base_t serial3Base;
    const static Base_t serial4Base;
    const static Base_t serial5Base;
    const static Base_t serial6Base;
    const static Base_t serial7Base;
    const static Base_t* allSerialBases[7];
    const static DmaSerialTeensy* dmaSerials[7];

    static void txCompleteCallback1();
    static void txCompleteCallback2();
    static void txCompleteCallback3();
    static void txCompleteCallback4();
    static void txCompleteCallback5();
    static void txCompleteCallback6();
    static void txCompleteCallback7();
    static void (* const allTxIsr[7])();

    int serialNo;
    int rxPinIndex;
    int txPinIndex;

    uint8_t* txBuffer = nullptr;
    uint8_t* rxBuffer = nullptr;
    volatile size_t txBufferTail;
    volatile size_t txBufferHead;
    volatile size_t txBufferCount;
    volatile size_t rxBufferTail;
    // volatile uint32_t rxBufferHead; // no need for this

    volatile bool transmitting = false;

    const Base_t* serialBase = nullptr;
    DMAChannel* dmaChannelSend = nullptr;
    DMAChannel* dmaChannelReceive = nullptr;

    void txIsr();

public:

    explicit DmaSerialTeensy(int serialNo);
    int peek() override;
    void begin(uint32_t baud, uint16_t format = 0);
    int available() override;
    int read() override;
    using Print::write;
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    size_t write(char c);
    size_t write(unsigned long n)   { return write((uint8_t)n); }
    size_t write(long n)            { return write((uint8_t)n); }
    size_t write(unsigned int n)    { return write((uint8_t)n); }
    size_t write(int n)             { return write((uint8_t)n); }
};

#if defined(__IMXRT1062__)

extern DmaSerialTeensy dmaSerial1;
extern DmaSerialTeensy dmaSerial2;
extern DmaSerialTeensy dmaSerial3;
extern DmaSerialTeensy dmaSerial4;
extern DmaSerialTeensy dmaSerial5;
extern DmaSerialTeensy dmaSerial6;
extern DmaSerialTeensy dmaSerial7;

#endif
