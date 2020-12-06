# DMA Serial Library for Teensy 4.0

This small library provides you with the same API as Arduino UART serials like Serial1 but uses DMA and interrupt
in background to improve efficiency. You only need to replace your old Serial1 to 7 with dmaSerial1 to 7 and 
everything works fine. For an example on how to use this library, please refer to DmaTeensy4_0.ino file.

You can change the size of the background buffers by changing the values defined as DMA_TX_BUFFER_SIZE and 
DMA_RX_BUFFER_SIZE.

You are free to use this library in any project as long as you give me credit by keeping my email address
at the top of the header and source file.
