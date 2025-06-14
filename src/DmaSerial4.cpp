/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#include "DmaSerial.h"

#ifndef ARDUINO_TEENSY_MICROMOD
const DmaSerial::Base_t DmaSerial::serial4Base = {
    &IMXRT_LPUART3,
    DMAMUX_SOURCE_LPUART3_RX,
    DMAMUX_SOURCE_LPUART3_TX,
    CCM_CCGR0,
    CCM_CCGR0_LPUART3(CCM_CCGR_ON),
    {{16,2, &IOMUXC_LPUART3_RX_SELECT_INPUT, 0}, {0xff, 0xff, nullptr, 0}},
	{{17,2, &IOMUXC_LPUART3_TX_SELECT_INPUT, 0}, {0xff, 0xff, nullptr, 0}},
};
#else
const DmaSerial::Base_t DmaSerial::serial4Base = {
    &IMXRT_LPUART4,
    DMAMUX_SOURCE_LPUART4_RX,
    DMAMUX_SOURCE_LPUART4_TX,
    CCM_CCGR1,
    CCM_CCGR1_LPUART4(CCM_CCGR_ON),
    {{7,2, &IOMUXC_LPUART4_RX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}},
    {{8,2, &IOMUXC_LPUART4_TX_SELECT_INPUT, 2}, {0xff, 0xff, nullptr, 0}},
};
#endif