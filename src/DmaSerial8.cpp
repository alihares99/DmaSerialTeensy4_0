/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#include "DmaSerial.h"

#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41) && 0

const DmaSerial::Base_t DmaSerial::serial8Base = {
	&IMXRT_LPUART5,
	DMAMUX_SOURCE_LPUART5_RX,
	DMAMUX_SOURCE_LPUART5_TX,
	CCM_CCGR3,
	CCM_CCGR3_LPUART5(CCM_CCGR_ON),
	{{34,1, &IOMUXC_LPUART5_RX_SELECT_INPUT, 1}, {48, 2, &IOMUXC_LPUART5_RX_SELECT_INPUT, 0}},
	{{35,1, &IOMUXC_LPUART5_TX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
};
#endif
