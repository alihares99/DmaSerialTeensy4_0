/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#include "DmaSerial.h"
	
const DmaSerial::Base_t DmaSerial::serial6Base = {
	&IMXRT_LPUART1,
	DMAMUX_SOURCE_LPUART1_RX,
	DMAMUX_SOURCE_LPUART1_TX,
	CCM_CCGR5,
	CCM_CCGR5_LPUART1(CCM_CCGR_ON),
	{{25,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
	{{24,2, nullptr, 0}, {0xff, 0xff, nullptr, 0}},
};