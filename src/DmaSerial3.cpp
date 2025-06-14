/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#include "DmaSerial.h"

const DmaSerial::Base_t DmaSerial::serial3Base = {
	&IMXRT_LPUART2,
	DMAMUX_SOURCE_LPUART2_RX,
	DMAMUX_SOURCE_LPUART2_TX,
	CCM_CCGR0,
	CCM_CCGR0_LPUART2(CCM_CCGR_ON),
	{{15,2, &IOMUXC_LPUART2_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	{{14,2, &IOMUXC_LPUART2_TX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
};
