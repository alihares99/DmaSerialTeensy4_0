/**
 * Created by Hares.
 * You are free to use this file in any project as long as you keep my email address alihares99@gmail.com here.
 */

#include "DmaSerial.h"

const DmaSerial::Base_t DmaSerial::serial7Base = {
	&IMXRT_LPUART7,
	DMAMUX_SOURCE_LPUART7_RX,
	DMAMUX_SOURCE_LPUART7_TX,
	CCM_CCGR5,
	CCM_CCGR5_LPUART7(CCM_CCGR_ON),
	{{28,2, &IOMUXC_LPUART7_RX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
	{{29,2, &IOMUXC_LPUART7_TX_SELECT_INPUT, 1}, {0xff, 0xff, nullptr, 0}},
};