#ifndef TRACK_STORE_TEST_DEVICE_H
#define TRACK_STORE_TEST_DEVICE_H

#include <stdint.h>

extern uint8_t EA, IAP_CONTR, IAP_CMD, IAP_TRIG, IAP_DATA;
extern uint8_t IAP_TPS, IAP_ADDRE, IAP_ADDRH, IAP_ADDRL;
extern uint32_t system_clock;
void TrackTest_IAPNop(void);
#define _nop_() TrackTest_IAPNop()

#endif
