//
// Created by keeper on 2026/7/26.
//

#ifndef RC_REMOTE_C8T6_MYDBUS_H
#define RC_REMOTE_C8T6_MYDBUS_H

#include "main.h"
#include "usart.h"
#include "MyUart.h"

#define DBUS_FRAME_LEN      18

#define DBUS_MIN_VAL        364
#define DBUS_MAX_VAL        1684
#define DBUS_CENTER_VAL     1024

void DBUS_SendFrame(Rx_Frame_t *rx);

#endif //RC_REMOTE_C8T6_MYDBUS_H
