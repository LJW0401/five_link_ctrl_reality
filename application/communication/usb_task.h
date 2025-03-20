#ifndef USB_TASK_H
#define USB_TASK_H

#include "robot_param.h"
#include "referee.h"

typedef struct
{
    fp32 last_time;
    bool error;
}usb_watch_dog_t;

extern void usb_task(void const * argument);

#endif /* USB_TASK_H */
