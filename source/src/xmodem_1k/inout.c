/*
 * inout.c
 *
 *  Created on: 2019-10-15
 *      Author: DELL
 */

#include "bl_config.h"
#include "uartStdio.h"
#include "uart.h"
#include "bl_platform.h"
#include "bl.h"
#include "delay.h"

int _inbyte(unsigned short milliSec)
{
    int ret = -1;
    int status;
    StopTimer();
    StartTimer(milliSec);
    do{
        ret = UARTConsoleGetcNonBlocking();
        status = IsTimerElapsed();
    }while(ret < 0 && status == 0);

    return ret;
}

void _outbyte(int c)
{
    UARTPutc(c);
}
