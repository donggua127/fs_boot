/**
 * \file  bl_main.c
 *
 * \brief Implements main function for StarterWare bootloader
 *
*/

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "bl_config.h"
#include "uartStdio.h"
#include "bl_copy.h"
#include "bl_platform.h"
#include "bl.h"
#include "bl_nand.h"
#include "nandlib.h"
#include "delay.h"
#include "hw_types.h"

/******************************************************************************
**                    External Variable Declararions 
*******************************************************************************/

extern char *deviceType;

/******************************************************************************
**                     Local Function Declararion 
*******************************************************************************/

static void (*appEntry)();

static void showMenu();
static unsigned int updateApp();
static unsigned int updateFlash();
/******************************************************************************
**                     Global Variable Definitions
*******************************************************************************/

unsigned int entryPoint = 0;
unsigned int DspEntryPoint = 0;

NandInfo_t *hNandInfo;

/******************************************************************************
**                     Global Function Definitions
*******************************************************************************/
/*
 * \brief This function initializes the system and copies the image. 
 *
 * \param  none
 *
 * \return none 
*/
int main(void)
{

    int secondCnt;
    char str;
    int status;
    /* Configures PLL and DDR controller*/
    BlPlatformConfig();

    hNandInfo = (NandInfo_t *)BlNANDConfigure();

    UARTPuts("StarterWare ", -1);
    UARTPuts(deviceType, -1);
    UARTPuts(" Boot Loader\n\r", -1);

    while(1)
    {
        UARTPuts("Press k to enter the update process within 3s",-1);

        secondCnt = 3;
        do{
            StartTimer(1000);
            do{
                str = UARTConsoleGetcNonBlocking();
                status = IsTimerElapsed();
            }while(str != 'k' && status == 0);
            StopTimer();
            secondCnt--;
            UARTprintf("\b\b%ds",secondCnt);
        }while(secondCnt > 0 && str != 'k');
        UARTPuts("\r\n",-1);

        if(str == 'k')
        {

            showMenu();

            secondCnt = 5;
            do{
                StartTimer(1000);
                do{
                    str = UARTConsoleGetcNonBlocking();
                    status = IsTimerElapsed();
                    if(str == '\r')
                    {
                        showMenu();
                    }
                }while(str != 'u' && str != 'd' && str != 'b' && status == 0);
                StopTimer();
                secondCnt--;
                UARTprintf("\b\b%ds",secondCnt);
            }while( str != 'u' && str != 'd' && str != 'b' && secondCnt > 0);
            UARTPuts("\r\n",-1);

            if(str == 'u')
            {
                UARTPuts("\r\nWrite Application to NandFlash and Boot ... \r\n",-1);
                if(updateFlash() == true)
                {
                    break;
                }
            }
            else if(str == 'd')
            {
                UARTPuts("\r\nUpdate Application for Debug ... \r\n",-1);
                if(updateApp() == true)
                {
                    break;
                }

            }
            else
            {
                UARTPuts("Load Application From NAND Flash\r\n",-1);
                if(ImageCopy() == true)
                {
                    break;
                }
            }
        }
        else
        {
            UARTPuts("Load Application From NAND Flash\r\n",-1);
            if(ImageCopy() == true)
            {
                break;
            }
        }
    }


    UARTPuts("Jumping to Application...\r\n\n", -1);

    /* Do any post-copy config before leaving boot loader */
    BlPlatformConfigPostBoot();

    /* Giving control to the application */
    appEntry = (void (*)(void)) entryPoint;

    (*appEntry)( );

    return 0;
}

static unsigned int updateApp()
{
    int fileLen;
    int retVal = false;

    UARTPuts("\r\nStart xmodem Receive...\n\r", -1);
    fileLen = xmodemReceive((char *)DDR_START_ADDR, 256*1024);

    if (fileLen < 0) {
        UARTprintf ("Xmodem receive error: status: %d\n", fileLen);
    }
    else  {
        UARTprintf ("Xmodem successfully received %d bytes\n", fileLen);
        UARTBootCopy();
        retVal = true;
    }
    return retVal;
}

static unsigned int updateFlash()
{
    int fileLen;
    int numPagesFile;
    int retVal = false;
    UARTPuts("\r\nStart xmodem Receive...\n\r", -1);
    fileLen = xmodemReceive((char *)DDR_START_ADDR, 256*1024);

    if (fileLen < 0) {
        UARTprintf ("Xmodem receive error: status: %d\n", fileLen);
        return 0;
    }
    else  {
        UARTprintf ("Xmodem successfully received %d bytes\n", fileLen);
#if 0
        UARTBootCopy();
#endif
        numPagesFile = 0;
        while ( (numPagesFile * hNandInfo->pageSize)  < fileLen )
        {
            numPagesFile++;
        }

        if(BlNANDWriteFlash(hNandInfo,(unsigned char*)DDR_START_ADDR,numPagesFile) == NAND_STATUS_PASSED)
        {
            UARTPuts("Write Flash Success...\r\n",-1);
            UARTPuts("Boot from NAND Flash...\r\n",-1);
            retVal = ImageCopy();
        }
    }
    return retVal;

}

static void showMenu()
{
    UARTPuts("\r\n\r\n",-1);
    UARTPuts("u: update flash with xmodem file\r\n",-1);
    UARTPuts("d: update application with xmodem file\r\n",-1);
    UARTPuts("b: boot directly from flash\r\n",-1);
    UARTPuts("Press key to select operation within 5s",-1);
}

void BootAbort(void)
{
    while(1);
}

/******************************************************************************
**                              END OF FILE
*******************************************************************************/
