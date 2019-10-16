/**
 * \file  bl_nand.c
 *
 * \brief NAND Initialization functions.  And a funciton to copy data 
 *        from NAND to the given address.
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

#include <string.h>

#include "uartStdio.h"
#include "nandlib.h"
#include "hw_types.h"

#include "bl.h"
#include "bl_platform.h"

#include "bl_nand.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// General type include
#include "tistdtypes.h"

/******************************************************************************
**                     Macro Defination 
*******************************************************************************/

/*****************************************************************************/
/*
** Macros which defines the read write size, buffer size and number of transfers
**
*/
#define NAND_DATA_BUFF_SIZE    (NAND_PAGE_SIZE_IN_BYTES)
#define NAND_ECC_BUFF_SIZE     ((NAND_PAGE_SIZE_IN_BYTES/NAND_BYTES_PER_TRNFS) \
                                * NAND_MAX_ECC_BYTES_PER_TRNFS)


/******************************************************************************
**                     Function Prototype Declaration.
*******************************************************************************/


/******************************************************************************
**                    Local  Declaration 
*******************************************************************************/
NandInfo_t              nandInfo;
NandCtrlInfo_t          nandCtrlInfo;
NandEccInfo_t           nandEccInfo;
NandDmaInfo_t           nandDmaInfo;

#if defined(__IAR_SYSTEMS_ICC__)

#pragma data_alignment=4
volatile unsigned char rxData[NAND_DATA_BUFF_SIZE];

#elif defined(__TMS470__) || defined(_TMS320C6X)

#pragma DATA_ALIGN(rxData, 4);
volatile unsigned char rxData[NAND_DATA_BUFF_SIZE];

#else

volatile unsigned char  __attribute__ ((aligned (4))) rxData[NAND_DATA_BUFF_SIZE];

#endif

unsigned char eccData[NAND_ECC_BUFF_SIZE];


/*
* \brief - NAND Configures.
* \param - none.
*
* \return  none.
*/
NandInfo_t *BlNANDConfigure(void)
{
    unsigned int retVal;
    unsigned int i;

    retVal = true;

    /* Initialize the nandInfo struct */
    nandInfo.hNandCtrlInfo = &nandCtrlInfo;
    nandInfo.hNandEccInfo = &nandEccInfo;
    nandInfo.hNandDmaInfo = &nandDmaInfo;
    BlPlatformNANDInfoInit(&nandInfo);

    for(i=0;i<NAND_ECC_BUFF_SIZE;i++)
    {
        eccData[i] = 0xff;
    }
    /* Open the NAND device */
    retVal = NANDOpen(&nandInfo);
    if (retVal & NAND_STATUS_FAILED)
    {
        UARTPuts("\r\n*** ERROR : NAND Open Failed... ",-1);
        while(1);    
    }
    else if (retVal & NAND_STATUS_WAITTIMEOUT)
    {
        UARTPuts("\r\n*** ERROR : Device Is Not Ready...!!!\r\n", -1);
        while(1);
    }
    else if (retVal & NAND_STATUS_NOT_FOUND)
    {
        UARTPuts("\r\n*** ERROR : DEVICE MAY NOT BE ACCESSABLE OR NOT PRESENT."
                 "\r\n", -1);
        while(1);
    }
    
    return &nandInfo;
}

/**
* \brief - Reads bytes from NAND.
* \param - flashAddr - NAND Flash address.\n.
* \param - size - Indicates the total size needs to be read from flash.\n.
* \param - destAddr - Destination address where data needs to be copy.\n.
*
* \return none
**/
void BlNANDReadFlash (NandInfo_t *hNandInfo, unsigned int flashAddr, unsigned int size, unsigned char *destAddr)
{
    static unsigned int currBlock = 0xFFFFFFFF;
    static unsigned int currPage = 0xFFFFFFFF;

    unsigned int pageSize   = hNandInfo->pageSize;
    unsigned int blockSize  = hNandInfo->blkSize;
    unsigned int memBufferPtr = 0;
    unsigned int bytesLeftInBuff = 0;
    unsigned int bytesToCopy = 0;
    unsigned int tmp;
    
    NandStatus_t retVal;

    /* Convert the flashAddr to block, page numbers */
    unsigned int blkNum = (flashAddr / blockSize);
    unsigned int pageNum = (flashAddr - (blkNum * blockSize)) / pageSize;
    
    
    /* Check to see if we need to buffer a new page */
    if ((blkNum != currBlock) || (pageNum != currPage))
    {
        if((tmp=NANDBadBlockCheck(hNandInfo, blkNum)) == NAND_BLOCK_GOOD)
        {
            currBlock = blkNum;
            currPage = pageNum;        
            retVal = NANDPageRead( hNandInfo, currBlock, currPage,
                                   rxData, &eccData[0]);
            if(retVal != NAND_STATUS_PASSED)
            {
                UARTPuts("\r\n Reading Image From NAND ...NAND Read Failed.", -1);
                BootAbort();
            }
        }
        else
        {
            UARTPuts("\r\n NAND Bad Block Check Failed.", -1);
            BootAbort();
        }
    }
    
    /* Figure out offset in buffered page */
    memBufferPtr = flashAddr - (currBlock * blockSize) - (currPage * pageSize);    
  
    // Now we do the actual reading of bytes  
    // If there are bytes in the memory buffer, use them first
    bytesLeftInBuff = NAND_DATA_BUFF_SIZE - memBufferPtr;
    if (bytesLeftInBuff > 0)
    {
        bytesToCopy = (bytesLeftInBuff >= size) ? size : bytesLeftInBuff;
    
        // Copy bytesToCopy bytes from current buffer pointer to the dest
        memcpy((void *)destAddr, (void *)&rxData[memBufferPtr], bytesToCopy);
        destAddr  += bytesToCopy;
        size      -= bytesToCopy;
        flashAddr += bytesToCopy;
      
        bytesLeftInBuff -= bytesToCopy;
    }
  
    // If we have one or more full blocks to copy, copy them directly
    // Any leftover data (partial page) gets copied via the memory buffer
    while (size > 0)
    {
        unsigned char *tempPtr = NULL;
        currPage += 1;
  
        // Check to see if curr page is at end of a block
        if (currPage >= hNandInfo->pagesPerBlk)
        {
            // If so, increment current block until we are in a good block
            do 
            {
                currBlock += 1;
            }
            while (NANDBadBlockCheck(hNandInfo,currBlock)!= NAND_BLOCK_GOOD);
            currPage  = 0;
        } 

        // Read the new current page in the current block to its destination
        tempPtr = (unsigned char *)(size >= pageSize) ? destAddr : ((unsigned char *)rxData);
        bytesToCopy = (size >= pageSize) ? pageSize : size;
    
        retVal = NANDPageRead( hNandInfo, currBlock, currPage,
                               tempPtr, &eccData[0]);
        if(retVal != NAND_STATUS_PASSED)
        {
            UARTPuts("\r\n Reading Image From NAND ...NAND Read Failed.", -1);
            BootAbort();
        }
    
        if (tempPtr != destAddr)
        {
            // If the last copy was a partial page, copy byteCnt
            // bytes from memory buffer pointer to the dest
            memcpy((void *)destAddr, (void *)rxData, bytesToCopy);
        }
    
        size      -= bytesToCopy;
        destAddr  += bytesToCopy;
        flashAddr += bytesToCopy;
    }
}

/************************************************************
* Local Function Definitions                                *
************************************************************/
// Simple wait loop - comes in handy.
static void BlNANDWaitLoop(Uint32 loopcnt)
{
  Uint32 i;
  for (i = 0; i<loopcnt; i++)
  {
    asm("   NOP");
  }
}

// Verify data written by reading and comparing byte for byte
static Uint32 BlNANDVerifyPage(NandInfo_t *hNandInfo, Uint32 block, Uint32 page, Uint8* src, Uint8* dest)
{
    Uint32 i, errCnt;

    if (NANDPageRead(hNandInfo, block, page, dest, eccData) != NAND_STATUS_PASSED)
        return NAND_STATUS_FAILED;

    errCnt = 0;
    for (i=0; i <(hNandInfo->pageSize); i++)
    {
        // Check for data read errors
        if ( src[i] != dest[i] )
        {
            errCnt++;
#if (0)
      DEBUG_printString("Data verification failed! Byte # ");
      DEBUG_printHexInt(i);
      DEBUG_printString(" Expected Data: ");
      DEBUG_printHexInt( src[i]);
      DEBUG_printString(", Received Byte: ");
      DEBUG_printHexInt( dest[i]);
      DEBUG_printString("\r\n");
#endif
        }
    }

    if (errCnt != 0)
    {
        return NAND_STATUS_FAILED;
    }
    else
    {
        return NAND_STATUS_PASSED;
    }
}

Uint32 BlNANDWriteFlash(NandInfo_t *hNandInfo, Uint8 *srcBuf, Uint32 totalPageCnt)
{
    Uint32    blockNum,pageNum,pageCnt;
    Uint32    numBlks;

    Uint8     *dataPtr;
    Uint8     nandRx[NAND_PAGE_SIZE_IN_BYTES];
    // Get total number of blocks needed
    numBlks = 0;
    while ( (numBlks*hNandInfo->pagesPerBlk)  < totalPageCnt )
    {
        numBlks++;
    }
    UARTprintf("Number of blocks needed for data: %x\r\n",numBlks);

    // Start in block 1 (leave block 0 alone)
    blockNum = 1;

    while (blockNum < NAND_NUMOF_BLK)
    {
        // Find first good block

        while (NANDBadBlockCheck(hNandInfo,blockNum) != NAND_BLOCK_GOOD)
        {
            blockNum++;
        }

        // Erase the current block
        NANDBlockErase(hNandInfo,blockNum);
        if (NANDBadBlockCheck(hNandInfo,blockNum) != NAND_BLOCK_GOOD)
        {
            UARTPuts("Erase bad Block...\r\n",-1);
        }

        // Start writing in page 0 of current block
        pageNum = 0;
        pageCnt = 0;

        // Setup data pointer
        dataPtr = srcBuf;

        // Start page writing loop
        do
        {
            UARTprintf("Writing image data to block %x, page %x.\r\n",blockNum,pageNum);

            // Write the AIS image data to the NAND device
            if (NANDPageWrite(hNandInfo, blockNum,  pageNum, dataPtr,eccData) != NAND_STATUS_PASSED)
            {
                UARTPuts("Write failed. Marking block as bad...\r\n",-1);
                NANDReset(hNandInfo);
                NANDMarkBlockAsBad(hNandInfo,blockNum);
                dataPtr -=  pageNum * hNandInfo->pageSize;
                blockNum++;
                continue;
            }

            if (NANDBadBlockCheck(hNandInfo,blockNum) != NAND_BLOCK_GOOD)
            {
                UARTPuts("Pagewrite bad Block...\r\n",-1);
            }

            BlNANDWaitLoop(200);

            // Verify the page just written
            if (BlNANDVerifyPage(hNandInfo, blockNum, pageNum, dataPtr, nandRx) != NAND_STATUS_PASSED)
            {
                UARTPuts("Verify failed. Marking block as bad...\r\n",-1);
                NANDReset(hNandInfo);
                NANDMarkBlockAsBad(hNandInfo,blockNum);
                dataPtr -=  pageNum * hNandInfo->pageSize;
                blockNum++;
                continue;
            }
            if (NANDBadBlockCheck(hNandInfo,blockNum) != NAND_BLOCK_GOOD)
            {
                UARTPuts("Pageread bad Block...\r\n",-1);
            }

            pageNum++;
            pageCnt++;
            dataPtr +=  hNandInfo->pageSize;

            if (pageNum == hNandInfo->pagesPerBlk)
            {
                // A block transition needs to take place; go to next good block
                do
                {
                    blockNum++;
                }
                while (NANDBadBlockCheck(hNandInfo,blockNum) != NAND_BLOCK_GOOD);

                // Erase the current block
                NANDBlockErase(hNandInfo,blockNum);

                pageNum = 0;
            }
        } while (pageCnt < totalPageCnt);

        break;
  }
  return NAND_STATUS_PASSED;
}

/******************************************************************************
**                              END OF FILE
*******************************************************************************/
