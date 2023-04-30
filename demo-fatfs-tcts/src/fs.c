/**************************************************************************
*  Copyright (c) 2019-2023 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
***************************************************************************
*  History:
*
*  04.05.2019  mifi  First Version.
**************************************************************************/
#define __FS_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include "tal.h"
#include "terminal.h"

#include "ff.h"
#include "diskio.h"

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define LOGICAL_DRIVE   "0:/"

/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

/*
 * File system object
 */
static FATFS   FSObject;
static uint8_t Buffer[(4*1024)];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  GetFreeInfo                                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static FRESULT GetFreeInfo (void)
{
   FRESULT    Res;
   FATFS     *fs;
   uint32_t   fre_clust, fre_sect, tot_sect;

   /* Get volume information and free clusters of drive 0 */
   Res = f_getfree("0:", &fre_clust, &fs);
   if (Res != FR_OK) goto Exit;  /*lint !e801*/

   /* Get total sectors and free sectors */
   tot_sect = (fs->n_fatent - 2) * fs->csize;
   fre_sect = fre_clust * fs->csize;

   /* Print the free space (assuming 512 bytes/sector) */
   term_printf("%10lu KiB total drive space.\r\n", tot_sect / 2);
   term_printf("%10lu KiB available.\r\n", fre_sect / 2);
   term_printf("\r\n");

Exit:
   return(Res);
} /* GetFreeInfo */

/*************************************************************************/
/*  SimpleTest                                                           */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static FRESULT SimpleTest (void)
{
   FRESULT    Res;
   FIL        fil;
   uint8_t   bData = 0;
   uint32_t  dSize;

   term_printf("Simple write/read test... ");
   Res = f_open(&fil, "fatfs.txt", FA_CREATE_ALWAYS | FA_WRITE);
   if (FR_OK == Res)
   {
      bData = '1';
      f_write(&fil, &bData, 1, (UINT*)&dSize);
      f_sync(&fil);
      f_close(&fil);

      f_open(&fil, "fatfs.txt", FA_READ);
      bData = 0;
      f_read(&fil, &bData, 1, (UINT*)&dSize);
      f_close(&fil);

      if ('1' == bData)
      {
         term_printf("OK\r\n");
      }
      else
      {
         term_printf("Error\r\n");
      }
   }
   else
   {
      term_printf("Error\r\n");
   }
   term_printf("\r\n");

   return(Res);
} /* SimpleTest */

/*************************************************************************/
/*  PerfWriteRead                                                        */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: FRESULT                                                      */
/*************************************************************************/
static FRESULT PerfWriteRead (void)
{
   FRESULT    Res;
   FIL        fil;
   uint32_t  dSize;

   uint32_t  dStartTime;
   uint32_t  dEndTime;
   uint32_t  dCount;
   float     fExecTime;
   float     fValue;
   int       nR1, nR2;


   term_printf("Performance write/read:\r\n");
   dStartTime = OS_TimeGet();

   Res = f_open(&fil, "data.bin", FA_CREATE_ALWAYS | FA_WRITE);
   if (FR_OK == Res)
   {
      dCount     = 0;
      dStartTime = OS_TimeGet();
      dEndTime   = dStartTime + 1000;

      do
      {
         Res = f_write(&fil, Buffer, sizeof(Buffer), (UINT*)&dSize);
         if (Res != FR_OK) goto Exit;  /*lint !e801*/

         dCount += sizeof(Buffer);
      }
      while (OS_TimeGet() < dEndTime);

      dEndTime = OS_TimeGet();

      /* Write more data for the next read test */
      for (int i = 0; i < 512; i++)
      {
         f_write(&fil, Buffer, sizeof(Buffer), (UINT*)&dSize);
      }
      /******************************************/

      f_close(&fil);

      /* Calculate execution time */
      fExecTime = ((float)(dEndTime - dStartTime) / (float)1000);
      fValue    = (float)((float)dCount / (1024.0 * 1024.0)) / fExecTime;

      nR1 = (int)fValue;
      nR2 = (int)((fValue - (float)nR1) * 100.0);

      term_printf("Write: %2d.%02d MB/s\r\n", nR1, nR2);
   }

   Res = f_open(&fil, "data.bin", FA_READ);
   if (FR_OK == Res)
   {
      dCount     = 0;
      dStartTime = OS_TimeGet();
      dEndTime   = dStartTime + 1000;

      do
      {
         Res = f_read(&fil, Buffer, sizeof(Buffer), (UINT*)&dSize);
         if (Res != FR_OK) goto Exit;  /*lint !e801*/

         dCount += sizeof(Buffer);
      }
      while (OS_TimeGet() < dEndTime);

      dEndTime = OS_TimeGet();
      f_close(&fil);

      /* Calculate execution time */
      fExecTime = ((float)(dEndTime - dStartTime) / (float)1000);
      fValue    = (float)((float)dCount / (1024.0 * 1024.0)) / fExecTime;

      nR1 = (int)fValue;
      nR2 = (int)((fValue - (float)nR1) * 100.0);

      term_printf("Read : %2d.%02d MB/s\r\n", nR1, nR2);
   }
   term_printf("\r\n");

Exit:
   return(Res);
} /* PerfWriteRead */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  fs_Test                                                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void fs_Test (void)
{
   FRESULT    Res;
   DSTATUS    DiskStatus;

   /* Init diskio interface */
   disk_initialize(0);

   /* Get drive status */
   DiskStatus = disk_status(0) & STA_NODISK;

   /* Check if a memory card is available */
   if (0 == (DiskStatus & STA_NODISK))
   {
      Res = f_mount(&FSObject, LOGICAL_DRIVE, 0);
      if (Res != FR_OK) goto Exit;  /*lint !e801*/
   }

   Res = GetFreeInfo();
   if (Res != FR_OK) goto Exit;  /*lint !e801*/

   OS_TimeDly(1000);

   Res = SimpleTest();
   if (Res != FR_OK) goto Exit;  /*lint !e801*/

   Res = PerfWriteRead();
   if (Res != FR_OK) goto Exit;  /*lint !e801*/


Exit:

   f_mount(NULL, "", 0);
   disk_removed(0);

} /* fs_Test */

/*** EOF ***/
