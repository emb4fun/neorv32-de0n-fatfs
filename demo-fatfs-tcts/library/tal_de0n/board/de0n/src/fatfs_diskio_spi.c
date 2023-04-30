/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2023 by Michael Fischer (www.emb4fun.de).
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
**************************************************************************/

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdio.h>
#include <string.h>
#include "tal.h"
#include "ff.h"
#include "diskio.h"

/*=======================================================================*/
/*  Prototypes                                                           */
/*=======================================================================*/
static uint8_t WaitReady (void);

/*=======================================================================*/
/*  Global                                                               */
/*=======================================================================*/

/*=======================================================================*/
/*  Extern                                                               */
/*=======================================================================*/

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define SD_LED_ON()
#define SD_LED_OFF()

/*
 * MMC/SD command (in SPI)
 */
#define CMD0    (0x40+0)   /* GO_IDLE_STATE */
#define CMD1    (0x40+1)   /* SEND_OP_COND (MMC) */
#define ACMD41  (0xC0+41)  /* SEND_OP_COND (SDC) */
#define CMD8    (0x40+8)   /* SEND_IF_COND */
#define CMD9    (0x40+9)   /* SEND_CSD */
#define CMD10   (0x40+10)  /* SEND_CID */
#define CMD12   (0x40+12)  /* STOP_TRANSMISSION */
#define ACMD13  (0xC0+13)  /* SD_STATUS (SDC) */
#define CMD16   (0x40+16)  /* SET_BLOCKLEN */
#define CMD17   (0x40+17)  /* READ_SINGLE_BLOCK */
#define CMD18   (0x40+18)  /* READ_MULTIPLE_BLOCK */
#define CMD23   (0x40+23)  /* SET_BLOCK_COUNT (MMC) */
#define ACMD23  (0xC0+23)  /* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24   (0x40+24)  /* WRITE_BLOCK */
#define CMD25   (0x40+25)  /* WRITE_MULTIPLE_BLOCK */
#define CMD55   (0x40+55)  /* APP_CMD */
#define CMD58   (0x40+58)  /* READ_OCR */

/*
 * Card type flags (CardType)
 */
#define CT_MMC          0x01  /* MMC ver 3 */
#define CT_SD1          0x02  /* SD ver 1 */
#define CT_SD2          0x04  /* SD ver 2 */
#define CT_SDC          (CT_SD1|CT_SD2)   /* SD */
#define CT_BLOCK        0x08  /* Block addressing */


/*
 * Card socket defines
 */
#define SOCK_CD_EMPTY   0x01  /* Card detect switch */
#define SOCK_WP_ACTIVE  0x02  /* Write protect switch */

/*
 * Wait for ready in timeout of 500ms
 */
#define WAIT_READY_TIME_MAX_MS   500


/*=======================================================================*/
/*  Definition of all global Data                                        */
/*=======================================================================*/

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static OS_SEMA           FSSema;
static DSTATUS           DiskStatus = STA_NOINIT;
static uint8_t           CardType = 0;        /* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */
static volatile uint16_t Timer1 = 0;          /* 100Hz decrement timers */
static volatile uint16_t Timer2 = 0;          /* 100Hz decrement timers */


/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*
 * Include platform dependent parts
 */
#include "fatfs_neorv32_de0n_spi.c"

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/
static uint8_t WaitReady (void)
{
   uint8_t res;

   /*
    * Divide by 10, because Timer2 is a 10ms timer.
    */

   Timer2 = (WAIT_READY_TIME_MAX_MS / 10);
   ReceiveU8();
   do
   {
      res = ReceiveU8();
   }
   while ((res != 0xFF) && Timer2);

   return(res);
} /* WaitReady */

/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/
static void ReleaseBus (void)
{
   /*
    * First deselect the CS line, and now make a dummy transmission.
    *
    * In SPI, each slave device is selected with separated CS signals,
    * and plural devices can be attached to an SPI bus. Generic SPI slave
    * device drives/releases its DO signal by CS signal asynchronously to
    * share an SPI bus. However MMC/SDC drives/releases DO signal in
    * synchronising to SCLK. There is a posibility of bus conflict when
    * attach MMC/SDC and any other SPI slaves to an SPI bus. Right image
    * shows the drive/release timing of MMC/SDC (DO is pulled to 1/2 vcc to
    * see the bus state). Therefore to make MMC/SDC release DO signal, the
    * master device must send a byte after deasserted CS signal.
    *
    * More information can be found here:
    * http://elm-chan.org/docs/mmc/mmc_e.html
    */
   DESELECT();
   ReceiveU8();
} /* ReleaseBus */

/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/
static uint8_t SendCMD (uint8_t  cmd,  /* Command byte */
                        uint32_t arg)  /* Argument */
{
   uint8_t n, res;

   if (cmd & 0x80)   /* ACMD<n> is the command sequense of CMD55-CMD<n> */
   {
      cmd &= 0x7F;
      res = SendCMD(CMD55, 0);
      if (res > 1)
         return res;
   }

   /* Select the card and wait for ready */
   DESELECT();
   SELECT();

   if (WaitReady() != 0xFF)
      return 0xFF;

   /* Send command packet */
   TRANSMIT_U8(cmd); /* Start + Command index */
   TRANSMIT_U8((uint8_t) (arg >> 24));  /* Argument[31..24] */
   TRANSMIT_U8((uint8_t) (arg >> 16));  /* Argument[23..16] */
   TRANSMIT_U8((uint8_t) (arg >> 8));   /* Argument[15..8] */
   TRANSMIT_U8((uint8_t) arg); /* Argument[7..0] */

   n = 0x01;   /* Dummy CRC + Stop */
   if (cmd == CMD0)
      n = 0x95;   /* Valid CRC for CMD0(0) */
   if (cmd == CMD8)
      n = 0x87;   /* Valid CRC for CMD8(0x1AA) */
   TRANSMIT_U8(n);

   /* Receive command response */
   if (cmd == CMD12)
      ReceiveU8();   /* Skip a stuff byte when stop reading */

   n = 10;  /* Wait for a valid response in timeout of 10 attempts */
   do
   {
      res = ReceiveU8();
   }
   while ((res & 0x80) && --n);

   return(res); /* Return with the response value */
} /* SendCMD */

/*************************************************************************/
/*  _timerproc                                                           */
/*                                                                       */
/*  Device timer interrupt procedure, will be called in period of 10ms.  */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void _timerproc (void)
{
   static uint32_t pvm;
   uint32_t n;
   DSTATUS s;

   /* 100Hz decrement timer */
   n = Timer1;
   if (n)
      Timer1 = (uint16_t)-- n;
   n = Timer2;
   if (n)
      Timer2 = (uint16_t)-- n;

   n = pvm;

   /* Sample socket switch */
   pvm = GetCDWP();

   /* Have contacts stabled? */
   if (n == pvm)
   {
      s = DiskStatus;

      /* Check write protect */
      if (pvm & SOCK_WP_ACTIVE)
         s |= STA_PROTECT;
      else
         s &= ~STA_PROTECT;

      /* Check socket empty */
      if (pvm & SOCK_CD_EMPTY)
         s |= (STA_NODISK | STA_NOINIT);
      else
         s &= ~STA_NODISK;

#if (SUPPORT_HW_WP == 0)
      /* Ignore the write protect */
      s &= ~STA_PROTECT;
#endif

      DiskStatus = s;
   }

} /* _timerproc */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  ff_memalloc                                                          */
/*                                                                       */
/*  Allocate a memory block                                              */
/*                                                                       */
/*  In    : msize                                                        */
/*  Out   : none                                                         */
/*  Return: Returns pointer to the allocated memory block                */
/*************************************************************************/
void *ff_memalloc (UINT msize)
{
   return(xmalloc(XM_ID_FS, msize));
} /* ff_memalloc */

/*************************************************************************/
/*  ff_memfree                                                           */
/*                                                                       */
/*  Free a memory block                                                  */
/*                                                                       */
/*  In    : mblock                                                       */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_memfree (void *mblock)
{
   xfree(mblock);
} /* ff_memfree */

/*************************************************************************/
/*  ff_cre_syncobj                                                       */
/*                                                                       */
/*  Create a Synchronization Object                                      */
/*                                                                       */
/*  In    : vol,  corresponding logical drive being processed.           */
/*          sobj, pointer to return the created sync object.             */
/*  Out   : none                                                         */
/*  Return: 0 = Error / 1 = OK                                           */
/*************************************************************************/
int ff_cre_syncobj (BYTE vol, FF_SYNC_t *sobj)
{
   (void)vol;

   /* Init the semaphore */
   OS_SemaCreate(&FSSema, 1, 1);
   *sobj = &FSSema;

   return(1);
} /* ff_cre_syncobj */

/*************************************************************************/
/*  ff_del_syncobj                                                       */
/*                                                                       */
/*  Delete a Synchronization Object                                      */
/*                                                                       */
/*  In    : sobj, sync object tied to the logical drive to be deleted.   */
/*  Out   : none                                                         */
/*  Return: 0 = Error / 1 = OK                                           */
/*************************************************************************/
int ff_del_syncobj (FF_SYNC_t sobj)
{
   /* Reset the semaphore */
   OS_SemaDelete(sobj);

   return(1);
} /* ff_del_syncobj */

/*************************************************************************/
/*  ff_req_grant                                                         */
/*                                                                       */
/*  Request Grant to Access the Volume                                   */
/*                                                                       */
/*  In    : sobj, sync object to wait.                                   */
/*  Out   : none                                                         */
/*  Return: 1 = Got a grant / 0 =  Could not get a grant                 */
/*************************************************************************/
int ff_req_grant (FF_SYNC_t sobj)
{
   OS_SemaWait(sobj, OS_WAIT_INFINITE);

   return(1);
} /* ff_req_grant */

/*************************************************************************/
/*  ff_rel_grant                                                         */
/*                                                                       */
/*  Request Grant to Access the Volume                                   */
/*                                                                       */
/*  In    : sobj, Sync object to be signaled.                            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void ff_rel_grant (FF_SYNC_t sobj)
{
   OS_SemaSignal(sobj);
} /* ff_rel_grant */

/*************************************************************************/
/*  disk_timerproc                                                       */
/*                                                                       */
/*  Device timer interrupt procedure, will be called in period of 1ms.   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void disk_timerproc (void)
{
   static uint16_t Timer10ms = 0;

   Timer10ms++;

   if (Timer10ms == 10)
   {
      Timer10ms = 0;
      _timerproc();  /* Drive timer procedure of low level disk I/O module */
   }

} /* disk_timerproc */

/*************************************************************************/
/*  disk_initialize                                                      */
/*                                                                       */
/*  Initialize a Drive                                                   */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: Status of Disk Functions                                     */
/*************************************************************************/
DSTATUS disk_initialize (uint8_t pdrv)
{
   uint8_t n, ty, cmd, ocr[4];

   if (DiskStatus & STA_NODISK)
   {
      /* No card in the socket */
      return(DiskStatus);
   }

   SD_LED_ON();

   if (0 == pdrv)
   {
      InitDiskIOHardware();

      /* low speed during init */
      SetLowSpeed();

      POWER_ON(); /* Force socket power ON */
      for (n = 10; n; n--)
         ReceiveU8();   /* 80 dummy clocks */

      ty = 0;
      if (SendCMD(CMD0, 0) == 1)
      {  /* Enter Idle state */
         Timer1 = 100;  /* Initialization timeout of 1000 msec */
         if (SendCMD(CMD8, 0x1AA) == 1)
         {  /* SDC ver 2.00 */
            for (n = 0; n < 4; n++)
               ocr[n] = ReceiveU8();
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {  /* The card can work at vdd range of 2.7-3.6V */
               while (Timer1 && SendCMD(ACMD41, 1UL << 30)) ;  /* ACMD41 with HCS bit */
               if (Timer1 && SendCMD(CMD58, 0) == 0)
               {  /* Check CCS bit */
                  for (n = 0; n < 4; n++)
                     ocr[n] = ReceiveU8();
                  ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2; /* Card id SDv2 */
               }
            }
         }
         else
         {  /* SDC ver 1.XX or MMC */
            if (SendCMD(ACMD41, 0) <= 1)
            {
               ty  = CT_SD1;
               cmd = ACMD41;  /* SDC ver 1.XX */
            }
            else
            {
               ty  = CT_MMC;
               cmd = CMD1; /* MMC */
            }
            while (Timer1 && SendCMD(cmd, 0)) ; /* Wait for leaving idle state */
            if (!Timer1 || SendCMD(CMD16, 512) != 0)  /* Select R/W block length */
               ty = 0;
         }
      }
      CardType = ty;
      ReleaseBus();

      if (ty)
      {  /* Initialization succeded */
         DiskStatus &= ~STA_NOINIT; /* Clear STA_NOINIT */

         SetHighSpeed();
      }
      else
      {  /* Initialization failed */
         POWER_OFF();
      }

   } /* end if (0 == pdrv) */

   SD_LED_OFF();

   return(DiskStatus);
} /* disk_initialize */

/*************************************************************************/
/*  disk_removed                                                         */
/*                                                                       */
/*  Medium was removed                                                   */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
void disk_removed (BYTE pdrv)
{
   (void)pdrv;
} /* disk_removed */

/*************************************************************************/
/*  disk_status                                                          */
/*                                                                       */
/*  Get Drive Status                                                     */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*  Out   : none                                                         */
/*  Return: Status of Disk Functions                                     */
/*************************************************************************/
DSTATUS disk_status (uint8_t pdrv)
{
   (void)pdrv;

   return(DiskStatus);
} /* disk_status */

/*************************************************************************/
/*  disk_read                                                            */
/*                                                                       */
/*  Read Sector(s)                                                       */
/*                                                                       */
/*  In    : pdrv,   physical drive nmuber to identify the drive          */
/*          buff,   data buffer to store read data                       */
/*          sector, sector address in LBA                                */
/*          count,  number of sectors to read                            */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
   if (!count)
      return RES_PARERR;
   if (DiskStatus & STA_NOINIT)
      return RES_NOTRDY;

   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }

      SD_LED_ON();

      if (!(CardType & CT_BLOCK))
         sector *= 512; /* Convert LBA to byte address if needed */

      if (count == 1)
      {  /* Single block read */
         if ((SendCMD(CMD17, sector) == 0)   /* READ_SINGLE_BLOCK */
             && ReceiveDatablock(buff, 512))
            count = 0;
      }
      else
      {  /* Multiple block read */
         if (SendCMD(CMD18, sector) == 0)
         {  /* READ_MULTIPLE_BLOCK */
            do
            {
               if (!ReceiveDatablock(buff, 512))
                  break;
               buff += 512;
            }
            while (--count);
            SendCMD(CMD12, 0);   /* STOP_TRANSMISSION */
         }
      }
      ReleaseBus();

      SD_LED_OFF();
   }

   return(count ? RES_ERROR : RES_OK);
} /* disk_read */

/*************************************************************************/
/*  disk_write                                                           */
/*                                                                       */
/*  Write Sector(s)                                                      */
/*                                                                       */
/*  In    : pdrv,   physical drive nmuber to identify the drive          */
/*          buff,   data to be written                                   */
/*          sector, sector address in LBA                                */
/*          count,  number of sectors to write                           */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
   if (!count)
      return RES_PARERR;
   if (DiskStatus & STA_NOINIT)
      return RES_NOTRDY;
   if (DiskStatus & STA_PROTECT)
      return RES_WRPRT;

   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }

      SD_LED_ON();

      if (!(CardType & CT_BLOCK))
         sector *= 512; /* Convert LBA to byte address if needed */

      if (count == 1)
      {  /* Single block write */
         if ((SendCMD(CMD24, sector) == 0)   /* WRITE_BLOCK */
             && TransmitDatablock(buff, 0xFE))
            count = 0;
      }
      else
      {  /* Multiple block write */
         if (CardType & CT_SDC)
         {
            SendCMD(CMD55, 0);
            SendCMD(CMD23, count);  /* ACMD23 */
         }
         if (SendCMD(CMD25, sector) == 0)
         {  /* WRITE_MULTIPLE_BLOCK */
            do
            {
               if (!TransmitDatablock(buff, 0xFC))
                  break;
               buff += 512;
            }
            while (--count);
            if (!TransmitDatablock(0, 0xFD)) /* STOP_TRAN token */
               count = 1;
         }
      }
      ReleaseBus();

      SD_LED_OFF();
   }

   return(count ? RES_ERROR : RES_OK);
} /* disk_write */

/*************************************************************************/
/*  disk_ioctl                                                           */
/*                                                                       */
/*  In    : pdrv, physical drive nmuber to identify the drive            */
/*          cmd,  control code                                           */
/*          buff, buffer to send/receive control data                    */
/*  Out   : none                                                         */
/*  Return: Results of Disk Functions                                    */
/*************************************************************************/
DRESULT disk_ioctl (uint8_t pdrv, uint8_t cmd, void *buff)
{
   DRESULT Result = RES_PARERR;
   uint8_t n, csd[16], *ptr = buff;
   uint16_t csize;

   if (0 == pdrv)
   {
      /* Check drive status */
      if (DiskStatus & STA_NOINIT)
      {
         return(RES_NOTRDY);
      }

      SD_LED_ON();

      Result = RES_ERROR;
      switch (cmd)
      {
         case CTRL_SYNC:  /* Make sure that pending write process has been finished */
            SELECT();
            if (WaitReady() == 0xFF)
               Result = RES_OK;
            break;

         case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */
            if ((SendCMD(CMD9, 0) == 0) && ReceiveDatablock(csd, 16))
            {
               if ((csd[0] >> 6) == 1)
               {  /* SDC ver 2.00 */
                  //@@MF csize = csd[9] + ((FFS_U16)csd[8] << 8) + 1;
                  csize = (uint16_t) ((uint16_t) csd[9] | ((uint16_t) csd[8] << 8)) + 1;
                  *(uint32_t *) buff = (uint32_t) csize << 10;
               }
               else
               {  /* MMC or SDC ver 1.XX */
                  n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                  csize = (csd[8] >> 6) + ((uint16_t) csd[7] << 2) + ((uint16_t) (csd[6] & 3) << 10) + 1;
                  *(uint32_t *) buff = (uint32_t) csize << (n - 9);
               }
               Result = RES_OK;
            }
            break;

         case GET_SECTOR_SIZE:  /* Get sectors on the disk (WORD) */
            *(uint16_t *) buff = 512;
            Result = RES_OK;
            break;

         case GET_BLOCK_SIZE:   /* Get erase block size in unit of sectors (DWORD) */
            if (CardType & CT_SD2)
            {  /* SDC ver 2.00 */
               if (SendCMD(ACMD13, 0) == 0)
               {  /* Read SD status */
                  ReceiveU8();
                  if (ReceiveDatablock(csd, 16))
                  {  /* Read partial block */
                     for (n = 64 - 16; n; n--)
                        ReceiveU8();   /* Purge trailing data */
                     *(uint32_t *) buff = 16UL << (csd[10] >> 4);
                     Result = RES_OK;
                  }
               }
            }
            else
            {  /* SDC ver 1.XX or MMC */
               if ((SendCMD(CMD9, 0) == 0) && ReceiveDatablock(csd, 16))
               {  /* Read CSD */
                  if (CardType & CT_SD1)
                  {  /* SDC ver 1.XX */
                     *(uint32_t *) buff = (((csd[10] & 63) << 1) + ((uint16_t) (csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                  }
                  else
                  {  /* MMC */
                     *(uint32_t *) buff = ((uint16_t) ((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                  }
                  Result = RES_OK;
               }
            }
            break;

         /* Following command are not used by FatFs module */

         case MMC_GET_TYPE:  /* Get MMC/SDC type (BYTE) */
            *ptr = CardType;
            Result = RES_OK;
            break;

         case MMC_GET_CSD:   /* Receive CSD as a data block (16 bytes) */
            if (SendCMD(CMD9, 0) == 0  /* READ_CSD */
                && ReceiveDatablock(ptr, 16))
               Result = RES_OK;
            break;

         case MMC_GET_CID:   /* Receive CID as a data block (16 bytes) */
            if (SendCMD(CMD10, 0) == 0 /* READ_CID */
                && ReceiveDatablock(ptr, 16))
               Result = RES_OK;
            break;

         case MMC_GET_OCR:   /* Receive OCR as an R3 resp (4 bytes) */
            if (SendCMD(CMD58, 0) == 0)
            {  /* READ_OCR */
               for (n = 0; n < 4; n++)
                  *ptr++ = ReceiveU8();
               Result = RES_OK;
            }
            break;

         case MMC_GET_SDSTAT:   /* Receive SD statsu as a data block (64 bytes) */
            if (SendCMD(ACMD13, 0) == 0)
            {  /* SD_STATUS */
               ReceiveU8();
               if (ReceiveDatablock(ptr, 64))
                  Result = RES_OK;
            }
            break;

         default:
            Result = RES_PARERR;
      }

      ReleaseBus();

      SD_LED_OFF();
   }

   return(Result);
} /* disk_ioctl */

/*************************************************************************/
/*  get_fattime                                                          */
/*                                                                       */
/*  Gets Time from RTC                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Time                                                         */
/*************************************************************************/
DWORD get_fattime (void)
{
   return(0);
} /* get_fattime */

/*** EOF ***/
