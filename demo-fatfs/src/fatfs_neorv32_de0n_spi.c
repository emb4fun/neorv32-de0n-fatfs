/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2012-2023 by Michael Fischer (www.emb4fun.de).
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
*  30.08.2012  mifi  First version, tested with an Altera DE1 board.
*  15.08.2013  mifi  The 16 bit access was replaced by 32 bit.
*  17.08_2013  mifi  Added support for CD and WP for Altera DE0-Nano.
*  24.04.2023  mifi  Tested with a NEORV32 on a DE0-Nano board.
**************************************************************************/
#define __FATFS_NEORV32_DE0CV_SPI_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/

/*=======================================================================*/
/*  DEFINE: All Structures and Common Constants                          */
/*=======================================================================*/

#define SPI_MASTER_BASE 0x92000000

/*
 * Simple SPI (pio) defines
 */
#define ADDR_TXDATA     0
#define ADDR_RXDATA     4
#define ADDR_CONTROL    8
#define ADDR_STATUS     12

#define CTRL_SSEL       1
#define CTRL_BIT32      2
#define CTRL_LOOP       4

#define STATUS_DONE     1
#define STATUS_CD       2
#define STATUS_WP       4

#define SELECT()        {                           \
                            Control1 &= ~CTRL_SSEL; \
                            SPI_CTRL  = Control1;   \
                        }


#define DESELECT()      {                           \
                            Control1 |= CTRL_SSEL;  \
                            SPI_CTRL  = Control1;   \
                        }

#define POWER_ON()
#define POWER_OFF()

#define SPI_BASE        SPI_MASTER_BASE
#define SPI_TXR         *((volatile uint32_t*)(SPI_BASE + ADDR_TXDATA))
#define SPI_RXR         *((volatile uint32_t*)(SPI_BASE + ADDR_RXDATA))
#define SPI_CTRL        *((volatile uint32_t*)(SPI_BASE + ADDR_CONTROL))
#define SPI_SR          *((volatile uint32_t*)(SPI_BASE + ADDR_STATUS))
#define SPI_SR_DONE     STATUS_DONE

/*=======================================================================*/
/*  DEFINE: Prototypes                                                   */
/*=======================================================================*/

/*=======================================================================*/
/*  DEFINE: Definition of all local Data                                 */
/*=======================================================================*/
static uint32_t Control1 = 0;

/*=======================================================================*/
/*  DEFINE: Definition of all local Procedures                           */
/*=======================================================================*/

/*************************************************************************/
/*  SetLowSpeed                                                          */
/*                                                                       */
/*  Set SPI port speed to 200 KHz. Provided that the CPU is              */
/*  running with 100 MHz.                                                */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SetLowSpeed(void)
{
   Control1 &= ~0xFF00;
   Control1 |= (249 << 8);
   SPI_CTRL  = Control1;

} /* SetLowSpeed */

/*************************************************************************/
/*  SetHighSpeed                                                         */
/*                                                                       */
/*  Set SPI port speed to 25 MHz. Provided that the CPU is               */
/*  running with 100 MHz.                                                */
/*                                                                       */
/*  Note: 25MHz works only with SD cards, MMC need 20MHz max.            */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SetHighSpeed(void)
{
   Control1 &= ~0xFF00;

   if (0 == (CardType & 0x01))
   {
      /* SD card 25 MHz */
      Control1 |= (1 << 8);
   }
   else
   {
      /* MMC card 16 MHz */
      Control1 |= (2 << 8);
   }
   SPI_CTRL  = Control1;

} /* SetHighSpeed */

/*************************************************************************/
/*  InitDiskIOHardware                                                   */
/*                                                                       */
/*  Here the diskio interface is initialise.                             */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void InitDiskIOHardware(void)
{
   /*
    * Deselct before to prevent glitch
    */
   DESELECT();

   /* Slow during init */
   SetLowSpeed();

} /* InitDiskIOHardware */

/*************************************************************************/
/*  Now here comes some macros to speed up the transfer performance.     */
/*                                                                       */
/*            Be careful if you port this part to an other CPU.          */
/*             !!! This part is high platform dependent. !!!             */
/*************************************************************************/

/*
 * Transmit data only, without to store the receive data.
 * This function will be used normally to send an U8.
 */
#define TRANSMIT_U8(_dat)  SPI_TXR = (uint32_t)(_dat);     \
                           while(!(SPI_SR & SPI_SR_DONE));

/*
 * The next function transmit the data "very fast", because
 * we do not need to take care of receive data. This function
 * will be used to transmit data in 32 bit mode.
 */
#define TRANSMIT_FAST(_dat) SPI_TXR = (uint32_t)(_dat);    \
                            while(!(SPI_SR & SPI_SR_DONE));

/*
 * RECEIVE_FAST will be used in ReceiveDatablock only.
 */
#define RECEIVE_FAST(_dest)   SPI_TXR = (uint32_t)0xffffffff;     \
                              while( !( SPI_SR & SPI_SR_DONE ) ); \
                              *_dest++  = SPI_RXR;

/*************************************************************************/
/*  Set8BitTransfer                                                      */
/*                                                                       */
/*  Set Data Size of the SPI bus to 8 bit.                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void Set8BitTransfer(void)
{
   Control1 &= ~CTRL_BIT32;
   SPI_CTRL  = Control1;

} /* Set8BitTransfer */

/*************************************************************************/
/*  Set32BitTransfer                                                     */
/*                                                                       */
/*  Set Data Size of the SPI bus to 32 bit.                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void Set32BitTransfer(void)
{
   Control1 |= CTRL_BIT32;
   SPI_CTRL  = Control1;

} /* Set32BitTransfer */

/*************************************************************************/
/*  ReceiveU8                                                            */
/*                                                                       */
/*  Send a dummy value to the SPI bus and wait to receive the data.      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Data                                                         */
/*************************************************************************/
static uint8_t ReceiveU8 (void)
{
   SPI_TXR = (uint32_t) 0xff;

   /* wait for char */
   while (!(SPI_SR & SPI_SR_DONE)) ;

   return(SPI_RXR);
} /* ReceiveU8 */

/*************************************************************************/
/*  ReceiveDatablock                                                     */
/*                                                                       */
/*  Receive a data packet from MMC/SD card. Number of "btr" bytes will   */
/*  be store in the given buffer "buff". The byte count "btr" must be    */
/*  a multiple of 8.                                                     */
/*                                                                       */
/*  In    : buff, btr                                                    */
/*  Out   : none                                                         */
/*  Return: In case of an error return FALSE                             */
/*************************************************************************/
static int ReceiveDatablock(uint8_t * buff, uint32_t btr)
{
   uint8_t token, cnt;
   uint32_t *buff32 = (uint32_t*)buff;

   Timer1 = 10;
   do /* Wait for data packet in timeout of 100ms */
   {
      token = ReceiveU8();
   }
   while ((token == 0xFF) && Timer1);

   if (token != 0xFE)
      return(0);  /* If not valid data token, return with error */

   /* Receive the data block into buffer */
   Set32BitTransfer();

   /* Divide by 8 */
   cnt = btr >> 3;

   do /* Receive the data block into buffer */
   {
      RECEIVE_FAST(buff32);
      RECEIVE_FAST(buff32);
   }
   while (--cnt);

   Set8BitTransfer();
   ReceiveU8();   /* Discard CRC */
   ReceiveU8();   /* Discard CRC */

   return(1);  /* Return with success */
} /* ReceiveDatablock */

/*************************************************************************/
/*  TransmitDatablock                                                    */
/*                                                                       */
/*  Send a block of 512 bytes to the MMC/SD card.                        */
/*                                                                       */
/*  In    : buff, token (Data/Stop token)                                */
/*  Out   : none                                                         */
/*  Return: In case of an error return FALSE                             */
/*************************************************************************/
static int TransmitDatablock(const uint8_t * buff, uint8_t token)
{
   uint8_t resp, cnt = 0;
   uint32_t *buff32 = (uint32_t*)buff;

   if (WaitReady() != 0xFF)
      return(0);

   TRANSMIT_U8(token);  /* Xmit data token */
   if (token != 0xFD)   /* Is data token */
   {

      cnt = 512 / 8;

      /* Send the 512 byte data block */
      Set32BitTransfer();
      do /* Send the 512 byte data block */
      {
         TRANSMIT_FAST(*buff32++);
         TRANSMIT_FAST(*buff32++);
      }
      while (--cnt);

      Set8BitTransfer();
      TRANSMIT_U8(0xFF);   /* CRC (Dummy) */
      TRANSMIT_U8(0xFF);   /* CRC (Dummy) */

      resp = ReceiveU8();  /* Reveive data response */
      if ((resp & 0x1F) != 0x05) /* If not accepted, return with error */
      {
         return(0);
      }
   }

   return(1);  /* Return with success */
} /* TransmitDatablock */

/*************************************************************************/
/*  GetCDWP                                                              */
/*                                                                       */
/*  Return the status of the CD and WP socket pin.                       */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Data                                                         */
/*************************************************************************/
static uint32_t GetCDWP(void)
{
   uint32_t value = 0;
   uint32_t status;

   status = SPI_SR;

   if (status & STATUS_CD)
   {
      value |= SOCK_CD_EMPTY;
   }

   if (status & STATUS_WP)
   {
      value |= SOCK_WP_ACTIVE;
   }

   return(value);
} /* GetCDWP */

/*** EOF ***/
