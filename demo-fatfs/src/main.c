/**************************************************************************
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
#define __MAIN_C__

/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "talmem.h"
#include "ff.h"
#include "diskio.h"
#include "neorv32.h"

extern void disk_timerproc (void);

/*=======================================================================*/
/*  All Structures and Common Constants                                  */
/*=======================================================================*/

#define FS_MEMORY_SIZE  (64 * 1024)

#define BAUD_RATE       19200

#define LOGICAL_DRIVE   "0:/"

/*=======================================================================*/
/*  Definition of all local Data                                         */
/*=======================================================================*/

static volatile uint32_t dSystemTick = 0;

static FATFS   FSObject;
static uint8_t Buffer[(4*1024)];

static char    PrintfBuffer[128];

/*=======================================================================*/
/*  Definition of all local Procedures                                   */
/*=======================================================================*/

/*************************************************************************/
/*  SysTick_Handler                                                      */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SysTick_Handler (void)
{
   static uint16_t wTimer10ms = 0;

   /* Clear/ack pending FIRQ */
   neorv32_cpu_csr_write(CSR_MIP, ~(1<<GPTMR_FIRQ_PENDING));

   /* 1ms, system ticker */
   dSystemTick++;

   /* 10ms, timer */
   wTimer10ms++;
   if (10 == wTimer10ms)
   {
      wTimer10ms = 0;
      disk_timerproc();
   }

} /* SysTick_Handler */

/*************************************************************************/
/*  SysTickStart                                                         */
/*                                                                       */
/*  Start the SysTick.                                                   */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void SysTickStart (void)
{
   uint32_t ticks;

   /* Install GPTMR interrupt handler */
   neorv32_rte_handler_install(GPTMR_RTE_ID, SysTick_Handler);

   ticks = ((NEORV32_SYSINFO->CLK / 2) / 1000);

   /* Configure timer for 1000Hz ticks in continuous mode (with clock divisor = 2) */
   neorv32_gptmr_setup(CLK_PRSC_2, 1, ticks - 1);

   /*
    * Enable interrupt
    */

   /* Enable GPTMR FIRQ channel */
   neorv32_cpu_csr_set(CSR_MIE, 1 << GPTMR_FIRQ_ENABLE);

   /* Enable machine-mode interrupts */
   neorv32_cpu_csr_set(CSR_MSTATUS, 1 << CSR_MSTATUS_MIE);

} /* SysTickStart */

/*************************************************************************/
/*  OS_TimeDly                                                           */
/*                                                                       */
/*  Loop for a specified number of milliseconds,                         */
/*  will not release the CPU.                                            */
/*                                                                       */
/*  In    : TimeoutMs                                                    */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void OS_TimeDly (uint32_t dTimeoutMs)
{
   uint32_t dTimeEnd;

   dTimeEnd = dSystemTick + dTimeoutMs;
   while (dSystemTick <= dTimeEnd)
   {
      __asm__ volatile ("nop");
   }

} /* OS_TimeDly */

/*************************************************************************/
/*  OS_TimeGet                                                           */
/*                                                                       */
/*  Return the time from the system ticker.                              */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: Systick                                                      */
/*************************************************************************/
static uint32_t OS_TimeGet (void)
{
   return(dSystemTick);
} /* OS_TimeGet */

/*************************************************************************/
/*  term_printf                                                          */
/*                                                                       */
/*  This is a printf like output function.                               */
/*  The function is based on a CrossWorks for ARM example.               */
/*  Use "Customizing putchar" in the help function of CrossWorks.        */
/*                                                                       */
/*  In    : fmt                                                          */
/*  Out   : none                                                         */
/*  Return: Number of characters transmitted                             */
/*************************************************************************/
static int term_printf (const char *fmt, ...)
{
   int     n = 0; /* Number of characters transmitted */
   char   *buffer;
   va_list ap;

   va_start(ap, fmt);
   n = vsnprintf(PrintfBuffer, sizeof(PrintfBuffer), fmt, ap);
   va_end(ap);
   (void)ap;

   buffer = PrintfBuffer;
   while(*buffer != 0)
   {
      neorv32_uart0_putc(*buffer);
      buffer++;
   }

   return(n);
} /* term_printf */

/*************************************************************************/
/*  GetFreeInfo                                                          */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: FRESULT                                                      */
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
/*  Return: FRESULT                                                      */
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

/*************************************************************************/
/*  FsTest                                                               */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: none                                                         */
/*************************************************************************/
static void FsTest (void)
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

} /* FsTest */

/*=======================================================================*/
/*  All code exported                                                    */
/*=======================================================================*/

/*************************************************************************/
/*  main                                                                 */
/*                                                                       */
/*  In    : none                                                         */
/*  Out   : none                                                         */
/*  Return: never                                                        */
/*************************************************************************/
int main (void)
{
   uint8_t *pBuffer;

   /*
    * Capture all exceptions and give debug info via UART
    * this is not required, but keeps us safe.
    */
   neorv32_rte_setup();

   /* setup UART at default baud rate, no interrupts */
   neorv32_uart0_setup(BAUD_RATE, 0);

   /* Check if UART0 unit is implemented at all */
   if (0 == neorv32_uart0_available())
   {
      return(1);
   }

   /* Check if GPTMR unit is implemented at all */
   if (0 == neorv32_gptmr_available())
   {
      neorv32_uart0_puts("ERROR! General purpose timer not implemented!\n");
      return(1);
   }

   /*
    * Initialize the memory pool
    */
   tal_MEMInit();

   pBuffer = xmalloc(XM_ID_HEAP, FS_MEMORY_SIZE);
   tal_MEMAdd(XM_ID_FS, "FS", pBuffer, FS_MEMORY_SIZE);

   /* Start the SysTick */
   SysTickStart();

   /* Intro */
   neorv32_uart0_printf("\n\n<<< FatFs filesystem benchmark >>>\n\n");

   /* FatFs file system test */
   neorv32_uart0_printf("Benchmark started ...\n\n");

   FsTest();

   neorv32_uart0_printf("Benchmark end.\n");


   while (1)
   {
      __asm__ volatile ("nop");
   }

   /*
    * This return here make no sense.
    * But to prevent the compiler warning:
    *    "return type of 'main' is not 'int'
    * We use an int as return :-)
    */
   return(0); /*lint !e527*/
} /* main */

/*** EOF ***/
