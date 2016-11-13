/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "ff.h"

#define SDC_BURST_SIZE      16


/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
//static THD_WORKING_AREA(waThread1, 128);
//static THD_FUNCTION(Thread1, arg) {
//
//  (void)arg;
//  chRegSetThreadName("blinker");
//  while (true) {
//    palSetLine(LINE_ARD_D13);
//    chThdSleepMilliseconds(500);
//    palClearLine(LINE_ARD_D13);
//    chThdSleepMilliseconds(500);
//  }
//}

volatile unsigned int coreClock;

/*
 * Working area for driver.
 */
static uint8_t sd_scratchpad[512];

/*
 * SDIO configuration.
 */
static const SDCConfig sdccfg = {
  sd_scratchpad,
  SDC_MODE_4BIT
};

/* Buffer for block read/write operations, note that extra bytes are
   allocated in order to support unaligned operations.*/
static uint8_t buf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 4];

/* Additional buffer for sdcErase() test */
static uint8_t buf2[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE ];

/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
static FATFS SDC_FS;

/* Work register for fs command */
DWORD AccSize;
WORD AccFiles, AccDirs;

/* File info object. */
FILINFO Finfo;

/* FS mounted and ready.*/
static bool fs_ready = FALSE;

/* Generic large buffer.*/
static uint8_t fbuff[1024];

static FRESULT scan_files(BaseSequentialStream *chp, char *path) {
  DIR dirs;
  FRESULT res;
  BYTE i;
  char *fn;

  if ((res = f_opendir(&dirs, path)) == FR_OK) {

    i = strlen(path);

    while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]) {

      if (_FS_RPATH && Finfo.fname[0] == '.')
        continue;

      fn = Finfo.fname;

	  if (Finfo.fattrib & AM_DIR) {
	    AccDirs++;
	    *(path+i) = '/';
        strcpy(path+i+1, fn);
        res = scan_files(chp, path);
        *(path+i) = '\0';

        if (res != FR_OK)
          break;

      } else {
        chprintf(chp, "%s/%s (size: %lu kB)\r\n", path, fn, Finfo.fsize / 1024);
        AccFiles++;
        AccSize += Finfo.fsize;
      }
    }
  }

  return res;
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  coreClock = SystemCoreClock;
  //chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbStart(serusbcfg.usbp, &usbcfg);

  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Initializes the SDIO drivers.
   */
  sdcStart(&SDCD1, &sdccfg);

  /* Card presence check.*/
  while (!sdcIsCardInserted(&SDCD1)) {
    chThdSleepMilliseconds(100);
  }

  /* Connection to the card.*/
  if (sdcConnect(&SDCD1)) {
    while(1);
  }

  FRESULT err;
  DWORD fre_clust, fre_sect, tot_sect;
  FATFS *fsp;

  err = f_mount(&SDC_FS, "/", 1);

  if (err != FR_OK) {
    while(1);
  }

  fs_ready = TRUE;

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  BaseSequentialStream *chp = (BaseSequentialStream *)&SDU1;

  chThdSleepMilliseconds(2500);


  if (serusbcfg.usbp->state == USB_ACTIVE) {

    err = f_getfree("/", &fre_clust, &fsp);

    if (err != FR_OK) {
      chprintf(chp, "FS: f_getfree() failed\r\n");
    }
    else {
      tot_sect = (fsp->n_fatent - 2) * fsp->csize;
      fre_sect = fre_clust * fsp->csize;

      chprintf(chp,
               "FS: %lu free clusters with %lu sectors (%lu bytes) per cluster\r\n",
               fre_clust, (uint32_t)fsp->csize, (uint32_t)fsp->csize * 512);

      chprintf(chp,
               "    %lu bytes (%lu MB) free of %lu MB\r\n",
               fre_sect * 512,
               fre_sect / 2 / 1024,
               tot_sect / 2 / 1024);

      fbuff[0] = 0;
      scan_files(chp, (char *)fbuff);
    }

  }

  while (true) {
    palClearPad(GPIOC, GPIOC_LED1);
    palClearPad(GPIOC, GPIOC_LED2);
    palClearPad(GPIOC, GPIOC_LED3);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOC, GPIOC_LED1);
    palSetPad(GPIOC, GPIOC_LED2);
    palSetPad(GPIOC, GPIOC_LED3);
    chThdSleepMilliseconds(500);
  }
}
