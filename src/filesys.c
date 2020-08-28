/*
 * filesys.c
 *
 *  Created on: 2017¦~7¤ë16¤é
 *      Author: Jason
 */

#include "ch.h"
#include "hal.h"
#include <string.h>
#include "filesys.h"
#include "ff.h"
#include "sysParam.h"
#include "time.h"

unsigned long sdWriteOffset;
FATFS SDC_FS;
FRESULT fres;
FIL sdFile;
DIR dir;
char fileName[32];

time_t getTimeUnixSec(void)
{
  struct tm tim;
  RTCDateTime timespec;
  rtcGetTime(&RTCD1,&timespec);
  rtcConvertDateTime(&timespec,&tim,NULL);
  return mktime(&tim);

}

void changeLogFile()
{
  RTCDateTime timespec;
  struct tm tim;
  rtcGetTime(&RTCD1,&timespec);
  rtcConvertDateTimeToStructTm(&timespec,&tim,NULL);
  sprintf(fileName,"%02d_%02d_%02d %02d_%02d_%02d.log",tim.tm_year+1980, tim.tm_mon, tim.tm_mday, tim.tm_hour, tim.tm_min, tim.tm_sec);

  moduleParam.csvLastCreationTime = mktime(&tim);
//  if(acqState.fileID == 0){
//    RTCDateTime timespec;
//    struct tm tim;
//    rtcGetTime(&RTCD1,&timespec);
//    rtcConvertDateTimeToStructTm(&timespec,&tim,NULL);
//  }
  //sprintf(fileName,"%s-%s-03D",sysParam.filter)
}

void validLogFileName()
{
  RTCDateTime timespec;
  struct tm tim;
  rtcGetTime(&RTCD1,&timespec);
  time_t logFileInterval;
  switch(moduleParam.logFileIntervalType){
  case 0:   // minutes
    logFileInterval = moduleParam.logFileInterval * 60;
    break;
  case 1:   // hour
    logFileInterval = moduleParam.logFileInterval * 3600;
    break;
  case 2:   // day
    logFileInterval = moduleParam.logFileInterval * 86400;
    break;
  }

  rtcConvertDateTimeToStructTm(&timespec,&tim,NULL);
  tim.tm_sec = 0;

  unsigned long delta = mktime(&tim) - moduleParam.logFileInitTime;
  if(delta % logFileInterval == 0){
    sprintf(fileName,"%04d_%02d_%02d.log",tim.tm_year+1900, tim.tm_mon+1, tim.tm_mday, tim.tm_hour, tim.tm_min, tim.tm_sec);
  }


  //moduleParam.csvLastCreationTime = mktime(&tim);
//  if(acqState.fileID == 0){
//    RTCDateTime timespec;
//    struct tm tim;
//    rtcGetTime(&RTCD1,&timespec);
//    rtcConvertDateTimeToStructTm(&timespec,&tim,NULL);
//  }
  //sprintf(fileName,"%s-%s-03D",sysParam.filter)
}

#define CSV_HEADER  "Date,Time,CH0,CH1,CH2,CH3,CH4,CH5,CH6,CH7\r\n"

int writeRecord(char *str)
{
  FIL fil;
  FRESULT fres;
  UINT Size;
  validLogFileName();

  fres = f_open(&fil,fileName,FA_WRITE);
  if(fres != FR_OK){
    // try to create new file
    fres = f_open(&fil,fileName, FA_CREATE_NEW);
    if(fres != FR_OK){
      return -1;
    }
  }

  if(fil.obj.objsize == 0){
    f_write(&fil,CSV_HEADER,strlen(CSV_HEADER),&Size);
  }

  fres = f_lseek(&fil,fil.obj.objsize);
  f_write(&fil,str,strlen(str),&Size);
  f_close(&fil);

  return 0;
}

int fs_logData(char *timeStr)
{
  char str[256];

}

int sdSaveFile(char *data, int len)
{
//  UINT Size;
//  uint32_t writeLen = acqState.sdRecordLimitInBytes - sdWriteOffset;
//  writeLen = (writeLen > len)?len:writeLen;
//
//  fres =  f_lseek(&sdFile,data,writeLen,&Size);
//
//  if(fres != FR_OK){
//    f_close(&sdFile);
//    return;
//  }else{
//    sdWriteOffset += writeLen;
//    if(sdWriteOffset == acqState.sdRecordLimitInBytes){
//      return 1;
//    }
//  }

  return 0;
}

int checkSdCard()
{
  f_mount(&SDC_FS,"0",1);
  fres = f_opendir(&dir,"/");
  if(fres == FR_OK){
    return 1;
    sdFile.err = 0x1;
  }
  return 0;
}

void filCheckLogFileSize()
{
//  if(sysParam.recordMinutes == 0){
//    sysParam.recordMinutes = 10;
//  }
//
//  if(sysParam.sampleRate == 1000){
//    if(sysParam.recordMinutes > 5)
//      sysParam.recordMinutes = 5;
//  }
//
//  acqState.sdRecordLimitInBytes = sysParam.sampleRate*60*sysParam.recordMinutes;
//
//  if(sysParam.axis == 3){
//    acqState.sdRecordLimitInBytes*=12;
//  }
//  else{
//    acqState.sdRecordLimitInBytes*=6;
//  }
//
//  acqState.sdRecordLimitInBytes += SD_HEADER_OFFSET;
}

void fileSysTest()
{

}

int filSysInit()
{
    sdcStart(&SDCD1,NULL);
    // mount
    FRESULT err;
    //DIR dir;


    sdcConnect(&SDCD1);
    err = f_mount(&SDC_FS,"0",1);
    if(err != FR_OK){
      sdcDisconnect(&SDCD1);
      return 1;
    }

    return 0;

}
