/*
 * filesys.h
 *
 *  Created on: 2017¦~7¤ë16¤é
 *      Author: Jason
 */

#ifndef SOURCE_FILESYS_H_
#define SOURCE_FILESYS_H_

int checkSdCard();
int filSysInit();
int fs_logData();
int writeRecord(char *str);
#endif /* SOURCE_FILESYS_H_ */
