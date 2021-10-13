/*
 * wiringSerial.h:
 *	Handle a serial port
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */
#ifndef _WIRING_SERIAL_H
#define _WIRING_SERIAL_H

#include "circular_buff.h"


#define DEBUG_LEVEL 1


#define XSTR(x) STR(x)
#define STR(x) #x

#if defined(_WIN32) && !defined(__CYGWIN__)
#define COMPILE_FOR_WINDOWS
#define COMPILED_FOR "Windows"
#elif defined(__CYGWIN__)
#define COMPILE_FOR_CYGWIN
#define COMPILED_FOR "Cygwin"
//#elif (defined(__arm__) || defined(__thumb__)) && (!defined(__raspi__))
//#define COMPILE_FOR_LPC21
//#define COMPILED_FOR "ARM"
//#define printf iprintf
#elif defined(__APPLE__)
#define COMPILE_FOR_LINUX
#define COMPILED_FOR "Apple MacOS X"
#elif defined(__FreeBSD__)
#define COMPILE_FOR_LINUX
#define COMPILED_FOR "FreeBSD"
#elif defined(__OpenBSD__)
#define COMPILE_FOR_LINUX
#define COMPILED_FOR "OpenBSD"
#else
#define COMPILE_FOR_LINUX
#define COMPILED_FOR "Linux"
#endif



#if defined COMPILE_FOR_WINDOWS || defined COMPILE_FOR_CYGWIN
#include <windows.h>
#include <io.h>
#endif // defined COMPILE_FOR_WINDOWS || defined COMPILE_FOR_CYGWIN

#if defined COMPILE_FOR_WINDOWS
#include <conio.h>
//#define TRACE(x) OutputDebugString(x)
#define TRACE(x) printf("%s",x)
#endif // defined COMPILE_FOR_WINDOWS

#if defined COMPILE_FOR_CYGWIN
//#define TRACE(x) OutputDebugString(x)
#define TRACE(x) printf("%s",x)
#endif // defined COMPILE_FOR_WINDOWS

#if defined COMPILE_FOR_LINUX
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
extern void Sleep(unsigned long MilliSeconds);
#define TRACE(x) fprintf(stderr, "%s",x)
#endif // defined COMPILE_FOR_LINUX

#if defined COMPILE_FOR_LINUX || defined COMPILE_FOR_CYGWIN
#include <termios.h>
#include <unistd.h>     // for read and return value of lseek
#include <sys/time.h>   // for select_time
extern int kbhit(void);
extern int getch(void);
extern struct termios keyboard_origtty;
#endif // defined COMPILE_FOR_LINUX || defined COMPILE_FOR_CYGWIN

#include <ctype.h>      // isdigit()
#include <stdio.h>      // stdout
#include <stdarg.h>
#include <time.h>
#if defined (COMPILE_FOR_LINUX)
	#if defined(__OpenBSD__)
	#include <errno.h>
	#else
	#include <sys/errno.h>
	#endif
#endif

#if defined COMPILE_FOR_LPC21
#include <stdlib.h>
#include <string.h>
#include <lpc_ioctl.h>  // if using libc serial port communication
#else
#include <fcntl.h>
#endif


#if defined INTEGRATED_IN_WIN_APP

#define ErrorPrintf(fmt, ...) AppDebugPrintf(1, fmt, ##__VA_ARGS__)
#define DebugPrintf AppDebugPrintf
void AppDebugPrintf(int level, const char *fmt, ...);

#define exit(val)   AppException(val)
void AppException(int exception_level);

int AppDoProgram(int argc, char *argv[]);

#define Exclude_kbhit 1
int AppSyncing(int trials);
void AppWritten(int size);

#else
#define ErrorPrintf(fmt, ...) DebugPrintf(0, fmt, ##__VA_ARGS__)
//void DebugPrintf(int level, const char *fmt, ...);
#define DebugPrintf(level,fmt,...)  printf (fmt,##__VA_ARGS__)
#endif



#if defined COMPILE_FOR_LINUX
#define stricmp strcasecmp
#define strnicmp strncasecmp
#endif // defined COMPILE_FOR_LINUX

#ifndef O_BINARY
#define O_BINARY 0
#endif // O_BINARY

#ifndef DWORD
#define DWORD unsigned long
#endif // DWORD


typedef struct {

	unsigned char ControlLines;
	unsigned char ControlLinesSwapped;
	unsigned char ControlLinesInverted;
	unsigned char LogFile;

	char portName[20];                  // Name of the serial port to use to
										// communicate with the microcontroller.
										// Read from the command line.

	unsigned char PrintSerialNumber;    // Print out the 128 bits serial number

	int baudrate; /**< Baud rate to use on the serial
	 * port communicating with the
	 * microcontroller. Read from the
	 * command line.                        */

#if defined COMPILE_FOR_WINDOWS || defined COMPILE_FOR_CYGWIN
	HANDLE hCom;
#endif // defined COMPILE_FOR_WINDOWS || defined COMPILE_FOR_CYGWIN

#if defined COMPILE_FOR_LINUX || defined COMPILE_FOR_LPC21
	int fdCom;
#endif // defined COMPILE_FOR_LINUX || defined COMPILE_FOR_LPC21

#if defined COMPILE_FOR_LINUX
	struct termios oldtio, newtio;
#endif // defined COMPILE_FOR_LINUX

#ifdef INTEGRATED_IN_WIN_APP
	unsigned char NoSync;
#endif

	unsigned serial_timeout_count; /**< Local used to track timeouts on serial port read. */

	circular_buffer rxBuff,txBuff;
	int rxBuffC;
	int txBuffC;
} SerialPort_t;


#ifdef __cplusplus
extern "C" {
#endif

extern void  serialSetRxBuff(int size);
extern void  serialSetTxBuff(int size);
extern void  serialSetPort   (SerialPort_t *dev,const char *device);
extern void  serialSetBaud   (SerialPort_t *dev,const int baud);
extern int   serialOpen      (SerialPort_t *dev) ;
extern void  serialClose     (SerialPort_t *dev) ;
extern void  serialFlushTX     (SerialPort_t *dev) ;
extern void  serialFlushRX     (SerialPort_t *dev) ;
extern void  serialPutchar   (SerialPort_t *dev, const unsigned char c) ;
extern int   serialPuts      (SerialPort_t *dev, const char *s) ;
extern void  serialPrintf    (SerialPort_t *dev, const char *message, ...) ;
extern int   serialDataAvail (SerialPort_t *dev) ;
extern int   serialGetchar (SerialPort_t *dev,char *c);
extern int   serialWriteBytes(SerialPort_t *dev, const char *s,int len);
extern int   serialReadBytes(SerialPort_t *dev,  char *s,int len,int timeout);

#ifdef __cplusplus
}
#endif

#endif //_WIRING_SERIAL_H
