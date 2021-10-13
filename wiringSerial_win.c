/*
 * wiringSerial.c:
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "wiringSerial.h"

#ifdef _WIN32

int serialOpen (SerialPort_t *dev)
{

	DCB dcb;
	COMMTIMEOUTS commtimeouts;

	dev->hCom = CreateFile(dev->portName, GENERIC_READ | GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);

	if (dev->hCom == INVALID_HANDLE_VALUE)
	{
		ErrorPrintf("Can't open COM-Port %s ! - Error: %ld\n", dev->portName, GetLastError());
		exit(2);
	}

	DebugPrintf(3, "COM-Port %s opened...\n", dev->portName);

	GetCommState(dev->hCom, &dcb);
	dcb.BaudRate = dev->baudrate;//atol(dev->baud_rate);
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fOutX = FALSE;
	dcb.fInX = FALSE;
	dcb.fNull = FALSE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;

	// added by Herbert Demmel - iF CTS line has the wrong state, we would never send anything!
	dcb.fOutxCtsFlow = FALSE;
	dcb.fOutxDsrFlow = FALSE;

	if (SetCommState(dev->hCom, &dcb) == 0)
	{
		ErrorPrintf("Can't set baudrate %s ! - Error: %ld", dev->baudrate, GetLastError());
		exit(3);
	}

	/*
	 *  Peter Hayward 02 July 2008
	 *
	 *  The following call is only needed if the WaitCommEvent
	 *  or possibly the GetCommMask functions are used.  They are
	 *  *not* in this implimentation.  However, under Windows XP SP2
	 *  on my laptop the use of this call causes XP to freeze (crash) while
	 *  this program is running, e.g. in section 5/6/7 ... of a largish
	 *  download.  Removing this *unnecessary* call fixed the problem.
	 *  At the same time I've added a call to SetupComm to request
	 *  (not necessarity honoured) the operating system to provide
	 *  large I/O buffers for high speed I/O without handshaking.
	 *
	 *   SetCommMask(port->hCom,EV_RXCHAR | EV_TXEMPTY);
	 */
	SetupComm(dev->hCom, 32000, 32000);

	SetCommMask(dev->hCom, EV_RXCHAR | EV_TXEMPTY);

	commtimeouts.ReadIntervalTimeout = MAXDWORD;
	commtimeouts.ReadTotalTimeoutMultiplier = 0;
	commtimeouts.ReadTotalTimeoutConstant = 1;
	commtimeouts.WriteTotalTimeoutMultiplier = 0;
	commtimeouts.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(dev->hCom, &commtimeouts);
	if(dev->rxBuffC>0)
		cb_init(&dev->rxBuff,dev->rxBuffC,1);
	else{
		dev->rxBuffC = 256;
		cb_init(&dev->rxBuff,dev->rxBuffC,1);
	}

	if(dev->txBuffC>0)
		cb_init(&dev->txBuff,dev->txBuffC,1);
	else{
		dev->txBuffC = 256;
		cb_init(&dev->txBuff,dev->txBuffC,1);
	}

  return 0;
}


void  serialSetPort(SerialPort_t *dev,const char *port_name){
	strncpy(dev->portName,port_name,20);
}

void  serialSetBaud(SerialPort_t *dev,const int baud){
	dev->baudrate = baud;
}



void serialFlushRX (SerialPort_t *dev)
{
	PurgeComm(dev->hCom,  PURGE_RXABORT | PURGE_RXCLEAR);
	cb_flush(&dev->rxBuff);
}

void serialFlushTX (SerialPort_t *dev)
{
	PurgeComm(dev->hCom, PURGE_TXABORT | PURGE_TXCLEAR );
	cb_flush(&dev->txBuff);
}

void serialClose (SerialPort_t *dev)
{
	CloseHandle(dev->hCom);
	cb_free(&dev->rxBuff);
	cb_free(&dev->txBuff);
}



void serialPutchar (SerialPort_t *dev, const unsigned char c)
{
	int realsize;
	WriteFile(dev->hCom, &c, 1, &realsize, NULL);
}




int serialPuts (SerialPort_t *dev, const char *s)
{
	int realsize;
	WriteFile(dev->hCom, s, strlen(s), &realsize, NULL);
	return realsize;
}


int serialWriteBytes (SerialPort_t *dev, const char *s,int len)
{
	int realsize;
	WriteFile(dev->hCom, s, len, &realsize, NULL);
	return realsize;
}


void serialPrintf (SerialPort_t *dev, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (dev, buffer) ;
}




int serialDataAvail (SerialPort_t *dev)
{
	char buff[32];
	int r;
	int count;
	DebugPrintf(1,"startR\n");
	count = cb_availData(&dev->rxBuff);

	if(count>0)
		return count;

	count = cb_availFree(&dev->rxBuff);;
	ReadFile(dev->hCom, buff, count, &r, NULL);
	for(int i=0;i<r;i++){
		DebugPrintf(1,"0X%x ",buff[i]);
		cb_push_back(&dev->rxBuff,&buff[i]);

	}
	DebugPrintf(1,"\nendR\n");
	return r;
}




int serialGetchar (SerialPort_t *dev,char *c)
{

	if(cb_pop_front(&dev->rxBuff,c)==CB_SUCCESS)
	{
		return 1;
	}
	return  0;
}

int serialReadBytes(SerialPort_t *dev,  char *s,int len,int timeout){
	char buff[32];
	int r=0;
	int count;
	int index=0,tryRead=0;
	count = cb_availData(&dev->rxBuff);

	for(index=0;index<count && index<len;index++){

		cb_push_back(&dev->rxBuff,&s[index]);
		len--;
	}
//	DebugPrintf("len:%d\n",len);
	while(tryRead++<=1 && len>0){
		ReadFile(dev->hCom, buff, len, &r, NULL);
		for(int i=0;i<r;i++){
		//DebugPrintf(1,"0X%x ",buff[i]);
			s[index++] = buff[i];
		}
		len = len -r;
		if(len!=0)
			Sleep(timeout);
		DebugPrintf(1,"len:%d r:%d index:%d<<<\n",len,r,index);
	}
	DebugPrintf(1,"\nendR\n");
	return index;
}

#if 0// !defined INTEGRATED_IN_WIN_APP
void DebugPrintf(int level, const char *fmt, ...) {
	va_list ap;

	if (level <= DEBUG_LEVEL) {
		char pTemp[2000];
		va_start(ap, fmt);
		vprintf(fmt, ap);
		vsprintf(pTemp, fmt, ap);
		TRACE(pTemp);
		va_end(ap);
		//fflush(stdout);
	}
}

#endif

#endif
