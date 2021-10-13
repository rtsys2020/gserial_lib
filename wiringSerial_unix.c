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
#include <errno.h>
//#define __unix__  1
#ifdef __unix__
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "wiringSerial.h"

int serialOpen (SerialPort_t *dev)
{
	speed_t myBaud ;
	printf("baudrate:%d\n",dev->baudrate);
	switch (dev->baudrate)
	{
	case     50:	myBaud =     B50 ; break ;
	case     75:	myBaud =     B75 ; break ;
	case    110:	myBaud =    B110 ; break ;
	case    134:	myBaud =    B134 ; break ;
	case    150:	myBaud =    B150 ; break ;
	case    200:	myBaud =    B200 ; break ;
	case    300:	myBaud =    B300 ; break ;
	case    600:	myBaud =    B600 ; break ;
	case   1200:	myBaud =   B1200 ; break ;
	case   1800:	myBaud =   B1800 ; break ;
	case   2400:	myBaud =   B2400 ; break ;
	case   4800:	myBaud =   B4800 ; break ;
	case   9600:	myBaud =   B9600 ; break ;
	case  19200:	myBaud =  B19200 ; break ;
	case  38400:	myBaud =  B38400 ; break ;
	case  57600:	myBaud =  B57600 ; break ;
	case 115200:	myBaud = B115200 ; break ;
	case 230400:	myBaud = B230400 ; break ;

	default:
		return -2 ;
	}
	//O_NONBLOCK
	printf("open port %s\n",dev->portName);
	dev->fdCom = open(dev->portName, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK );

	if (dev->fdCom < 0) {
		int err = errno;
		printf("Can't open COM-Port %s ! (Error: %dd (0x%X))\n",
				dev->portName, err, err);
		fflush(stdout);
		return -1;
	}
	printf("file dev:%d\n",dev->fdCom);

	fcntl (dev->fdCom, F_SETFL, O_RDWR) ;

	tcgetattr( dev->fdCom, &dev->newtio );

	cfmakeraw   (&dev->newtio) ;
	cfsetispeed (&dev->newtio, myBaud) ;
	cfsetospeed (&dev->newtio, myBaud) ;

	dev->newtio.c_cflag |= (CLOCAL | CREAD) ;
	dev->newtio.c_cflag &= ~PARENB ;
	dev->newtio.c_cflag &= ~CSTOPB ;
	dev->newtio.c_cflag &= ~CSIZE ;
	dev->newtio.c_cflag |= CS8 ;
	dev->newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	dev->newtio.c_oflag &= ~OPOST ;

	dev->newtio.c_cc [VMIN]  =   0 ;
	dev->newtio.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

	tcsetattr (dev->fdCom, TCSANOW | TCSAFLUSH, &dev->newtio) ;

	int     status;
	ioctl (dev->fdCom, TIOCMGET, &status);

	status |= TIOCM_DTR ;
	status |= TIOCM_RTS ;

	ioctl (dev->fdCom, TIOCMSET, &status);

	usleep (10000) ;	// 10mS	
	//	//DebugPrintf(3, "COM-Port %s opened...\n", port->serial_port);
	//
	//	/* clear input & output buffers, then switch to "blocking mode" */
	//	tcflush(dev->fdCom, TCOFLUSH);
	//	tcflush(dev->fdCom, TCIFLUSH);
	//	fcntl(dev->fdCom, F_SETFL, fcntl(dev->fdCom, F_GETFL) & ~O_NONBLOCK);
	//
	//	tcgetattr(dev->fdCom, &dev->oldtio); /* save current port settings */
	//
	//	bzero(&dev->newtio, sizeof(dev->newtio));
	//	dev->newtio.c_cflag = CS8 | CLOCAL | CREAD;
	//
	//#if defined(__FreeBSD__) || defined(__OpenBSD__)
	//
	//	if (cfsetspeed(&dev->newtio,(speed_t) (dev->baudrate))) {
	//		ErrorPrintf(, "baudrate %d not supported\n", dev->baudrate);
	//		return -2;
	//		//exit(3);
	//	};
	//#else
	//
	//#ifdef __APPLE__
	//#define NEWTERMIOS_SETBAUDARTE(bps) dev->newtio.c_ispeed = dev->newtio.c_ospeed = bps;
	//#else
	//#define NEWTERMIOS_SETBAUDARTE(bps) dev->newtio.c_cflag |= bps;
	//#endif
	//
	//	switch ((dev->baudrate)) {
	//#ifdef B115200
	//	case 1152000:
	//		NEWTERMIOS_SETBAUDARTE(B115200)
	//		;
	//		break;
	//#endif // B1152000
	//#ifdef B57600
	//	case 576000:
	//		NEWTERMIOS_SETBAUDARTE(B57600)
	//		;
	//		break;
	//#endif // B576000
	//#ifdef B230400
	//	case 230400:
	//		NEWTERMIOS_SETBAUDARTE(B230400)
	//		;
	//		break;
	//#endif // B230400
	//#ifdef B115200
	//	case 115200:
	//		NEWTERMIOS_SETBAUDARTE(B115200)
	//		;
	//		break;
	//#endif // B115200
	//#ifdef B57600
	//	case 57600:
	//		NEWTERMIOS_SETBAUDARTE(B57600)
	//		;
	//		break;
	//#endif // B57600
	//#ifdef B38400
	//	case 38400:
	//		NEWTERMIOS_SETBAUDARTE(B38400)
	//		;
	//		break;
	//#endif // B38400
	//#ifdef B19200
	//	case 19200:
	//		NEWTERMIOS_SETBAUDARTE(B19200)
	//		;
	//		break;
	//#endif // B19200
	//#ifdef B9600
	//	case 9600:
	//		NEWTERMIOS_SETBAUDARTE(B9600)
	//		;
	//		break;
	//#endif // B9600
	//
	//		// Special value
	//		// case   32000: NEWTERMIOS_SETBAUDARTE(32000); break;
	//
	//	default: {
	//		printf("unknown baudrate %d\n", dev->baudrate);
	//		return -2;
	//		//exit(3);
	//	}
	//	}
	//
	//#endif
	//
	//	dev->newtio.c_iflag = IGNPAR | IGNBRK | IXON | IXOFF;
	//	dev->newtio.c_oflag = 0;
	//
	//	/* set input mode (non-canonical, no echo,...) */
	//	dev->newtio.c_lflag = 0;
	//
	//	cfmakeraw(&dev->newtio);
	//	dev->newtio.c_cc[VTIME] = 1; /* inter-character timer used */
	//	dev->newtio.c_cc[VMIN] = 1; /* blocking read until 0 chars received */
	//
	//	tcflush(dev->fdCom, TCIFLUSH);
	//	if (tcsetattr(dev->fdCom, TCSANOW, &dev->newtio)) {
	//		ErrorPrintf("Could not change serial port behaviour (wrong baudrate?)\n");
	//		return -3;
	//		//exit(3);
	//	}

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
	tcflush (dev->fdCom, TCIOFLUSH) ;
	cb_flush(&dev->rxBuff);
}

void serialFlushTX (SerialPort_t *dev)
{
	tcflush (dev->fdCom, TCIOFLUSH) ;
	cb_flush(&dev->txBuff);
}

void serialClose (SerialPort_t *dev)
{
	tcflush(dev->fdCom, TCOFLUSH);
	tcflush(dev->fdCom, TCIFLUSH);
	tcsetattr(dev->fdCom, TCSANOW, &dev->oldtio);

	close(dev->fdCom);
	cb_free(&dev->rxBuff);
	cb_free(&dev->txBuff);
}



void serialPutchar (SerialPort_t *dev, const unsigned char c)
{

	write(dev->fdCom, &c, 1);
}




int serialPuts (SerialPort_t *dev, const char *s)
{
	int realsize;

	realsize = write(dev->fdCom, s,strlen(s));
	return realsize;
}


int serialWriteBytes (SerialPort_t *dev, const char *s,int len)
{
	int realsize;
	realsize = write(dev->fdCom, s, len);
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

	count = cb_availFree(&dev->rxBuff);
	r = read(dev->fdCom, buff, count);

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
	printf("--start--%d -- count:%d\n",dev->fdCom,count);
	for(index=0;index<count && index<len;index++){

		cb_push_back(&dev->rxBuff,&s[index]);
		len--;
	}


	while(tryRead++<2 && len>0){
		r = read(dev->fdCom, buff, len);
		if(r>0){
			printf("read %d bytes\n",r);
			for(int i=0;i<r;i++){
				s[index++] = buff[i];
			}
			len = len -r;
			printf("sleep\n");
		}else if(r<0){
			printf("error %s\n",strerror(errno));

		}else{
			printf("timeout\n");
			break;
		}
		if(len!=0)
			usleep(timeout);

	}
	DebugPrintf(1,"\nendR\n");
	return index;
}

#endif
