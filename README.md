#include "wiringSerial.h"

SerialPort_t port;

serialSetPort(&port,"/dev/ttyS1");

serialSetBaud(&port,115200);

if(serialOpen(&port)!=0){

	printf("error open port kill task\n");
	
	fflush(stdout);
	
	return -1;
	
}


serialFlushRX(&port);

serialPuts(&port, (const char *) "\r\n");
serialReadBytes(&port,(char*)rx_buff,100,1000) ;

