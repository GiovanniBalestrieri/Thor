#include <iostream>
#include <unistd.h>
#include "hlserial_cpp.h"

int main()
{
	Serial serial;
	if(serial.connect("/dev/ttyACM0", 57600, SYNC_SEQ, BLK_READ) < 0)
	{
		std::cout << "Errore inizializzazione serial\n";
		return -1;
	}
	int i = 0;
	serial.flush_io();
	while(i < 10)
	{
		std::cout << "reading...\n";
		char * buff = NULL;
		int a = serial.serial_read(&buff, READ_ALL);
		if(a == 0)
		{
			std::cout << "Ricevuto\n";
			for(int j = 0; j< 58; j++)
			{
				std::cout << (char)buff[j];	
			}
			std::cout << "\n";
		}
		else{
			std::cout << "Non ricevuto\n";
		}
		
		serial.flush_io();
		free(buff);
		i++;
	}
	serial.disconnect();
	return 0;
}
