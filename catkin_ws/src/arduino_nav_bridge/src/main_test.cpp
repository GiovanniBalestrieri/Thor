#include <iostream>
#include "cserial.h"

int main()
{
	cserial serial;
	if(serial.connect("/dev/ttyACM1", 57600) < 0)
	{
		std::cout << "Errore inizializzazione serial\n";
		return -1;
	}
	int i = 0;
	while(i < 600)
	{
		std::cout << "reading...\n";
		char * buff = (char *) malloc(56*sizeof(char));
		if(buff == NULL)
		{
			std::cout << "Errore di allocazione";
			return -1;
		}
		int a = serial.serial_read(buff, 56);
		if(a == 0)
		{
			std::cout << "Ricevuto\n";
			for(int j = 0; j< 55; j++)
			{
				std::cout << (int)buff[j];	
			}
			std::cout << "\n";
		}
		else{
			std::cout << "Non ricevuto\n";
		}
		free(buff);
		i++;
	}
	serial.disconnect();
	return 0;
}
