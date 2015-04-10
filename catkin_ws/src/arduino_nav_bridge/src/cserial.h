#ifndef CSERIAL_H
#define CSERIAL_H

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#define READ_ALL -1
#define STDBUFFSIZE 255

class cserial
{
	private:
		int connected;
		int serialfileno;
		speed_t intToBaud(int);
		char * read_buffer();
		char * partial_read_buffer(size_t);
	public:
		cserial();
		~cserial();
		int connect(char *, int);
		int disconnect();
		char * serial_read(size_t);
		int write(void *, int);
		int is_connected();
};

#endif