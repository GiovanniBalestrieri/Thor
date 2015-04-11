#include "cserial.h"

extern int errno;

cserial::cserial()
{
	serialfileno = -1;
	connected = 0;
}

cserial::~cserial()
{
	close(serialfileno);
	serialfileno = -1;
	connected = 0;
}

int cserial::connect(char * device, int baudrate)
{
	if(serialfileno == -1 && device != NULL)
	{
		serialfileno = open( device, O_RDWR | O_NOCTTY | O_NDELAY);
		if(serialfileno < 0)
		{
			/* Errore nell'apertura del file */
			connected = 0;
			return -1;
		}
		struct termios tty;
		memset(&tty, 0, sizeof tty);
		
		if(tcgetattr(serialfileno, &tty) != 0)
		{
			/* Errore nell'impostazione degli attributi */
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}

		/* Imposto il baud Rate di input e di output */
		speed_t speed = intToBaud(baudrate);
		if(speed == -1)
		{
			/* Baudrate non valido */
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		if(cfsetospeed(&tty, speed) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		if(cfsetispeed(&tty, speed) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
	
		/* Imposto alcuni flag */
		/* minimo numero di byte da leggere */
		tty.c_cc[VMIN]   =  0;
		/* timeout se non vengono letti almeno c_cc[VMIN] bytes */
		tty.c_cc[VTIME]  =  0;
		// 8N1
	    	tty.c_cflag &= ~PARENB;
	    	tty.c_cflag &= ~CSTOPB;
	    	tty.c_cflag &= ~CSIZE;
	    	tty.c_cflag |= CS8;
	    	// no flow control
	    	tty.c_cflag &= ~CRTSCTS;

	    	tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	    	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	    	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	    	tty.c_oflag &= ~OPOST; // make raw
		/* Eseguo il flush sulla porta per eliminare eventuali 
		 * vecchie scritture non terminate */
		
		if(tcflush(serialfileno, TCIFLUSH) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		if(tcflush(serialfileno, TCOFLUSH) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		/* Imposto gli attributi sul file descriptor */
		if(tcsetattr(serialfileno, TCSANOW, &tty ) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		connected = 1;
		usleep(1000*1000);
		return 0;
	}
}

int cserial::disconnect()
{
	close(serialfileno);
	serialfileno = -1;
	connected = 0;
	return 0;
}

int cserial::is_connected()
{
	return connected;
}

int cserial::serial_read(char * buff, size_t size)
{
	if(size == READ_ALL)
	{
		int a = read_buffer(buff);
		return a;
	}
	else
	{
		return partial_read_buffer(buff, size);
	}
}

int cserial::read_buffer(char * buff)
{	
	int linesize = STDBUFFSIZE;
	int i = 0;
	char c;
	if(buff == NULL)
	{
		return -1;
	}
	int syncseqrec = 0;
	while(syncseqrec < 3)
	{
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n < 0)
		{
			if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
			{
				return -1;
			}
			else continue;
		}
		if(c == -86){
			syncseqrec++;
		}
		else
		{
			syncseqrec = 0;
		}
	}
	while(1){
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n < 0)
		{
			if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
			{
				return -1;
			}
			else continue;
		}
		else if(n == 0)
		{
			break;
		}
		
		if(i >= linesize)
		{
			linesize += linesize;
			buff = (char *)realloc(buff, linesize);
			if(buff == NULL)
			{
				return -1;
			}
		}
		buff[i] = c;
		i++;
	}
	return 0;
}

int cserial::partial_read_buffer(char * buff, size_t size)
{
	int linesize = size;
	char c;
	int i = 0;
	if(buff == NULL)
	{
		return -1;
	}
	int syncseqrec = 0;
	while(syncseqrec < 4)
	{
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n < 0)
		{

			if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
			{
				fprintf(stderr, "%d\n", errno);
				return -1;
			}
			else continue;
		}
		if(c == -86){
			syncseqrec++;
		}
		else
		{
			syncseqrec = 0;
		}
	}
	while(linesize > 0){
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n <= 0)
		{

			if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK && errno != 0)
			{
				return -1;
			}
			else continue;
		}
		buff[i] = c;
		linesize -= n;
		i++;
	}
	return 0;
}

int cserial::serial_write(char * buff, int fd, size_t size)
{
	size_t rem = size;
	while(rem > 0)
	{
		int n = write(serialfileno, buff, rem);
		if(n < 0){
			if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK && errno != 0)
			{
				return -1;
			}
			else continue;
		}
		rem -= n;
	}
	return 0;
}

speed_t cserial::intToBaud(int baud)
{
	if(baud == 9600) return B9600;
	else if(baud == 19200) return B19200;
	else if(baud == 38400) return B38400;
	else if(baud == 57600) return B57600;
	else if(baud == 115200) return B115200;
	else return -1;
}

char cserial::compute_checksum(char * buff, int size)
{
	int i;
	unsigned char sum = 0;
	for(i = 0; i<size; i++)
	{
		sum += buff[i];
	}
	return sum;
}
