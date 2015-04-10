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
		serialfileno = open( device, O_RDWR| O_NONBLOCK | O_NDELAY );
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
		tty.c_cc[VMIN]   =  1;
		/* timeout se non vengono letti almeno c_cc[VMIN] bytes */
		tty.c_cc[VTIME]  =  5;
		/* ???  funzionano... */
		tty.c_cflag     &=  ~PARENB;
		tty.c_cflag     &=  ~CSTOPB;
		tty.c_cflag     &=  ~CSIZE;
		tty.c_cflag     |=  CS8;

		tty.c_cflag     &=  ~CRTSCTS;
		tty.c_cflag     |=  CREAD | CLOCAL;     
		
		/* Eseguo il flush sulla porta per eliminare eventuali 
		 * vecchie scritture non terminate */
		
		if(tcflush(serialfileno, TCIOFLUSH) < 0)
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

char * cserial::serial_read(size_t size)
{
	if(size == READ_ALL)
	{
		return read_buffer();
	}
	else
	{
		return partial_read_buffer(size);
	}
}

char * cserial::read_buffer()
{	
	int linesize = STDBUFFSIZE;
	int i = 0;
	char c;
	char * buff = (char *) malloc(linesize*sizeof(char));
	if(buff == NULL)
	{
		return NULL;
	}
	while(1){
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n < 0)
		{
			if(errno != EINTR)
			{
				return NULL;
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
				return NULL;
			}
		}
		buff[i] = c;
		i++;
	}
	return buff;
}

char * cserial::partial_read_buffer(size_t size)
{
	int linesize = size;
	char c;
	int i = 0;
	char * buff = (char *) malloc(linesize*sizeof(char));
	if(buff == NULL)
	{
		return NULL;
	}
	while(linesize > 0){
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n < 0)
		{
			if(errno != EINTR)
			{
				return NULL;
			}
			else continue;
		}
		else if(n == 0)
		{
			/* Buffer finito prematuramente */
			return buff;
		}
		buff[i] = c;
		linesize -= n;
		i++;
	}
	return buff;
}

int cserial::write(void * buff, int fd)
{
	/* NON IMPLEMENTATA */
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
