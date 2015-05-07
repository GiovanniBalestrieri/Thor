#include "hlserial_cpp.h"

extern int errno;

Serial::Serial()
{
	serialfileno = -1;
	connected = 0;
	use_sync = -1;
}

Serial::~Serial()
{
	if(serialfileno != -1)
	{
		close(serialfileno);
		serialfileno = -1;
	}
	connected = 0;
	use_sync = -1;
}

int Serial::connect(char * device, int baudrate, int sync, int m)
{
	mode = m;
	blktimeo = 1000*100;
	/* sync sequence option */
	if(sync == SYNC_SEQ || sync == RAW_SEQ)
	{
		use_sync = sync;
		if(sync == SYNC_SEQ)
		{
			/* 
			 * if synchronization is required, a default 
			 * sequence is instantiated.
			 */
			sync_size = DEF_SYNC_SIZE;
			sync_seq = (char *)malloc(sync_size*sizeof(char));
			if(sync_seq == NULL)
			{
				return -1;
			}
			int y;			
			for(y = 0; y < sync_size; y++)
			{
				sync_seq[y] = DEF_SYNC_BYTE;
			}		
		}	
	}	
	else
	{
		/* bad value for sync parameter*/
		return -1;
	}
	/* check if there are no active connection */
	if(serialfileno == -1 && device != NULL)
	{
		serialfileno = open( device, O_RDWR | O_NOCTTY | O_NDELAY);
		if(serialfileno < 0)
		{
			/* file opening error */
			connected = 0;
			return -1;
		}
		struct termios tty;
		memset(&tty, 0, sizeof tty);
		
		if(tcgetattr(serialfileno, &tty) != 0)
		{
			/* attribute setting error */
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}

		/* Baud rate setting.
		 * guess the same baud rate for I/O
		 */
		speed_t speed = intToBaud(baudrate);
		if(speed == B0)
		{
			/* invalid baud rate */
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		/* output baud rate */
		if(cfsetospeed(&tty, speed) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		/* output baud rate */
		if(cfsetispeed(&tty, speed) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
	
		/* default flag setting */
		/* minumum byte count for correct message receiving */
		/* zero means no minimum byte count */
		tty.c_cc[VMIN]   =  0;
		/* timeout on minimum byte count */
		/* zero means non-blocking reading/writing*/
		tty.c_cc[VTIME]  =  0;
		/* standard encode parameters */
	    	tty.c_cflag &= ~PARENB;
	    	tty.c_cflag &= ~CSTOPB;
	    	tty.c_cflag &= ~CSIZE;
	    	tty.c_cflag |= CS8;
	    	/* no hardware flow control */
	    	tty.c_cflag &= ~CRTSCTS;
		/* allow read */
	    	tty.c_cflag |= CREAD | CLOCAL;
	    	/* disable software flow control */
	    	tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
		/* enable raw reading and disable echo response */
	    	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	    	tty.c_oflag &= ~OPOST;
		/* port flush */
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
		/* flag setting on fd */
		if(tcsetattr(serialfileno, TCSANOW, &tty ) < 0)
		{
			connected = 0;
			close(serialfileno);
			serialfileno = -1;
			return -1;
		}
		connected = 1;
		/* wait for port reset */
		sleep(1);
	}
	return 0;
}

int Serial::disconnect()
{
	/* close the file descriptor */
	if(serialfileno != -1)
	{
		close(serialfileno);
		serialfileno = -1;
	}
	connected = 0;
	use_sync = -1;
	return 0;
}

int Serial::is_connected()
{
	return connected;
}

int Serial::serial_read(char ** buff, int size)
{
	/* check read size to start correct function */
	if(mode == BLK_READ)
	{
		int ret;
		do
		{
			if(size == READ_ALL)
			{
				ret = read_buffer(buff);
			}
			else
			{
				ret = partial_read_buffer(*buff, size);
			}
			if(ret == 0)
			{
				usleep(blktimeo);
			}
		}while(ret == 0);
		return 0;
	}
	else if(mode == NBLK_READ)
	{
		if(size == READ_ALL)
		{
			return read_buffer(buff);
		}
		else
		{
			return partial_read_buffer(*buff, size);
		}
	}
	return -1;
}

int Serial::flush_io()
{
	if(tcflush(serialfileno, TCIFLUSH) < 0 || tcflush(serialfileno, TCOFLUSH) < 0) return -1;

	return 0;
}

int Serial::read_buffer(char ** buff)
{	
	/* read all the input buffer */
	int linesize = STDBUFFSIZE;
	int i = 0;
	char c;
	*buff = (char *)malloc(linesize*sizeof(char));
	if(buff == NULL)
	{
		return -1;
	}
	if(use_sync == SYNC_SEQ){
	/* if synchronisation is required, ignore every sequence
	 * that is different from sync_seq */
		int syncseqrec = 0;
		
		while(syncseqrec < sync_size)
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
			else if(n == 0)
			{
				/* if zero is returned, the transmission
				 * has stopped without using the syncronisation
				 * sequence 
				 */

				return 0;
			}
			if(c == sync_seq[syncseqrec]){
				/* if c matches the correct sync character, 
				 * counter is rised 
				 */
				syncseqrec++;
			}
			else
			{
				/* otherwise counter is set to 0 */
				syncseqrec = 0;
			}
		}
	}
	while(1){
		errno = 0;
		int n = read(serialfileno, &c, 1);
		if(n < 0)
		{
			if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
			{
				/* check if reading was interrupted by signals */
				return -1;
			}
			else continue;
		}
		else if(n == 0)
		{
			/* transmission terminated */
			break;
		}
		if(i >= linesize)
		{
			/* byff is dynamically allocated, so it's dimension
			 * is doubled before every potential overflow
			 */
			linesize += linesize;
			*buff = (char *)realloc(*buff, linesize);
			if(buff == NULL)
			{
				return -1;
			}
		}
		(*buff)[i] = c;
		i++;
	}
	return i;
}

int Serial::partial_read_buffer(char * buff, size_t size)
{
	/* same as read_buffer but buff is statically allocated
	 * and reading is interrupted to size-th character
	 */
	int linesize = size;
	char c;
	int i = 0;
	if(buff == NULL)
	{
		return -1;
	}
	if(use_sync == SYNC_SEQ)
	{
		int syncseqrec = 0;
		while(syncseqrec < sync_size)
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
			else if(n == 0)
			{
				/* if zero is returned, the transmission
				 * has stopped without using the syncronisation
				 * sequence 
				 */

				return 0;
			}
			if(c == sync_seq[syncseqrec]){
				syncseqrec++;
			}
			else
			{
				syncseqrec = 0;
			}
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
	return i;
}

int Serial::serial_write(char * buff, size_t size)
{
	/* write all possible character on the channel
	 * until size is reached
	 */
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

speed_t Serial::intToBaud(int baud)
{
	/* converts int representation of baud rate
	 * to termios representation
	 */
	if(baud == 9600) return B9600;
	else if(baud == 19200) return B19200;
	else if(baud == 38400) return B38400;
	else if(baud == 57600) return B57600;
	else if(baud == 115200) return B115200;
	else return B0;
}

char Serial::compute_checksum(char * buff, int size)
{
	/* not implemented yet */
	int i;
	unsigned char sum = 0;
	for(i = 0; i<size; i++)
	{
		sum += buff[i];
	}
	return sum;
}

int Serial::set_sync_sequence(char * sequence, int size)
{
	/* set use_sync to SYNC_SEQ and allocates the sequence
	 * array to size dimention, then sync_seq is filled with
	 * user defined sequence
	 */
	
	use_sync = SYNC_SEQ;
	sync_seq = (char *)realloc(sync_seq, size*sizeof(char));
	if(sync_seq == NULL)
	{
		return -1;
	}
	memcpy(sync_seq, sequence, size);
	return 0;
}

void Serial::set_polling_t(int t)
{
	mode = BLK_READ;
	blktimeo = t;
}

void Serial::set_nonblocking()
{
	mode = NBLK_READ;
}
