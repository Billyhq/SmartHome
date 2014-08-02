#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <termios.h>
#include <errno.h>
#include <string.h>  //bzero

#define UART_DEV_NAME  "/dev/ttyAMA0"
#define UART_BAUD_RATE B9600
 

#define E(x...) do {                                        \
        fprintf(stderr, "%s(%d) ", __FUNCTION__, __LINE__); \
        fprintf(stderr, ##x);                               \
        0;                                            \
    } while(0)

int r_open(const char *router)
{
    struct termios options;

    int handle = open(router, O_RDWR|O_NOCTTY);   //block mode, nonblock mode if O_NONBLOCK is defined
    if(handle < 0)
    {
        E("error opening %s: %s\n", router, strerror(errno));
        return handle;
    }

    if( tcgetattr( handle,&options) !=  0)
    {
        E("tcgetattr error\n");
        return handle; 
    }

    cfsetispeed(&options, UART_BAUD_RATE); 
    cfsetospeed(&options, UART_BAUD_RATE);  

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;

    /* disable h/w and software flow control */
    options.c_cflag &= ~CRTSCTS;

    /*8bit data, 1 stop bit, no parity*/
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB; 

    options.c_iflag &= ~INPCK; //disable input parity checking    
    options.c_iflag &= ~(IXON | IXOFF | IXANY); 
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL );

    /* Shalle we also disbale implementation defined input/output processing */
    options.c_oflag &= ~OPOST;

	options.c_lflag &= ~IEXTEN;
    /* Choosing Raw Input */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL| ISIG);

    options.c_cc[VTIME] = 1; 
    options.c_cc[VMIN]  = 0; 

    tcflush(handle,TCIFLUSH);
    //change valid immediately
    if (tcsetattr(handle,TCSANOW,&options) != 0)  
    {
        E("tcsetattr error\n");
        return handle; 
    }

    return handle;
}

int main()
{
    int fd;
    int n = 0; 
    char read_buf[256];
    char write_buf[256];
    struct termios opt;
    const char *dev = UART_DEV_NAME;
    int read_time = 0;
    int one_frame_len = 0;

    fd = r_open(dev);
    if(fd == -1)
    {
    	  perror("open serial 0\n");
    	  exit(0);
    }

    printf("configure complete\n");
    printf("start send and receive data\n");
    bzero(read_buf, sizeof(read_buf)); //ÀàËÆÓÚmemset
    bzero(write_buf, sizeof(write_buf));

    while(1) 
    {
        while( (n = read(fd, read_buf, sizeof(read_buf))) > 0 )
	{
            read_time++;
            one_frame_len += n; 
        }
        if(read_time)
        {
            printf("one frame len is: %d\n", one_frame_len); 
            read_time = 0;
            one_frame_len = 0;
        } 
    }

    close(fd);

    return 0;
} 
