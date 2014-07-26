#include <stdio.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <termios.h>
#include <errno.h>
#include "pi.h"

#define UART_DEV_NAME  "/dev/ttyAMA0"
#define UART_BAUD_RATE B38400

 #define UART_FRAME_SOF              0xFE
 
enum
{
    UART_FRAME_CMDID_SWITCH,
    UART_FRAME_CMDID_NUM
};

#define E(x...) do {                                        \
        fprintf(stderr, "%s(%d) ", __FUNCTION__, __LINE__); \
        fprintf(stderr, ##x);                               \
        0;                                            \
    } while(0)

int r_open(const char *router)
{
    struct termios options;

    int handle = open(router, O_RDWR|O_NOCTTY|O_NDELAY);
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


    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB; 

    options.c_iflag &= ~INPCK;    
    options.c_iflag &= ~(IXON | IXOFF | IXANY); 
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL );

    /* Shalle we also disbale implementation defined input/output processing */
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~IEXTEN;

    /* We know RPC msg are always 32bit aligned so that read at least 4 bytes.
       Timeout value is TBD  */
    options.c_cc[VTIME] = 0; 
    options.c_cc[VMIN]  = 0; 

    tcflush(handle,TCIFLUSH);

    /* Choosing Raw Input */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL| ISIG);

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
    int i;
    int len;
    int n = 0; 
    char read_buf[256];
    char write_buf[256];
    struct termios opt;
    char localStatus = 0xFE;
    char downLoadStatus; 
    char downLoadFlag = 0x00;
    const char *dev = UART_DEV_NAME;

#if 1
    fd = r_open(dev);
    if(fd == -1)
    {
    	  perror("open serial 0\n");
    	  exit(0);
    }
#else
    fd = open(UART_DEV_NAME, O_RDWR | O_NOCTTY); //默认为阻塞读方式
    tcgetattr(fd, &opt); 
    cfsetispeed(&opt, UART_BAUD_RATE);
    cfsetospeed(&opt, UART_BAUD_RATE);

    if(tcsetattr(fd, TCSANOW, &opt) != 0 )
    { 
    	  perror("tcsetattr error");
    	  return -1;
    }
    /*8位数据位，1位停止位，没有校验位*/
    opt.c_cflag &= ~CSIZE; 
    opt.c_cflag |= CS8; 
    opt.c_cflag &= ~CSTOPB; 
    opt.c_cflag &= ~PARENB; 
    //opt.c_cflag &= ~INPCK;
    opt.c_cflag |= (CLOCAL | CREAD);

    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    /* disable h/w and software flow control */
    opt.c_cflag &= ~CRTSCTS;  //new 6.17

    opt.c_oflag &= ~OPOST;
    opt.c_oflag &= ~(ONLCR | OCRNL); //添加的
    opt.c_iflag &= ~INPCK;    
    opt.c_iflag &= ~(IXON | IXOFF | IXANY); 
    opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                         | INLCR | IGNCR | ICRNL );

    opt.c_cc[VTIME] = 0;
    opt.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);
    if(tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        perror("serial error");
        return -1;
    }
    
#endif

    printf("configure complete\n");
    printf("start send and receive data\n");
    bzero(read_buf, sizeof(read_buf)); //类似于memset
    bzero(write_buf, sizeof(write_buf));
#if 0    
    write_buf[0] = UART_FRAME_SOF;
    write_buf[1] = UART_FRAME_CMDID_SWITCH;  //cmdID
    write_buf[2] = 0x01;                     //payload len
    write_buf[3] = 0x01;
    write(fd, write_buf, 4);
//#else
    write_buf[0] = 0x00;
    write_buf[1] = 0x55;  //cmdID
    write_buf[2] = 0xAA;                     //payload len
    write_buf[3] = 0xFF;
    write(fd, write_buf, 4);
#endif
    //close(fd);
    
    //return 0;
    
    //report_status((int)1);
    while( (n = read(fd, read_buf, sizeof(read_buf))) > 0 )
	{
	    read_buf[len] = '\0';
		printf("Len %d \n", len);
        printf("%s \n", read_buf);
    }

#if 0   
    while(1)
    { 
        n = 0;
        len = 0;
#if 0        
        downLoadFlag = 0x00;
        check_instruction( read_buf );
        downLoadStatus = read_buf[0];
        printf( "download status is %c: \n", downLoadStatus );
        if( downLoadStatus == '1' || downLoadStatus == '2' || downLoadStatus == '3' ) 
        {
        	  if( downLoadStatus != localStatus )
        	  {
        	      downLoadFlag = 0x01;
        	      localStatus = downLoadStatus;
        	  }
            if( downLoadStatus == '1' || downLoadStatus == '3' )   //start
            {
                write_buf[3] = 0x01;
            }
            else if( downLoadStatus == '2' )
            {
                write_buf[3] = 0x00;
            }
        }
#else
        while( (n = read(fd, read_buf, sizeof(read_buf))) > 0 )
        {
          	for(i = len; i < (len + n); i++)
          	{
        	  	write_buf[i] = read_buf[i - len];
        	  }
        	  len += n;
        }
        write_buf[len] = '\0';

        printf("Len %d \n", len);
        printf("%s \n", write_buf);

        //n = write(fd, write_buf, len);
        //printf("write %d chars\n",n);
#endif
        if( downLoadFlag )
        {
        	  write(fd, write_buf, 4);
        	  for( i = 0; i < 4; i++ )
        	      printf("send to sensor: %d  ",write_buf[i]);
        	  printf("\n");
        }
        sleep(2);
    }
#endif
    return 0;
} 
