#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>

#define MSG_INTERVAL   3   //send msg every 3s
/*This is for UART device*/
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


static int client_sockfd;//客户端套接字;
static char client2server_data[] = "Client To Server";

static void Client2ServerFn(int sig)
{
    if(send(client_sockfd, client2server_data, sizeof(client2server_data),0) < 0)
    {
        printf("Error! client to server.\n");
    }
    else
    {
        printf("OK! client to server.\n");
        alarm(MSG_INTERVAL);
    }
}

static void send_msg2_server(char *read_buf, int len)
{

    if(send(client_sockfd, read_buf, len,0) < 0)
    {
        printf("Error! client to server.\n");
    }
    else
    {
        printf("OK! client to server, length is %d.\n", len);
    }
}

int main(int argc, char *argv[])
{
    int uart_fd;
    int uart_tmp_len = 0; 
    char read_buf[256];
    char write_buf[256];
    struct termios opt;
    const char *dev = UART_DEV_NAME;
    int read_time = 0;
    int one_frame_len = 0;

	int len;
	struct sockaddr_in remote_addr; //服务器端网络地址结构体
	char buf[BUFSIZ];  //数据传送的缓冲区
    fd_set readfd;
    int maxfd;
    struct timeval timeout = {0,0};


    uart_fd = r_open(dev);
    if(uart_fd == -1)
    {
    	  perror("open serial fail\n");
    	  exit(0);
    }
    else
        printf("uart open sucessfully\n");

	memset(&remote_addr,0,sizeof(remote_addr)); //数据初始化--清零
	remote_addr.sin_family=AF_INET; //设置为IP通信
	//remote_addr.sin_addr.s_addr=inet_addr("54.213.192.93");//服务器IP地址
	remote_addr.sin_addr.s_addr=inet_addr("115.28.154.195");//服务器IP地址
	remote_addr.sin_port=htons(8000); //服务器端口号
	
	/*创建客户端套接字--IPv4协议，面向连接通信，TCP协议*/
	if((client_sockfd=socket(PF_INET,SOCK_STREAM,0))<0)
	{
		perror("socket");
		return 1;
	}
	
	/*将套接字绑定到服务器的网络地址上*/
	if(connect(client_sockfd,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr))<0)
	{
		perror("connect");
		return 1;
	}
	printf("connected to server\n");
	//len=recv(client_sockfd,buf,BUFSIZ,0);//接收服务器端信息
    //buf[len]='\0';
	//printf("%s",buf); //打印服务器端信息

    //signal(SIGALRM, Client2ServerFn);
    //alarm(MSG_INTERVAL);	
	/*循环的发送接收信息并打印接收信息--recv返回接收到的字节数，send返回发送的字节数*/
#if 0
	while((len=recv(client_sockfd,buf,BUFSIZ,0))>0)
	{
		/*printf("Enter string to send:");
		scanf("%s",buf);
		if(!strcmp(buf,"quit"))
			break;*/
		//len=send(client_sockfd,buf,strlen(buf),0);
		buf[len]='\0'; 
        printf("Server To Client:%s\n",buf); 
    }
#else
    while(1)
    {
        FD_ZERO(&readfd);
        FD_SET(client_sockfd, &readfd);
        FD_SET(uart_fd, &readfd);
        maxfd = client_sockfd > uart_fd ? client_sockfd + 1: uart_fd + 1;
        //only care readfd, in block mode
        switch(select(maxfd, &readfd, NULL, NULL, &timeout))
        {
            case -1:
                close(client_sockfd);
                close(uart_fd);
                printf("select return -1\n");
                exit(-1); 
                break;
            case 0:
                break;
            default:
                if(FD_ISSET(client_sockfd, &readfd))
                {
                    len = recv(client_sockfd, buf, BUFSIZ, 0);
		            //buf[len]='\0';
                    if(len > 0)
                    {
                        printf("Server message, length is %d\n",len); 
                        if(write(uart_fd, buf, len) == len)    //send data to UART
                            printf("write to uart sucessfully!\n");
                        else
                            printf("write to uart fail!\n");
                    }
                }
                else if(FD_ISSET(uart_fd, &readfd))
                {
                    while( (uart_tmp_len = read(uart_fd, read_buf + one_frame_len, sizeof(read_buf) - one_frame_len)) > 0 )
                    {
                        read_time++;
                        one_frame_len += uart_tmp_len; 
                    }
                    if(read_time)
                    {
                        printf("UART message, length is %d\n", one_frame_len); 
                        send_msg2_server(read_buf, one_frame_len);
                        read_time = 0;
                        one_frame_len = 0;
                    } 
                }
        }
    }
#endif
	close(client_sockfd);//关闭套接字
    return 0;
}
