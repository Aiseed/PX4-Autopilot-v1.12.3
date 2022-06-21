#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <systemlib/mavlink_log.h>


#include <px4_defines.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
// #include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <px4_time.h>
/*
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <px4_time.h>
*/

static bool thread_should_exit = false;		/**< px4_uorb_subs exit flag */
static bool thread_running = false;		/**< px4_uorb_subs status flag */
static int rw_uart_task;				/**< Handle of px4_uorb_subs task / thread */

static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);


unsigned short CCrcGetCrc16(char data[], unsigned short len);
char* returnCommand(char can[]);
/**
 * daemon management function.
 */
__EXPORT int rw_uart_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int rw_uart_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: px4_uorb_adver {start|stop|status} [-p <additional params>]\n\n");
}

int set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	default:
		warnx("ERR: baudrate: %d\n", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* 以新的配置填充结构体 */
	/* 设置某个选项，那么就使用"|="运算，
	 * 如果关闭某个选项就使用"&="和"~"运算
	 * */
	tcgetattr(fd, &uart_config); // 获取终端参数

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

	/* 无偶校验，一个停止位 */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

	 /* 设置波特率 */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return false;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}
	// 设置与终端相关的参数，TCSANOW 立即改变参数
	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return false;
	}

	return true;
}


int uart_init(char * uart_name)
{
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
	/*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
	// 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
		printf("failed to open port: %s\n", uart_name);
		return false;
	}
	printf("Open the %i\n",serial_fd);
	return serial_fd;
}


/**
消息发布进程，会不断的接收自定义消息
 */
int rw_uart_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("px4_uorb_subs already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;//定义一个守护进程
		rw_uart_task = px4_task_spawn_cmd("rw_uart",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT,//调度优先级
			2000,//堆栈分配大小
			rw_uart_thread_main,
			(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		}
		else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int rw_uart_thread_main(int argc, char *argv[])
{
	char data = '0';
	//char out_buf[5]="endl";
	char buffer[5] = "";	
	/*
	 * TELEM1 : /dev/ttyS1
	 * TELEM2 : /dev/ttyS2
	 * GPS    : /dev/ttyS3
	 * NSH    : /dev/ttyS5
	 * SERIAL4: /dev/ttyS6
	 * N/A    : /dev/ttyS4
	 * IO DEBUG (RX only):/dev/ttyS0
	 */
	int uart_read = uart_init("/dev/ttyS");
	if (false == uart_read)
		return -1;
	if (false == set_uart_baudrate(uart_read, 9600)) {
		printf("[JXF]set_uart_baudrate is failed\n");
		return -1;
	}
	printf("[JXF]uart init is successful\n");
	warnx("[rw_uart] starting\n");
	
	thread_running = true;
	//write(uart_read, &out_buf[0],4);
	




    

   	const short Table_CRC16[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };    
	
    

	while (!thread_should_exit) {
		read(uart_read, &data, 1);
		if (data == 'R') {
			for (int i = 0; i < 4; ++i) {
				read(uart_read, &data, 1);
				buffer[i] = data;
				write(uart_read, &buffer[i], sizeof(buffer[i]));
				data = '0';
				usleep(5000);//5ms pause
			}
			printf("%s\n", buffer);
		}
		warnx("[rw_uart] starting\n");
		
	    printf("rw_uart TX-test:running!\n");
		
		usleep(1000000);
	}
	//write(uart_read, &out_buf[0],4);
	warnx("[rw_uart] exiting.\n");
	thread_running = false;
	int fd=close(uart_read);
	printf("close stauts: %d\n",fd);
	return 0;
}
// unsigned short == uint16
unsigned short CCrcGetCrc16(char data[], unsigned short len, short Table_CRC16[]) {
    unsigned short wResult = 0;
    unsigned short wTableNo = 0;
    int i = 0;

    for(i = 0; i < len; i++) {
            wTableNo = ((wResult & 0xff) ^ (data[i] & 0xff));
            wResult = ((wResult >> 8) & 0xff) ^ Table_CRC16[wTableNo];
    }

    return wResult;
}


char* returnCommand(char can[], char CRC[]){
    char command[36]={0xEB, 0x90, 0x23, 0x08, can[0], can[1], can[2], can[3], can[4], can[5], can[6], can[7],
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CRC[0], CRC[1]};
    return command;
}

