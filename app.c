/*
 *	Robert Margelli - S224854
 *	app .c
 * 	This is a user-space application that works with /dev/sample
 * 	prints the measured distance and computes the half period
 * 	for blinking the user LED.
 */

 /* includes */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <float.h>

/* defines*/
#define TRUE 1
/* ioctl commands */
#define INVALID 	-1
#define START_TRIGGER 	0
#define START_LED	1
#define WRITE_HPERIOD	2

/* function prototypes */
int get_hperiod(float distance);

/* main */
int main(int argc, char **argv)
{
	char *app_name = argv[0];
	char *dev_name = "/dev/sample";		// device file name
	int fd = -1;				// file descriptor of the above device file
	int x = 0;			// stores number of read/written bytes in reads and writes

	int trigger_hightime = 1;		// ms trigger should stay high
	unsigned long long echo_hightime;	// ns echo line stays high
	float distance=0;			// measured distance
	int ledwq_flag = 0;			// 0: LED work not started
	int blk_hperiod = 0;			// period/2 of LED blinking

	/* check correct app call */
	if(argc != 1)
	{
		printf("Wrong execution.  Usage: %s \n", argv[0]);
		return 1;
	}

	/*
 	 * Open the sample device RD | WR are allowed
 	 */
	if ((fd = open(dev_name, O_RDWR)) < 0) {
		fprintf(stderr, "%s: unable to open %s: %s\n",
			app_name, dev_name, strerror(errno));
		return( 1 );
	}

	ioctl(fd,START_TRIGGER,NULL);	// the next write will start the trigger WQ
	x = write(fd, &trigger_hightime, sizeof(trigger_hightime));	// start the trigger WQ

	while(TRUE)	// infinite loop for reading echo_hightime, printing distance and sending hperiod of blinking
	{
		do
		{
			x = read(fd, &echo_hightime, sizeof(echo_hightime));	// busy wait: read echo line high time intervale
		} while(echo_hightime == 0);

		distance = (float)(0.001*(echo_hightime)/58);	// convert ns to us and then get distance in cm
		printf("Measured distance: %f cm\n", distance);
		blk_hperiod = get_hperiod(distance);	// get blinking semi-period based on distance

		if(ledwq_flag == 0)	// first loop cycle so create the LED wq
		{
			ioctl(fd, START_LED, NULL);	// the next write will start the LED WQ
			x = write(fd, &blk_hperiod, sizeof(blk_hperiod)); // start LED WQ and pass the first blinking semi-period
			ledwq_flag = 1;
		}

		ioctl(fd, WRITE_HPERIOD, NULL); // the next write will write a new semi-period value
		x = write(fd, &blk_hperiod, sizeof(blk_hperiod)); // write a new semi-period value

	}

	if (fd >= 0)	// close fd if open
	{
		close(fd);
	}
	return( 0 );
}


/* returns half period (in ms) based on computed distance */
int get_hperiod(float distance)
{
	int hperiod = 0;
	if(distance < 10.0)
	{
		hperiod = 0;		// keep LED on (no blinking)
	}
	else if(distance >= 10.0 && distance < 25.0)
	{
		hperiod = 200;
	}
	else if(distance >= 25.0 && distance < 50.0)
	{
		hperiod = 300;
	}
	else if(distance >= 50.0 & distance < 75.0)
	{
		hperiod = 400;
	}
	else if(distance >= 75.0 & distance <= 100.0)
	{
		hperiod = 500;
	}
	else if(distance > 100.0)
	{
		hperiod = 1000;
	}
	return hperiod;
}
