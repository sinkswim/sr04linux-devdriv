/*
 * 	Robert Margelli - S224854
 * 	sample.c
 * 	Driver for working with the HC-SR04 sensor
 *	as a proximity alert system.
 */

 /* includes */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <asm/uaccess.h>

/* defines */
#define TRUE 1
// #define SAMPLE_DEBUG
/* Definition of the GPIOs we want to use */
#define LED	129		// User led; PI1 -> 8*16 + 1
#define ECHO	130		// pin D8 -> echo; PI2 -> 8*16 + 2
#define TRIGGER 131		// pin D7 -> trigger; PI3 -> 8*16 + 3

/* Definition of ioctl commands */
#define	INVALID	-1
#define START_TRIGGER 	0
#define START_LED	1
#define WRITE_HPERIOD	2

/*
 * Device major number
 */
static uint module_major = 166;

/*
 * Device name
 */
static char * module_name = "sample";

/*
 * Device access lock. Only one process can access the driver at a time
 */
static int sample_lock = 0;

/* Device variables */
int mode = INVALID;	// device mode: INVALID = do nothing
			// START_TRIGGER = write trigger period and start the trigger wq
			// START_LED = write the blinking semi-period start the LED wq
			// WRITE_HPERIOD = write the blinking semi-period

static int blk_hperiod;		// stores LED blinking semi-period (ms)
static int trigger_hightime;	// trigger high time (ms)

static int echopin_value;	// used in echo ISR to sample pin D8 value
static ktime_t start, end;	// echo risedge/falledge timestamps
static s64 ts_diff;		// echo hightime (diff of start and end timestamps)
static int diff_ready=0;	// 1: new echo hightime is available, 0: no hightime available

/*
 * Declare the workqueue
 */
static struct workqueue_struct *t_wq;		// trigger wq
static struct workqueue_struct *led_wq;		// LED wq

typedef struct {		// LED work structure typedef
	struct work_struct l_work;
} led_work_t;

typedef struct {		// Trigger work structure typedef
	struct work_struct t_work;
	int	delay;
} my_work_t;

static my_work_t trigger_work;		// trigger work struct
static led_work_t led_work;		// LED work struct
static int stop_wq = 0;         // 0: the workers keep going

/* Trigger wq sets/unsets pin D7 (trigger) to drive the sensor */
static void trigger_wq_function( struct work_struct *work )
{
	int	delay;
	my_work_t *my_work;

	my_work = (my_work_t *)work;
	delay = my_work->delay;

	while(stop_wq == 0){
		gpio_set_value(TRIGGER, 1);
		msleep(delay);
		gpio_set_value(TRIGGER, 0);
		msleep(65);	// triggers must be interleaved with at least 60 ms intervals
	}
}

/* LED wq blinks LED with frequency based on hperiod global var */
static void led_wq_function( struct work_struct *led_work )
{
	while(stop_wq == 0)	// blink the LED
	{
			gpio_set_value( LED, 1 );
			msleep(blk_hperiod);
			gpio_set_value( LED, 0 );
			msleep(blk_hperiod);
	}
}

/* ECHO interrupt handler */
static irq_handler_t echo_handler( unsigned int irq, struct pt_regs *regs )
{
	echopin_value = gpio_get_value(ECHO);
	if(echopin_value != 0){ 	// rising edge -> get first timestamp
		start = ktime_get();
		diff_ready=0;
		}
	else if(echopin_value == 0){	// falling edge -> get second timestamp and compute the high time interval
		end = ktime_get();
		ts_diff = ktime_to_ns(ktime_sub(end,start));
		diff_ready=1;		// the next read operation will provide ts_diff to the app
		}

	return (irq_handler_t)IRQ_HANDLED;
}


/* Device read */
static ssize_t sample_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	int ret = 1;
	int zero = 0;

	if(diff_ready)	// echo high time is available, copy to user
	{
		memcpy(buffer, &ts_diff, sizeof(ts_diff));
		ret = sizeof(ts_diff);
		diff_ready=0;		// set diff_ready flag back to 0
	}
	else	// no echo hightime is available, copy a zero to user (app keeps busy waiting)
	{
		memcpy(buffer, &zero, sizeof(zero));
		ret = sizeof(zero);
	}


	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif


	return(ret);
}


/* Device write */
static ssize_t sample_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	int ret = 0;

	switch(mode)
	{
		case START_TRIGGER:
			memcpy( &trigger_hightime, buffer, sizeof(trigger_hightime) );	// copy trigger high time interval
			trigger_work.delay = trigger_hightime;	// write trigger high time in the work struct data
			queue_work( t_wq, (struct work_struct *)&trigger_work )	;// place a new trigger worker in the workqueue
			ret = sizeof(trigger_hightime);
			break;

		case START_LED:
			memcpy(&blk_hperiod, buffer, sizeof( blk_hperiod) );	// copy first (fake) blinking semi-period
			queue_work( led_wq, (struct work_struct *)&led_work );	// place a new LED worker in the workqueue
			ret = sizeof(blk_hperiod);
			break;

		case WRITE_HPERIOD:
			memcpy( &blk_hperiod, buffer, sizeof( blk_hperiod) );	// copy blinking semi-period to blk_hperiod global var
			ret = sizeof(blk_hperiod);
			break;

		case INVALID:	// invalid write, do nothing
			ret = 0;
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return(ret);
}


/* Device ioctl */
static ssize_t sample_ioctl(struct inode *inode, struct file *filep,
			    const unsigned int cmd, const unsigned long arg)
{
	int ret = 0;

	switch(cmd)
	{
		case START_TRIGGER:
			mode = START_TRIGGER;	// The next write will start the trigger wq
			break;

		case START_LED:
			mode = START_LED;	// The next write will start the LED wq
			break;

		case WRITE_HPERIOD:
			mode = WRITE_HPERIOD;	// The next write will overwrite the hperiod var
			break;

		default:
			mode = INVALID;		// Invalid ioctl, the next write will do nothing
			break;
	}

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return(ret);
}


/* Device open */
static int sample_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
	 * One process at a time
	 */
	if (sample_lock)
	{
		ret = -EBUSY;
	}
	else
	{
		sample_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG
			printk( KERN_INFO "%s: %s\n", module_name, __func__ );
		#endif
	}

	return(ret);
}

/* Device close */
static int sample_release(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
 	 * Release device,ready for our next caller
 	 */
	sample_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return(ret);
}


/*
 * Device operations
 */
static struct file_operations sample_fops = {
	.read = sample_read,
	.write = sample_write,
	.open = sample_open,
	.release = sample_release,
	.ioctl = sample_ioctl
};

/*
 * This function is called when the module is loaded
 */
static int __init sample_init_module(void)
{
	/*
 	 * Register device
 	 */
	int	ret;

	ret = register_chrdev(module_major, module_name, &sample_fops);
	if (ret < 0) {
		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
		       __func__, module_name, module_major, module_major );
		return(ret);
	}
	else
	{
		printk(KERN_INFO "%s: registering device %s with major %d\n",
		       __func__, module_name, module_major );
		/*
 		 * Reserve gpios
		*/
		if( gpio_request( TRIGGER, module_name ) )	// request pin D7 (trigger)
		{
			printk( KERN_INFO "%s: %s unable to get TRIG gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		if( gpio_request( ECHO, module_name ) )	// request pin D8 (echo)
		{
			printk( KERN_INFO "%s: %s unable to get ECHO gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		if( gpio_request( LED, module_name ) )	// request user LED GPIO
		{
			printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		/*
 		 * Set gpios directions
		*/
		if( gpio_direction_output( TRIGGER, 0 ) < 0 )	// Set pin D7 (trigger) as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set TRIG gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		if( gpio_direction_input( ECHO ) < 0 )	// Set pin D8 (echo) as input
		{
			printk( KERN_INFO "%s: %s unable to set ECHO gpio as input\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		if( gpio_direction_output( LED, 0 ) < 0 )	// Set LED gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return(ret);
		}

		// assign handler to ECHO pin IRQ
		if( request_irq( gpio_to_irq( ECHO ),		// IRQ of pin D8
                                 (irq_handler_t) echo_handler,	// ISR (callback)
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING , // sensitive to either edge
				  module_name,
				  NULL ) < 0 )
		{
			printk( KERN_INFO "%s: %s unable to register gpio irq\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		/* create the 2 needed workqueues */
		t_wq = create_workqueue( "Trigger wq" );
		if( t_wq )
		{
			INIT_WORK( (struct work_struct *)&trigger_work, trigger_wq_function );
		}

		led_wq = create_workqueue( "LED wq" );
		if( led_wq )
		{
			INIT_WORK( (struct work_struct *)&led_work, led_wq_function );
		}
	}

	return(ret);
}

/*
 * This function is called when the module is unloaded
 */
static void __exit sample_cleanup_module(void)
{
	/*
	 * Free echo irq
	 */
	free_irq( gpio_to_irq( ECHO ), NULL );

    /* destroy the workqueues */
    stop_wq = 1;                // stop the 2 running workers
    destroy_workqueue(t_wq);
    destroy_workqueue(led_wq);
    
	/*
	 * Release the gpios
	 */
	gpio_free(TRIGGER);
	gpio_free(ECHO);
	gpio_free(LED);

	/*
	 * Unregister device
	 */
	unregister_chrdev(module_major, module_name);

	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(sample_init_module);
module_exit(sample_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robert Margelli, robert.margelli@studenti.polito.it");
MODULE_DESCRIPTION("HC-SR04 Proximity Alert System");
