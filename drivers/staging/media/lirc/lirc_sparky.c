/*
 * lirc_sparky.c
 *
 * lirc_sparky - Device driver that records pulse- and pause-lengths
 *	      (space-lengths) (just like the lirc_serial driver does)
 *	      between GPIO interrupt events on the sparky.
 *	      Lots of code has been taken from the lirc_serial and licr_rpi modules,
 *	      so I would like say thanks to the authors.
 *
 * Copyright (C) 2016 Sudeep <sudeepalloblr@gmail.com>,
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <mach/gpio.h>
#include <mach/hardware.h>

#define LIRC_DRIVER_NAME "lirc_sparky"
#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 50

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

#define dprintk(fmt, args...)					\
	do {							\
		if (debug)					\
			printk(KERN_DEBUG LIRC_DRIVER_NAME ": "	\
			       fmt, ## args);			\
	} while (0)

/* module parameters */

/* set the default GPIO input pin */
static int gpio_in_pin = 47;  /* sparky Bports using for this, GPIOB15 is 47*/
/* set the default GPIO output pin */
static int gpio_out_pin = 48;

/* enable debugging messages */
static bool debug;
/* -1 = auto, 0 = active high, 1 = active low */
static int sense = -1;
/* use softcarrier by default */
static bool softcarrier = 1;
/* 0 = do not invert output, 1 = invert output */
static bool invert = 0;

struct gpio_chip *gpiochip;
struct irq_chip *irqchip;
struct irq_data *irqdata;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_sparky_exit(void);
/*GPIOB4-36 (32+4), B12-44 , B13-45 , B14-46,B15-47,B16-48,B19-51,B30-62 */
int valid_gpio_pins[] = {36, 44, 45, 46, 47, 48, 51, 62,};

static struct platform_device *lirc_sparky_dev;
static struct timeval lasttv = { 0, 0 };
static struct lirc_buffer rbuf;
static spinlock_t lock;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static int init_timing_params(unsigned int new_duty_cycle,
	unsigned int new_freq)
{
	if (1000 * 1000000L / new_freq * new_duty_cycle / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	if (1000 * 1000000L / new_freq * (100 - new_duty_cycle) / 100 <=
	    LIRC_TRANSMITTER_LATENCY)
		return -EINVAL;
	duty_cycle = new_duty_cycle;
	freq = new_freq;
	period = 1000 * 1000000L / freq;
	pulse_width = period * duty_cycle / 100;
	space_width = period - pulse_width;
	dprintk("in init_timing_params, freq=%d pulse=%ld, "
		"space=%ld\n", freq, pulse_width, space_width);
	return 0;
}

static long send_pulse_softcarrier(unsigned long length)
{
	int flag;
	unsigned long actual, target;
	unsigned long actual_us, initial_us, target_us;

	length *= 1000;

	actual = 0; target = 0; flag = 0;
	read_current_timer(&actual_us);

	while (actual < length) {
		if (flag) {
			gpiochip->set(gpiochip, gpio_out_pin, invert);
			target += space_width;
		} else {
			gpiochip->set(gpiochip, gpio_out_pin, !invert);
			target += pulse_width;
		}
		initial_us = actual_us;
		target_us = actual_us + (target - actual) / 1000;
		/*
		 * Note - we've checked in ioctl that the pulse/space
		 * widths are big enough so that d is > 0
		 */
		if  ((int)(target_us - actual_us) > 0)
			udelay(target_us - actual_us);
		read_current_timer(&actual_us);
		actual += (actual_us - initial_us) * 1000;
		flag = !flag;
	}
	return (actual-length) / 1000;
}

static long send_pulse(unsigned long length)
{
	if (length <= 0)
		return 0;

	if (softcarrier) {
		return send_pulse_softcarrier(length);
	} else {
		gpiochip->set(gpiochip, gpio_out_pin, !invert);
		safe_udelay(length);
		return 0;
	}
}

static void send_space(long length)
{
	gpiochip->set(gpiochip, gpio_out_pin, invert);
	if (length <= 0)
		return;
	safe_udelay(length);
}

static void rbwrite(int l)
{
	if (lirc_buffer_full(&rbuf)) {
		/* no new signals will be accepted */
		dprintk("Buffer overrun\n");
		return;
	}
	lirc_buffer_write(&rbuf, (void *)&l);
}

static void frbwrite(int l)
{
	/* simple noise filter */
	static int pulse, space;
	static unsigned int ptr;

	if (ptr > 0 && (l & PULSE_BIT)) {
		pulse += l & PULSE_MASK;
		if (pulse > 250) {
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
		return;
	}
	if (!(l & PULSE_BIT)) {
		if (ptr == 0) {
			if (l > 20000) {
				space = l;
				ptr++;
				return;
			}
		} else {
			if (l > 20000) {
				space += pulse;
				if (space > PULSE_MASK)
					space = PULSE_MASK;
				space += l;
				if (space > PULSE_MASK)
					space = PULSE_MASK;
				pulse = 0;
				return;
			}
			rbwrite(space);
			rbwrite(pulse | PULSE_BIT);
			ptr = 0;
			pulse = 0;
		}
	}
	rbwrite(l);
}
/* sparky change*/
static int sparky_irq_mode(int gpio_n , int mode)
{
        int a,b,i=0;
        int val,m_val;
        int base=INTC_GPIOA_TYPE0;
         if(gpio_n>31)
        {
                 i=gpio_n/32;
                 a=gpio_n%32; 
        }
        else
        {
                i=0;
                a=gpio_n;  /* valu should be 0 to 15*/
        }
        if(!(a/16))
        {
                i=(i*2)+1;
        }
        else
        {
                i=i*2;
		a=a%16; /* valu should be 0 to 15*/
        }
        base=base+i*4;
        val=act_readl(base);
       	dprintk("current IRQ reg value is %x pin pos is %d pair, mode is %d  \n",val,a,mode);
        b=(mode<<a*2);
        m_val=((val&~(3<<a*2))|b); /*clearing specific bits*/
        act_writel(m_val,base);
        dprintk("current IRQ reg m_value is %x I val is %d \n",m_val,i);
        return 0;
}



static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs)
{
	struct timeval tv;
	long deltv;
	int data;
	int signal;
	/* use the GPIO signal level */
	signal = gpiochip->get(gpiochip, gpio_in_pin);
	if(signal) 
	{
		sparky_irq_mode(gpio_in_pin,3);  // EDGE_FALLING
	}
	else if (signal==0)
	{
        	sparky_irq_mode(gpio_in_pin,2);  //EDGE_RISING      
	}	

	if (sense != -1) {
		/* get current time */
		do_gettimeofday(&tv);

		/* calc time since last interrupt in microseconds */
		deltv = tv.tv_sec-lasttv.tv_sec;
		if (tv.tv_sec < lasttv.tv_sec ||
		    (tv.tv_sec == lasttv.tv_sec &&
		     tv.tv_usec < lasttv.tv_usec)) {
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": AIEEEE: your clock just jumped backwards\n");
			printk(KERN_WARNING LIRC_DRIVER_NAME
			       ": %d %d %lx %lx %lx %lx\n", signal, sense,
			       tv.tv_sec, lasttv.tv_sec,
			       tv.tv_usec, lasttv.tv_usec);
			data = PULSE_MASK;
		} else if (deltv > 30) {
			data = PULSE_MASK; /* really long time */
			if (!(signal^sense)) {
				/* sanity check */
				printk(KERN_WARNING LIRC_DRIVER_NAME
				       ": AIEEEE: %d %d %lx %lx %lx %lx\n",
				       signal, sense, tv.tv_sec, lasttv.tv_sec,
				       tv.tv_usec, lasttv.tv_usec);
				/*
				 * detecting pulse while this
				 * MUST be a space!
				 */
				sense = sense ? 0 : 1;
			}
		} else {
			data = (int) (deltv*1000000 +
				      (tv.tv_usec - lasttv.tv_usec));
		}
//		frbwrite(signal^sense ? data : (data|PULSE_BIT));
		frbwrite(signal^sense ?(data|PULSE_BIT):data);

		lasttv = tv;
		wake_up_interruptible(&rbuf.wait_poll);
		if(signal)
		{
			sense=0;
		}
		else
		{
		        sense=1;
		}
	}
	return IRQ_HANDLED;
}

static int is_right_chip(struct gpio_chip *chip, void *data)
{
	dprintk("is_right_chip %s %d\n", chip->label, strcmp(data, chip->label));

	if (strcmp(data, chip->label) == 0)
		return 1;
	return 0;
}

static int init_port(void)
{
	int i, nlow, nhigh, ret, irq;
	gpiochip = gpiochip_find("owl-gpio-chip", is_right_chip);

	if (!gpiochip)
		return -ENODEV;

	if (gpio_request(gpio_out_pin, LIRC_DRIVER_NAME " ir/out")) {
		printk(KERN_ALERT LIRC_DRIVER_NAME
		       ": cant claim gpio pin %d\n", gpio_out_pin);
		ret = -ENODEV;
		goto exit_init_port;
	}

	if (gpio_request(gpio_in_pin, LIRC_DRIVER_NAME " ir/in")) {
		printk(KERN_ALERT LIRC_DRIVER_NAME
		       ": cant claim gpio pin %d\n", gpio_in_pin);
		ret = -ENODEV;
		goto exit_gpio_free_out_pin;
	}

	gpiochip->direction_input(gpiochip, gpio_in_pin);
	gpiochip->direction_output(gpiochip, gpio_out_pin, 1);
	gpiochip->set(gpiochip, gpio_out_pin, invert);

	irq = gpiochip->to_irq(gpiochip, gpio_in_pin);
	printk("to_irq %d\n", irq);
	irqdata = irq_get_irq_data(irq);

	if (irqdata && irqdata->chip) {
		irqchip = irqdata->chip;
	} else {
		ret = -ENODEV;
		goto exit_gpio_free_in_pin;
	}

	/* if pin is high, then this must be an active low receiver. */
	if (sense == -1) {
		/* wait 1/2 sec for the power supply */
		msleep(500);

		/*
		 * probe 9 times every 0.04s, collect "votes" for
		 * active high/low
		 */
		nlow = 0;
		nhigh = 0;
		for (i = 0; i < 9; i++) {
			if (gpiochip->get(gpiochip, gpio_in_pin))
				nlow++;
			else
				nhigh++;
			msleep(40);
		}
		sense = (nlow >= nhigh ? 1 : 0);
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": auto-detected active %s receiver on GPIO pin %d\n",
		       sense ? "low" : "high", gpio_in_pin);
	} else {
		printk(KERN_INFO LIRC_DRIVER_NAME
		       ": manually using active %s receiver on GPIO pin %d\n",
		       sense ? "low" : "high", gpio_in_pin);
	}

	return 0;

	exit_gpio_free_in_pin:
	gpio_free(gpio_in_pin);

	exit_gpio_free_out_pin:
	gpio_free(gpio_out_pin);

	exit_init_port:
	return ret;
}

// called when the character device is opened
static int set_use_inc(void *data)
{
	int result,irq;

	/* initialize timestamp */
	do_gettimeofday(&lasttv);
	irq = gpio_to_irq(gpio_in_pin);
	
	result = request_irq(irq, (irq_handler_t) irq_handler, IRQF_TRIGGER_FALLING| IRQF_DISABLED , LIRC_DRIVER_NAME, NULL);
//	result = request_irq(irq, (irq_handler_t) irq_handler, IRQF_TRIGGER_RISING | IRQF_DISABLED , LIRC_DRIVER_NAME, NULL);
	switch (result) {
	case -EBUSY:
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": IRQ %d is busy\n",
		       gpiochip->to_irq(gpiochip, gpio_in_pin));
		return -EBUSY;
	case -EINVAL:
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": Bad irq number or handler\n");
		return -EINVAL;
	default:
		printk("Interrupt %d obtained\n",
			gpiochip->to_irq(gpiochip, gpio_in_pin));
		break;
	};
	/* initialize pulse/space widths */
	init_timing_params(duty_cycle, freq);

	return 0;
}

static void set_use_dec(void *data)
{

	free_irq(gpiochip->to_irq(gpiochip, gpio_in_pin), (void *) 0);

	dprintk(KERN_INFO LIRC_DRIVER_NAME
		": freed IRQ %d\n", gpiochip->to_irq(gpiochip, gpio_in_pin));
}

static ssize_t lirc_write(struct file *file, const char *buf,
	size_t n, loff_t *ppos)
{
	int i, count;
	unsigned long flags;
	long delta = 0;
	int *wbuf;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;
	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf))
		return PTR_ERR(wbuf);
	spin_lock_irqsave(&lock, flags);

	for (i = 0; i < count; i++) {
		if (i%2)
			send_space(wbuf[i] - delta);
		else
			delta = send_pulse(wbuf[i]);
	}
	gpiochip->set(gpiochip, gpio_out_pin, invert);

//	spin_unlock_irqrestore(&lock, flags);
	kfree(wbuf);
	return n;
}

static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result;
	__u32 value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;
		break;

	case LIRC_SET_SEND_MODE:
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;

	case LIRC_GET_LENGTH:
		return -ENOSYS;
		break;

	case LIRC_SET_SEND_DUTY_CYCLE:
		printk("SET_SEND_DUTY_CYCLE\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value <= 0 || value > 100)
			return -EINVAL;
		return init_timing_params(value, freq);
		break;

	case LIRC_SET_SEND_CARRIER:
		printk("SET_SEND_CARRIER\n");
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		if (value > 500000 || value < 20000)
			return -EINVAL;
		return init_timing_params(duty_cycle, value);
		break;

	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_write,
	.unlocked_ioctl	= lirc_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

static struct lirc_driver driver = {
	.name		= LIRC_DRIVER_NAME,
	.minor		= -1,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= &rbuf,
	.set_use_inc	= set_use_inc,
	.set_use_dec	= set_use_dec,
	.fops		= &lirc_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static struct platform_driver lirc_sparky_driver = {
	.driver = {
		.name   = LIRC_DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init lirc_sparky_init(void)
{
	int result;

	/* Init read buffer. */
	result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if (result < 0)
		return -ENOMEM;

	result = platform_driver_register(&lirc_sparky_driver);
	if (result) {
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": lirc register returned %d\n", result);
		goto exit_buffer_free;
	}

	lirc_sparky_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
	if (!lirc_sparky_dev) {
		result = -ENOMEM;
		goto exit_driver_unregister;
	}

	result = platform_device_add(lirc_sparky_dev);
	if (result)
		goto exit_device_put;

	return 0;

	exit_device_put:
	platform_device_put(lirc_sparky_dev);

	exit_driver_unregister:
	platform_driver_unregister(&lirc_sparky_driver);

	exit_buffer_free:
	lirc_buffer_free(&rbuf);

	return result;
}

static void lirc_sparky_exit(void)
{
	platform_device_unregister(lirc_sparky_dev);
	platform_driver_unregister(&lirc_sparky_driver);
	lirc_buffer_free(&rbuf);
}

static int __init lirc_sparky_init_module(void)
{
	int result, i,val;

	result = lirc_sparky_init();
	if (result)
		return result;

	/* check if the module received valid gpio pin numbers */
	result = 0;
	if((gpio_in_pin > 41) && (gpio_in_pin < 62))
	{
//		printk("Test , Its in Bport %d\n ",gpio_in_pin);	
		val = act_readl(MFP_CTL1);
        	val=(val| (1<<22));    // bit 22 for B19,B18,B17,B16,B15,B14,B13,B12,B11,B10 - 10 Pins as GPIO
        	val=(val| (1<<6));    // bit 6 for B20
        	val=(val| (1<<5));    // bit 5 for B21
        	act_writel(val,MFP_CTL1);

         	val = act_readl(MFP_CTL2);
        	val=(val| (1<<28));    // bit 28 for B22,B23,B24,B25,B26,B27,B28,B29 - 8 pins as GPIO
        	act_writel(val,MFP_CTL2);
	}	
//	printk("Test1 , GPIO num = %d\n ",gpio_in_pin);
	if (gpio_in_pin != gpio_out_pin) {
		for(i = 0; (i < ARRAY_SIZE(valid_gpio_pins)) && (result != 2); i++) {
			if (gpio_in_pin == valid_gpio_pins[i] ||
			   gpio_out_pin == valid_gpio_pins[i]) {
				result++;
			}
		}
	}

	if (result != 2) {
		result = -EINVAL;
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": invalid GPIO pin(s) specified!\n");
		goto exit_sparky;
	}

	result = init_port();
	if (result < 0)
		goto exit_sparky;

	driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
			  LIRC_CAN_SET_SEND_CARRIER |
			  LIRC_CAN_SEND_PULSE |
			  LIRC_CAN_REC_MODE2;

	driver.dev = &lirc_sparky_dev->dev;
	driver.minor = lirc_register_driver(&driver);

	if (driver.minor < 0) {
		printk(KERN_ERR LIRC_DRIVER_NAME
		       ": device registration failed with %d\n", result);
		result = -EIO;
		goto exit_sparky;
	}

	printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");

	return 0;

	exit_sparky:
	lirc_sparky_exit();

	return result;
}

static void __exit lirc_sparky_exit_module(void)
{
	gpio_free(gpio_out_pin);
	gpio_free(gpio_in_pin);

	lirc_sparky_exit();

	lirc_unregister_driver(driver.minor);
	printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_sparky_init_module);
module_exit(lirc_sparky_exit_module);

MODULE_DESCRIPTION("Infra-red receiver driver for Sparky SBC GPIO.");
MODULE_AUTHOR("Sudeep <sudeepalloblr@gmail.com>");
MODULE_LICENSE("GPL");


module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin number of the Sparky SBC"
		 "GPIO PORTB Valid pin numbers are: 36, 44, 45, 46, 47, 48, 51, 62"
			"  GPIOB4 is 36(32+4),B12-44,B13-45,B14-46,B15-47,B16-48,B19-51,B30-62");

module_param(gpio_in_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_in_pin, "GPIO input pin number of the Sparky SBC."
		 "GPIO PORTB Valid pin numbers are: 36, 44, 45, 46, 47, 48, 51, 62"
		"  GPIOB4 is 36(32+4),B12-44,B13-45,B14-46,B15-47,B16-48,B19-51,B30-62");

module_param(sense, int,0);
MODULE_PARM_DESC(sense, "Override autodetection of IR receiver circuit"
		 " (0 = active high, 1 = active low )");

module_param(softcarrier, bool, S_IRUGO);
MODULE_PARM_DESC(softcarrier, "Software carrier (0 = off, 1 = on, default on)");

module_param(invert, bool, S_IRUGO);
MODULE_PARM_DESC(invert, "Invert output (0 = off, 1 = on, default off");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
