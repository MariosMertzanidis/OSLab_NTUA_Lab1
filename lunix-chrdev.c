/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 *
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	debug("I need refresh\n");
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	/* The following return is bogus, just for the stub to compile */
	return state->buf_timestamp!=(sensor->msr_data[state->type])->last_update;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	int ret=0;
	long good_data=0;
	uint32_t data,time;
	struct lunix_sensor_struct *sensor;
	//debug("entering update\n");

	WARN_ON(!(sensor=state->sensor));

	if (!lunix_chrdev_state_needs_refresh(state)){
		ret = -EAGAIN;
		goto out;
	}

	spin_lock(&sensor->lock);
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	 data = sensor->msr_data[state->type]->values[0];
	 time = sensor->msr_data[state->type]->last_update;

	spin_unlock(&sensor->lock);

	state->buf_timestamp=time;
	switch(state->type){
		case 0:
			good_data=lookup_voltage[data];
			break;
		case 1:
			good_data=lookup_temperature[data];
			break;
		case 2:
			good_data=lookup_light[data];
			break;
		case 3:
			ret= -1;
			goto out;
			break;
	}

	if((good_data%1000)/10==0){
		if(good_data<0 && good_data/1000==0){
			sprintf(state->buf_data, "-%ld.00%ld\n", good_data/1000,abs(good_data%1000));
		}
		else{
			sprintf(state->buf_data, "%ld.00%ld\n", good_data/1000,abs(good_data%1000));
		}
	}
	else if((good_data%1000)/100==0){
		if(good_data<0 && good_data/1000==0){
			sprintf(state->buf_data, "-%ld.0%ld\n", good_data/1000,abs(good_data%1000));
		}
		else{
			sprintf(state->buf_data, "%ld.0%ld\n", good_data/1000,abs(good_data%1000));
		}
	}
	else if(good_data/1000==0){
		if(good_data<0){
			sprintf(state->buf_data, "-0.%ld\n", abs(good_data%1000));
		}
		else{
			sprintf(state->buf_data, "0.%ld\n", abs(good_data%1000));
		}
	}
	else{
		sprintf(state->buf_data, "%ld.%ld\n", good_data/1000,abs(good_data%1000));
	}



	state->buf_lim=strlen(state->buf_data);
	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */

out:
	debug("leaving\n");
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	int dev_minor= iminor(inode);
	int sensor_nr=dev_minor/8;
	int msr=dev_minor%8;
	int ret;
	struct lunix_chrdev_state_struct *dev;
	printk("Sensor %d, measurement %d has been opened\n",sensor_nr,msr);
	dev= kzalloc(sizeof(*dev),GFP_KERNEL);

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	 dev->buf_timestamp=0;
	 dev->type=msr;
	 dev->sensor=lunix_sensors+sensor_nr;
	 sema_init(&dev->lock,1);

	/* Allocate a new Lunix character device private state structure */
	filp->private_data=dev;
	/* ? */
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	int dev_minor= iminor(inode);
	int sensor_nr=dev_minor/8;
	int msr=dev_minor%8;
	kfree(filp->private_data);
	printk("Sensor %d, measurement %d has been realeased\n",sensor_nr,msr);
	/* ? */
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret=0;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;
  debug("I entered read\n");
	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */
	if (down_interruptible(&state->lock)){
		ret = -ERESTARTSYS;
		goto out;
	}
	if (*f_pos>LUNIX_CHRDEV_BUFSZ)
		goto out;
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	 debug("I read vol2\n");
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			debug("I read vol3\n");
			/* ? */
			up(&state->lock);
			/* The process needs to sleep */
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))) {
            ret = -ERESTARTSYS;
            goto out;
      }
			/* See LDD3, page 153 for a hint */
			if (down_interruptible(&state->lock)){
				ret = -ERESTARTSYS;
				goto out;
			}
		}
	}
	else{
		ret=0;
		goto out;
	}
	/* End of file */
	/* ? */
	ret = min((int) cnt,(int) (state->buf_lim - *f_pos));
	if (copy_to_user(usrbuf, state->buf_data + *f_pos, ret)){
		ret= -EFAULT;
		goto out;
	}
	*f_pos+=ret;
	if (*f_pos == state->buf_lim){
		*f_pos=0;
		goto out;
	}
	/* Determine the number of cached bytes to copy to userspace */
	/* ? */

	/* Auto-rewind on EOF mode? */
	/* ? */
out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
{
  .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	ret=register_chrdev_region(dev_no,lunix_minor_cnt,"Lunix:TNG");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	ret=cdev_add(&lunix_chrdev_cdev,dev_no,lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
