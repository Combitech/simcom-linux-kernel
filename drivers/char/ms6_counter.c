#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include <linux/ms6_counter.h>


#define NUM_COUNTERS	8


static int major;

struct ms6_counter_dev {
	struct cdev c_dev;
	struct device *dev;
	int minor;
	int function;
};

static struct ms6_counter_dev counter_dev[NUM_COUNTERS];
static struct class *ms6_counter_class;

int counter_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ms6_counter_dev *dev = file->private_data;
	
	if (MAJOR (inode->i_rdev) != major)
		return -EINVAL;

	switch (cmd) {
	case MS6_COUNTER_SET_FUNC:
		dev->function = (int)arg;
		break;

	case MS6_COUNTER_GET_FUNC:
		return dev->function;

	default:
		return -ENOTTY;
	}

	return 0;
}


int counter_open(struct inode *inode, struct file *file)
{	
	int n = MINOR(file->f_dentry->d_inode->i_rdev);
	
	file->private_data = &counter_dev[n];
	
	return 0;
}


int counter_release(struct inode *inode, struct file *file)
{
	unsigned int minor=MINOR(file->f_dentry->d_inode->i_rdev);

	return 0;
}


ssize_t counter_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct ms6_counter_dev *dev = file->private_data;
	char data[30];
	
	sprintf(data, "You read from counter%d\n", dev->minor);
	copy_to_user((void *)buf, (void *)data, strlen(data));
		
	return (ssize_t) strlen(data);
}



static struct file_operations counter_fops = {
	.owner = THIS_MODULE,
	.read = counter_read,		/* counter_read */
	.ioctl = counter_ioctl,		/* counter_ioctl*/
	.open = counter_open,		/* counter_open*/
	.release = counter_release,	/* counter_release*/
};


static int __init ms6_counter_init_module(void)
{
	dev_t dev_id;
	int ret;
	int i;
	
	printk("Initializing ms6_counter driver...");
	
	/* Request ADdynamic allocation of a device major number */
	ret = alloc_chrdev_region(&dev_id, 0, NUM_COUNTERS, "ms6_counter");
	if(ret < 0) {
		printk("Can't register device counter\n");
		
		return ret;
	}
	major = MAJOR(dev_id);
	printk("Got major = %i\n", major);
	
	/* Populate sysfs entries */
	ms6_counter_class = class_create(THIS_MODULE, "counter");
	
	if(ms6_counter_class == NULL) {
		printk("Could not create class counter\n");

		return -ENOMEM;
	}
	
	for(i=0; i<NUM_COUNTERS; i++) {
		/* Connect the file operations with the cdev */
		cdev_init(&counter_dev[i].c_dev, &counter_fops);
		counter_dev[i].c_dev.owner = THIS_MODULE;
		
		/* Connect the major/minor number top the cdev */
		ret = cdev_add(&counter_dev[i].c_dev, dev_id+i, 1); 
		if(ret < 0 ) {
			printk("Could not add cdev %d", i);

			return ret;
		}
		
		/* Send uevents to udev, so it'll create /dev nodes */
		counter_dev[i].dev = device_create(ms6_counter_class, NULL, MKDEV(major, i), NULL, "counter%d", i);
		counter_dev[i].minor = i;
	}
	
	printk("Done!\n");
	
	return 0;
}
module_init(ms6_counter_init_module);


static void __exit ms6_counter_cleanup_module (void)
{
	dev_t dev_id = MKDEV(major, 0);
	int ret;
	int i;
	
	printk("Cleaning up ms6_counter driver..");
	
	for(i=0; i<NUM_COUNTERS; i++) {
		device_del(counter_dev[i].dev);
		cdev_del(&counter_dev[i].c_dev);	
	}
	
	/* Destroy counter class */
	class_destroy(ms6_counter_class);

	/* Release the major number */
	unregister_chrdev_region(dev_id, NUM_COUNTERS);
	
	printk("Done!\n");
}
module_exit(ms6_counter_cleanup_module);


MODULE_AUTHOR("Mats-Olov Rustad");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MS6 FPGA counter");
