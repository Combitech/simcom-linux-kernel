#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include <linux/ms6_adc.h>


#define NUM_ADCS	16


struct ms6_adc_dev {
	struct cdev c_dev;
	struct device *dev;
	int minor;
	int samplerate;
};


static struct ms6_adc_dev devices[NUM_ADCS];
static struct class *ms6_adc_class;
static int major;


int adc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ms6_adc_dev *dev = file->private_data;
	
	if (MAJOR (inode->i_rdev) != major)
		return -EINVAL;

	switch (cmd) {
	case MS6_ADC_SET_SAMPLERATE:
		dev->samplerate = (int)arg;
		break;

	case MS6_ADC_GET_SAMPLERATE:
		return dev->samplerate;

	default:
		return -ENOTTY;
	}

	return 0;
}


int adc_open(struct inode *inode, struct file *file)
{	
	int n = MINOR(file->f_dentry->d_inode->i_rdev);
	
	file->private_data = &devices[n];
	
	return 0;
}


int adc_release(struct inode *inode, struct file *file)
{
	unsigned int minor=MINOR(file->f_dentry->d_inode->i_rdev);

	return 0;
}


ssize_t adc_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	struct ms6_adc_dev *dev = file->private_data;
	char data[30];
	
	sprintf(data, "You read from adc %i\n", dev->minor);
	copy_to_user((void *)buf, (void *)data, strlen(data));
		
	return (ssize_t) strlen(data);
}


static struct file_operations adc_fops = {
	.owner = THIS_MODULE,
	.read = adc_read,			/* adc_read */
	.ioctl = adc_ioctl,			/* adc_ioctl*/
	.open = adc_open,			/* adc_open*/
	.release = adc_release,		/* adc_release*/
};


static int __init ms6_adc_init_module(void)
{
	dev_t dev_id;
	int res;
	int i;
	
	printk("Initializing ms6_adc driver\n");
	
	res = alloc_chrdev_region(&dev_id, 0, NUM_ADCS, "ms6_adc");
	major = MAJOR(dev_id);
	
	printk("Got major = %i\n", major);
	
	if(res < 0) {
		printk ("Unable to get major=%d for adc\n",
					major);
		return res;
	}
	
	printk("Creating class\n");
	ms6_adc_class = class_create(THIS_MODULE, "adc");
	
	if(!ms6_adc_class) {
		printk("Could not create class\n");
		return -ENOMEM;
	}
	
	for(i=0; i<16; i++) {
		cdev_init(&devices[i].c_dev, &adc_fops);
		
		if(cdev_add(&devices[i].c_dev,		//printk("Init device %i\n", i);
		 dev_id+i, 1) < 0 ) {
			printk("Could not add cdev %d", i);
		}
		
		devices[i].dev = device_create(ms6_adc_class, NULL, MKDEV(major, i), NULL, "adc%i", i);
		devices[i].minor = i;
	}
	
	printk("Done!\n");
	
	return 0;
}
module_init(ms6_adc_init_module);


static void __exit ms6_adc_cleanup_module (void)
{
	dev_t dev_id = MKDEV(major, 0);
	int i;
	
	printk("Unloading ms6_adc driver..");
	
	for(i=0; i<NUM_ADCS; i++) {
		device_del(devices[i].dev);
		cdev_del(&devices[i].c_dev);	
	}
	
	class_destroy(ms6_adc_class);
	unregister_chrdev_region(dev_id, NUM_ADCS);
	
	printk("Done!\n");
}
module_exit(ms6_adc_cleanup_module);


MODULE_AUTHOR("Mats-Olov Rustad");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MS6 FPGA A/D Converter");
