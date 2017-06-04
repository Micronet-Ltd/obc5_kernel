/**
 * @file   one_wire.c
 * @author Abid Esmail
 * @date   March 29 2017
 * @version 1.0
 * @brief  This one-wire driver expects iodriver to write 8 bytes of one-wire data
 * It formats this data into hex ASCII and adds a \r delimiter after the data rx.
 * Typically 9 bytes will be read back on a read operation on /dev/one_wire
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/mutex.h>
#define  DEVICE_NAME "one_wire"
#define  CLASS_NAME  "one_wire_chr"
#define MAX_BUF_SIZE 256

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Abid Esmail");
MODULE_DESCRIPTION("A one wire char driver");
MODULE_VERSION("1.0");

static int    majorNumber;
static char   message[MAX_BUF_SIZE] = {0};
static char * pmessage = message;
static short  size_of_message = 0;
static int    numberOpens = 0;
static struct class*  oneWireClass  = NULL;
static struct device* oneWireDevice = NULL;

static DEFINE_MUTEX(ebbchar_mutex);

static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
   .owner = THIS_MODULE,
};

static int __init one_wire_init(void){
   printk(KERN_INFO "one_wire: Initializing the one_wire driver LKM\n");

   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   if (majorNumber<0){
      printk(KERN_ALERT "one_wire failed to register a major number\n");
      return majorNumber;
   }
   printk(KERN_INFO "one_wire: registered correctly with major number %d\n", majorNumber);

   // Register the device class
   oneWireClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(oneWireClass)){
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(oneWireClass);
   }
   printk(KERN_INFO "one_wire: device class registered correctly\n");

   // Register the device driver
   oneWireDevice = device_create(oneWireClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(oneWireDevice)){
      class_destroy(oneWireClass);
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(oneWireDevice);
   }
   printk(KERN_INFO "one_wire: device class created correctly\n");
   mutex_init(&ebbchar_mutex);
   return 0;
}

static void __exit one_wire_exit(void){
   mutex_destroy(&ebbchar_mutex);
   device_destroy(oneWireClass, MKDEV(majorNumber, 0));
   class_unregister(oneWireClass);
   class_destroy(oneWireClass);
   unregister_chrdev(majorNumber, DEVICE_NAME);
   printk(KERN_INFO "one_wire: module exited!\n");
}

static int dev_open(struct inode *inodep, struct file *filep){

   //if(!mutex_trylock(&ebbchar_mutex)){
   // printk(KERN_ALERT "one_wire: Device in use by another process");
   // return -EBUSY;
   //}
   numberOpens++;
   //printk(KERN_INFO "one_wire: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}

/** @brief: This function is called when the device is read from userspace. The data
 *  is expected to be in chunks of 9 bytes.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
   int error_count = 0;
   short bytes_sent = 0;
   error_count = copy_to_user(buffer, message, size_of_message);

   if (error_count==0){
	   //printk(KERN_INFO "one_wire: Sent %d characters to the user\n", size_of_message);
	   bytes_sent = size_of_message;
	   size_of_message = 0;
	   return bytes_sent;
   }
   else {
	   printk(KERN_INFO "one_wire: Failed to send %d characters to the user\n", error_count);
	   return -EFAULT;
   }
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is converted to hex ASCII, a \r delimeter 
 *  is added and then it is copied to the message[] array. We expect 8 bytes to be written via
 *  iodriver
 *  
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
   short bytes_rx = len;
   u8 val = 0;
   if ((size_of_message + len) > MAX_BUF_SIZE){
	   printk(KERN_ALERT "one_wire: max size of buffer (%d) reached, write failed\n", (int)(size_of_message + len));
	   return -EFAULT;
   }
   while (bytes_rx--){
	   	val = (u8)*buffer;
	   	/* convert data to hex ASCII */
   		sprintf(pmessage + size_of_message, "%02x", val);
		buffer++;
		size_of_message += 2;
   }
   sprintf(pmessage + size_of_message, "\r");   // add \r char to delineate data
   size_of_message++;
   //printk(KERN_INFO "one_wire: Received %zu characters from the user, wrote %d to buf\n", len, size_of_message);
   return len;
}

static int dev_release(struct inode *inodep, struct file *filep){
   //mutex_unlock(&ebbchar_mutex);                      // release the mutex (i.e., lock goes up)
   //printk(KERN_INFO "one_wire: Device successfully closed\n");
   return 0;
}

module_init(one_wire_init);
module_exit(one_wire_exit);
