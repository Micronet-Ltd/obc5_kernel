#include <linux/poll.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/sched.h>


MODULE_LICENSE("Dual BSD/GPL");

/////

#define MAX_MSG_EVENTS 512
#define MSG_PAYLOAD_SIZE 16

// TODO: make continous allocation
struct vaccel_msg {
	uint8_t data[MSG_PAYLOAD_SIZE];
};

// typical queue implementation
struct vaccel_msg_queue {
	struct mutex lock;
	wait_queue_head_t wq;
	unsigned int head;
	unsigned int tail;
	struct vaccel_msg msgs[MAX_MSG_EVENTS];
};

static inline int queue_empty(struct vaccel_msg_queue * queue)
{
	return queue->head == queue->tail;
}

static inline struct vaccel_msg * queue_get_msg(struct vaccel_msg_queue * queue)
{
	queue->tail = (queue->tail + 1) % MAX_MSG_EVENTS;
	return &queue->msgs[queue->tail];
}

static void queue_add_msg(struct vaccel_msg_queue * queue, struct vaccel_msg * msg)
{
	queue->head = (queue->head + 1) % MAX_MSG_EVENTS;
	if(queue->head == queue->tail)
		queue->tail = (queue->tail + 1) % MAX_MSG_EVENTS;

	//queue->msgs[queue->head] = msg; // ptr
	memcpy(&queue->msgs[queue->head], msg, sizeof(struct vaccel_msg));
}



struct virt_accel {
	struct vaccel_msg_queue queue;
};

struct virt_accel g_virt_accel_dev;

/////

static int vaccel_dev_open(struct inode *inode, struct file *file)
{
	struct virt_accel * dev;

	dev = &g_virt_accel_dev;

	/*if(test_and_set_bit(1, &dev->inuse))
		return -EPERM;*/

	//dev = container_of(inode->i_cdev, struct virt_accel, cdev);
	file->private_data = dev;


	return 0;
}

static int vaccel_dev_release(struct inode *inode, struct file *file)
{
	//struct virt_accel * dev = file->private_data;

	//clear_bit(1, &dev->inuse);

	return 0;
}

//TODO: test
static unsigned int vaccel_dev_poll(struct file * file,  poll_table * wait)
{
	struct virt_accel * dev = file->private_data;
	unsigned int mask = 0;

	mutex_lock(&dev->queue.lock);
	poll_wait(file, &dev->queue.wq, wait);
	if(!queue_empty(&dev->queue))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&dev->queue.lock);

	return mask;
}

static ssize_t virt_accel_chr_read(struct file * file, char __user * buf,
		size_t count, loff_t *ppos)
{
	struct virt_accel * dev = &g_virt_accel_dev;
	uint8_t data[MSG_PAYLOAD_SIZE];
	int read_count = 0;

	if(count < MSG_PAYLOAD_SIZE) {
		pr_err("%s() count < %d\n", __func__, MSG_PAYLOAD_SIZE);
		return -EINVAL;
	}

	while(count >= MSG_PAYLOAD_SIZE) {
		struct vaccel_msg * pmsg = NULL;

		mutex_lock(&dev->queue.lock);
		while(queue_empty(&dev->queue)) {
			mutex_unlock(&dev->queue.lock);
			if((file->f_flags &  O_NONBLOCK))
				return -EAGAIN;
			wait_event_interruptible(dev->queue.wq, !queue_empty(&dev->queue));
			mutex_lock(&dev->queue.lock);
		}

		if( (pmsg = queue_get_msg(&dev->queue)) )
			memcpy(&data, pmsg->data, sizeof(data));

		mutex_unlock(&dev->queue.lock);

		if(pmsg) {
			if(copy_to_user(buf + read_count, data, MSG_PAYLOAD_SIZE))
				return -EINVAL;

			count -= MSG_PAYLOAD_SIZE;
			read_count += MSG_PAYLOAD_SIZE;
		}
	}

	return read_count;
}

static ssize_t virt_accel_chr_write(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos)
{
	struct virt_accel * dev = file->private_data;
	struct vaccel_msg msg;
	ssize_t w = 0;

	if((count < MSG_PAYLOAD_SIZE) || (0 != (count % MSG_PAYLOAD_SIZE)))
		return -EINVAL;

	// This will never block
	while(count >= MSG_PAYLOAD_SIZE) {
		if(copy_from_user(msg.data, buf, MSG_PAYLOAD_SIZE))
			return -EACCES;

		mutex_lock(&dev->queue.lock);
		queue_add_msg(&dev->queue, &msg);
		mutex_unlock(&dev->queue.lock);

		wake_up_interruptible(&dev->queue.wq);
		count -= MSG_PAYLOAD_SIZE;
		w += MSG_PAYLOAD_SIZE;
		buf += MSG_PAYLOAD_SIZE;
	}

	return w;
}

/////



static const struct file_operations vaccel_dev_fops = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_accel_chr_read,
	.write = virt_accel_chr_write,
	.open = vaccel_dev_open,
	.release = vaccel_dev_release,
	.poll   = vaccel_dev_poll,
};



static struct miscdevice vaccel_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "vaccel",
	.fops		= &vaccel_dev_fops,
};

static int __init virtual_accel_init(void)
{
	struct virt_accel * dev = &g_virt_accel_dev;
	int ret;

	init_waitqueue_head(&dev->queue.wq);
	mutex_init(&dev->queue.lock);


	ret = misc_register(&vaccel_dev);
	if(ret) {
		pr_err("%s() Unable to register misc device \n", __func__);
		return ret;
	}

	return 0;
}

static void __exit virtual_accel_exit(void)
{

	misc_deregister(&vaccel_dev);
}


module_init(virtual_accel_init);
module_exit(virtual_accel_exit);
