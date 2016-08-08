/*
 * MCU J1708 driver
 *
 * Copyright 2016 Micronet Inc., ruslan.sirota@micronet-inc.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

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

#define J1708_MAX_MSG_EVENTS 128
#define J1708_MSG_PAYLOAD_SIZE 21
//#define J1708_DEBUG

// TODO: make continous allocation
struct j1708_msg {
    uint8_t len;
	uint8_t data[J1708_MSG_PAYLOAD_SIZE];
};

// typical queue implementation
struct j1708_msg_queue {
	struct mutex lock;
	wait_queue_head_t wq;
	unsigned int head;
	unsigned int tail;
	struct j1708_msg msgs[J1708_MAX_MSG_EVENTS];
};

static inline int queue_empty(struct j1708_msg_queue * queue)
{
	return queue->head == queue->tail;
}

static inline struct j1708_msg * queue_get_msg(struct j1708_msg_queue * queue)
{
	queue->tail = (queue->tail + 1) % J1708_MAX_MSG_EVENTS;
	return &(queue->msgs[queue->tail]);
}

static void queue_add_msg(struct j1708_msg_queue * queue, struct j1708_msg * msg)
{
	queue->head = (queue->head + 1) % J1708_MAX_MSG_EVENTS;
	if(queue->head == queue->tail)
		queue->tail = (queue->tail + 1) % J1708_MAX_MSG_EVENTS;

	//queue->msgs[queue->head] = msg; // ptr
	memcpy(&queue->msgs[queue->head], msg, sizeof(struct j1708_msg));
}

struct virt_j1708 {
	struct j1708_msg_queue queue_in;
    struct j1708_msg_queue queue_out;
};

struct virt_j1708 g_virt_j1708_mcu_dev;

/////

static int j1708_dev_open(struct inode *inode, struct file *file)
{
	struct virt_j1708 * dev;

	dev = &g_virt_j1708_mcu_dev;

	file->private_data = dev;


	return 0;
}

static int j1708_dev_release(struct inode *inode, struct file *file)
{
	return 0;
}


static unsigned int j1708_dev_poll_in(struct file * file,  poll_table * wait)
{
	struct virt_j1708 * dev = file->private_data;
	unsigned int mask = 0;

	mutex_lock(&dev->queue_out.lock);
	poll_wait(file, &dev->queue_out.wq, wait);
	if(!queue_empty(&dev->queue_out))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&dev->queue_out.lock);

	return mask;
}

static ssize_t virt_j1708_chr_read_in(struct file * file, char __user * buf,
		size_t count, loff_t *ppos)
{
	struct virt_j1708 * dev = file->private_data;//&g_virt_j1708_mcu_dev;
    struct j1708_msg msg = {0};
	int read_count = 0;

	if(count < J1708_MSG_PAYLOAD_SIZE) {
		pr_err("%s() count < %d\n", __func__, J1708_MSG_PAYLOAD_SIZE);
		return -EINVAL;
	}

	if (count >= J1708_MSG_PAYLOAD_SIZE) {
		struct j1708_msg * pmsg = NULL;

		mutex_lock(&dev->queue_out.lock);
		while(queue_empty(&dev->queue_out)) {
			mutex_unlock(&dev->queue_out.lock);
			if((file->f_flags &  O_NONBLOCK))
				return -EAGAIN;
			wait_event_interruptible(dev->queue_out.wq, !queue_empty(&dev->queue_out));
			mutex_lock(&dev->queue_out.lock);
		}

		if( (pmsg = queue_get_msg(&dev->queue_out)) )
			memcpy(&msg, pmsg, sizeof(msg));

		mutex_unlock(&dev->queue_out.lock);

		if(pmsg && msg.len) {
#ifdef J1708_DEBUG
        {
            int i;
            pr_err("%s() count %d\n", __func__, msg.len);
            pr_err("%s() data: ", __func__);
    		for (i = 0; i < msg.len; i++) {
                pr_err("%x ", (unsigned int)(msg.data[i]));
    		}
            pr_err("\n");
    	}
#endif
			if(copy_to_user(buf + read_count, msg.data, msg.len))
				return -EINVAL;

			read_count += msg.len;
		}
	}

	return read_count;
}

static ssize_t virt_j1708_chr_write_in(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos)
{
	struct virt_j1708 * dev = file->private_data;
	struct j1708_msg msg;
	ssize_t w = 0;

	if(count > J1708_MSG_PAYLOAD_SIZE)
		return -EINVAL;

	// This will never block
    if(copy_from_user(msg.data, buf, count))
        return -EACCES;

#ifdef J1708_DEBUG
    {
        int i;
        pr_err("%s() count %x\n", __func__, (unsigned int)count);
        pr_err("%s() data: ", __func__);
		for (i = 0; i < count; i++) {
            pr_err("%x ", (unsigned int)(msg.data[i]));
		}
        pr_err("\n");
	}
#endif

    msg.len = count;

    mutex_lock(&dev->queue_in.lock);
    queue_add_msg(&dev->queue_in, &msg);
    mutex_unlock(&dev->queue_in.lock);

    wake_up_interruptible(&dev->queue_in.wq);
    w += count;

	return w;
}

/// 
/* Transuctions to MCU */
static unsigned int j1708_dev_poll_out(struct file * file,  poll_table * wait)
{
	struct virt_j1708 * dev = file->private_data;
	unsigned int mask = 0;

	mutex_lock(&dev->queue_in.lock);
	poll_wait(file, &dev->queue_in.wq, wait);
	if(!queue_empty(&dev->queue_in))
		mask |= POLLIN | POLLRDNORM;
	mutex_unlock(&dev->queue_in.lock);

	return mask;
}

static ssize_t virt_j1708_chr_read_out(struct file * file, char __user * buf,
		size_t count, loff_t *ppos)
{
	struct virt_j1708 * dev = file->private_data;//&g_virt_j1708_mcu_dev;
    struct j1708_msg msg = {0};
	//uint8_t data[J1708_MSG_PAYLOAD_SIZE];
	int read_count = 0;

	if(count < J1708_MSG_PAYLOAD_SIZE) {
		pr_err("%s() count < %d\n", __func__, J1708_MSG_PAYLOAD_SIZE);
		return -EINVAL;
	}

	if (count >= J1708_MSG_PAYLOAD_SIZE) {
		struct j1708_msg * pmsg = NULL;

		mutex_lock(&dev->queue_in.lock);
		while(queue_empty(&dev->queue_in)) {
			mutex_unlock(&dev->queue_in.lock);
			if((file->f_flags &  O_NONBLOCK))
				return -EAGAIN;
			wait_event_interruptible(dev->queue_in.wq, !queue_empty(&dev->queue_out));
			mutex_lock(&dev->queue_in.lock);
		}

		if( (pmsg = queue_get_msg(&dev->queue_in)) )
			memcpy(&msg, pmsg, sizeof(msg));

		mutex_unlock(&dev->queue_in.lock);

		if(pmsg && msg.len) {
#ifdef J1708_DEBUG
        {
            int i;
            pr_err("%s() count %d\n", __func__, msg.len);
            pr_err("%s() data: ", __func__);
    		for (i = 0; i < msg.len; i++) {
                pr_err("%x ", (unsigned int)(msg.data[i]));
    		}
            pr_err("\n");
    	}
#endif
			if(copy_to_user(buf + read_count, msg.data, msg.len))
				return -EINVAL;

			read_count += J1708_MSG_PAYLOAD_SIZE;
		}
	}

	return read_count;
}

static ssize_t virt_j1708_chr_write_out(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos)
{
	struct virt_j1708 * dev = file->private_data;
	struct j1708_msg msg;
	ssize_t w = 0;

	if(count > J1708_MSG_PAYLOAD_SIZE)
		return -EINVAL;

    if(copy_from_user(msg.data, buf, count))
        return -EACCES;

#ifdef J1708_DEBUG
    {
        int i;
        pr_err("%s() count %x\n", __func__, (unsigned int)count);
        pr_err("%s() data: ", __func__);
		for (i = 0; i < count; i++) {
            pr_err("%x ", (unsigned int)(msg.data[i]));
		}
        pr_err("\n");
	}
#endif

    mutex_lock(&dev->queue_out.lock);
    queue_add_msg(&dev->queue_out, &msg);
    mutex_unlock(&dev->queue_out.lock);

    wake_up_interruptible(&dev->queue_out.wq);
    w += count;

	return w;
} 
/// 

static const struct file_operations j1708_dev_fops_in = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_j1708_chr_read_in,
	.write = virt_j1708_chr_write_in,
	.open = j1708_dev_open,
	.release = j1708_dev_release,
	.poll   = j1708_dev_poll_in,
};

static struct miscdevice j1708_dev_in = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "mcu_j1708",
	.fops		= &j1708_dev_fops_in,
};

static const struct file_operations j1708_dev_fops_out = {
	.owner  = THIS_MODULE,
	.llseek = no_llseek,
	.read = virt_j1708_chr_read_out,
	.write = virt_j1708_chr_write_out,
	.open = j1708_dev_open,
	.release = j1708_dev_release,
	.poll   = j1708_dev_poll_out,
};

static struct miscdevice j1708_dev_out = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "j1708",
	.fops		= &j1708_dev_fops_out,
};

static int __init virtual_j1708_init(void)
{
	struct virt_j1708 * dev = &g_virt_j1708_mcu_dev;
	int ret;

	init_waitqueue_head(&dev->queue_in.wq);
	mutex_init(&dev->queue_in.lock);

    init_waitqueue_head(&dev->queue_out.wq);
	mutex_init(&dev->queue_out.lock);


	ret = misc_register(&j1708_dev_in);
	if(ret) {
		pr_err("%s() Unable to register misc device j1708_dev_in\n", __func__);
		return ret;
	}

    ret = misc_register(&j1708_dev_out);
	if(ret) {
		pr_err("%s() Unable to register misc device j1708_dev_out\n", __func__);
		return ret;
	}

	return 0;
}

static void __exit virtual_j1708_exit(void)
{

	misc_deregister(&j1708_dev_in);
    misc_deregister(&j1708_dev_out);
}


module_init(virtual_j1708_init);
module_exit(virtual_j1708_exit);

MODULE_AUTHOR("Ruslan Sirota <ruslan.sirota@micronet-inc.com>");
MODULE_DESCRIPTION("J1708 host interface");
MODULE_LICENSE("GPL");

