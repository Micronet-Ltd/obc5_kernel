/*!
 * @section LICENSE
 * $license$
 *
 * @filename $filename$
 * @date     $date$
 * @id       $id$
 *
 * @brief
 * The core code of bst device driver
*/

#ifndef _BSTCLASS_H
#define _BSTCLASS_H

#ifdef __KERNEL__
#include <linux/time.h>
#include <linux/list.h>
#else
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/types.h>
#endif

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mod_devicetable.h>

struct bst_dev {
	const char *name;

	int (*open)(struct bst_dev *dev);
	void (*close)(struct bst_dev *dev);
	struct mutex mutex;
	struct device dev;
	struct list_head node;
};

#define to_bst_dev(d) container_of(d, struct bst_dev, dev)

struct bst_dev *bst_allocate_device(void);
void bst_free_device(struct bst_dev *dev);

static inline struct bst_dev *bst_get_device(struct bst_dev *dev)
{
	return dev ? to_bst_dev(get_device(&dev->dev)) : NULL;
}

static inline void bst_put_device(struct bst_dev *dev)
{
	if (dev)
		put_device(&dev->dev);
}

static inline void *bst_get_drvdata(struct bst_dev *dev)
{
	return dev_get_drvdata(&dev->dev);
}

static inline void bst_set_drvdata(struct bst_dev *dev, void *data)
{
	dev_set_drvdata(&dev->dev, data);
}

int __must_check bst_register_device(struct bst_dev *);
void bst_unregister_device(struct bst_dev *);

void bst_reset_device(struct bst_dev *);


extern struct class bst_class;

#endif
