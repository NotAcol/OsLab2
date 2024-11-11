/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
 *
 */

#include "lunix-chrdev.h"

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <string.h>

#include "lunix-lookup.h"
#include "lunix.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
static int lunix_chrdev_state_needs_refresh(
    struct lunix_chrdev_state_struct *state) {
    // Why copy instead of just read ? doesnt even have to be atomic
    struct lunix_sensor_struct *sensor;

    WARN_ON(!(sensor = state->sensor));  // assert + log

    if (sensor->msr_data[state->type]->last_update != state->buf_timestamp)
        return 1;
    return 0;
    // 1 for need update 0 for up to date
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state) {
    uint16_t value = 0, timestamp = 0;

    debug("leaving\n");

    /*
     * Grab the raw data quickly, hold the
     * spinlock for as little as possible.
     */
    spin_lock(&state->sensor->lock);

    value = state->sensor->msr_data[state->type]->values[0];
    timestamp = state->sensor->msr_data[state->type]->last_update;

    spin_unlock(&state->sensor->lock);

    /*
     * Any new data available?
     */
    if (timestamp == state->buf_timestamp) return -EAGAIN;

    /*
     * Now we can take our time to format them,
     * holding only the private state semaphore
     */
    // NOTE(acol): not holding semaphores here cause this is only called by read
    // when it's already locked
    switch (state->type) {
        case BATT: {
            state->buf_lim =
                sprintf(state->buf_data, "%ld.%ld", uint16_to_batt(value) / 100,
                        uint16_to_batt(value) % 100);
        } break;
        case TEMP: {
            state->buf_lim =
                sprintf(state->buf_data, "%ld.%ld", uint16_to_temp(value) / 100,
                        uint16_to_temp(value) % 100);
        } break;
        case LIGHT: {
            state->buf_lim = sprintf(state->buf_data, "%ld.%ld",
                                     uint16_to_light(value) / 100,
                                     uint16_to_light(value) % 100);
        } break;
    }
    --state->buf_lim;  // removing trailing '\0'
    state->buf_timestamp = timestamp;

    debug("leaving\n");
    return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp) {
    /* Declarations */
    struct lunix_chrdev_state_struct *state;
    int ret;

    debug("entering\n");
    ret = -ENODEV;
    if ((ret = nonseekable_open(inode, filp)) < 0) goto out;

    /*
     * Associate this open file with the relevant sensor based on
     * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
     */
    unsigned int minor_num = iminor(inode);

    /* Allocate a new Lunix character device private state structure */
    state = kzalloc(sizeof(*state), GFP_KERNEL);
    filp->private_data = state;

    // TODO(acol): maybe check filp->f_flags
    init_MUTEX(state->lock);
    state->type =
        (lunix_msr_enum)((minor_num & 1) |
                         (minor_num & 2));  // NOTE(acol): its either 0 1 or 2
    state->sensor = lunix_sensors + (minor_num / 8);
    // NOTE(acol): lunix_sensors is declared as extern in lunix.h and
    // initialized in lunix-module.c
}

out : debug("leaving, with ret = %d\n", ret);
return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp) {
    kfree(flip->private_data);
    return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd,
                               unsigned long arg) {
    /* Why? */
    return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf,
                                 size_t cnt, loff_t *f_pos) {
    ssize_t ret = 0;

    struct lunix_sensor_struct *sensor;
    struct lunix_chrdev_state_struct *state;

    state = filp->private_data;
    WARN_ON(!state);

    sensor = state->sensor;
    WARN_ON(!sensor);

    if (down_interruptible(&state->lock)) return -ERESTARTSYS;

    // NOTE(acol): write till we run out of buffer space
    while (cnt > 0) {
        // NOTE(acol): If we have fully written last value fetch a new one
        if (*f_pos == 0) {
            while (lunix_chrdev_state_update(state) == -EAGAIN) {
                up(&state->lock);
                if (flip->f_flags & O_NOBLOCK) return -EAGAIN;
                if (wait_event_interruptible(
                        &sensor->wq, lunix_chrdev_state_needs_refresh(state)))
                    return -ERESTARTSYS;  // Returns anything other than 0 if we
                                          // wake up to an interupt
                if (down_interruptible(&state->lock)) return -ERESTARTSYS;
            }
        }  // fetch done
        // NOTE(acol): checks if we should write more than one value, cnt >
        // buf_lim+1 cause I dont want to write a value and just a space after
        if (cnt > state->buf_lim + 1 - *f_pos) {
            ret += state->buf_lim + 1 - *f_pos;
            cnt -= state->buf_lim + 1 - *f_pos;
            // NOTE(acol): add a space at the end of the internal buffer and
            // copy to user
            state->buf_data[state->buf_lim++] =
                ' ';  // buf_lim should always be 7
            if (copy_to_user(usrbuf, state->buf_data + *f_pos,
                             state->buf_lim - *f_pos)) {
                // if it cant copy everything unlock mutex and segfault :D
                up(&state->lock);
                return -EFAULT;
            }
            // NOTE(acol): update user buffer offset
            userbuf += state->buf_lim - *f_pos *f_pos = 0;
        } else {
            // NOTE(acol): checks if a full value will fit
            if (cnt >= state->buf_lim - *f_pos) {
                cnt = state->buf_lim - *f_pos;
                if (copy_to_user(usrbuf, state->buf_data + *f_pos,
                                 state->buf_lim - *f_pos)) {
                    up(&state->lock);
                    return -EFAULT;
                }
                *f_pos = 0;
            } else {
                if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) {
                    up(&state->lock);
                    return -EFAULT;
                }
                *f_pos += cnt;
            }
            ret += cnt;
            cnt = 0;  // will exit loop
        }
    }
    up(&state->lock);
    return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma) {
    return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = {
    .owner = THIS_MODULE,
    .open = lunix_chrdev_open,
    .release = lunix_chrdev_release,
    .read = lunix_chrdev_read,
    .unlocked_ioctl = lunix_chrdev_ioctl,
    .mmap = lunix_chrdev_mmap};

int lunix_chrdev_init(void) {
    /*
     * Register the character device with the kernel, asking for
     * a range of minor numbers (number of sensors * 8 measurements / sensor)
     * beginning with LINUX_CHRDEV_MAJOR:0
     */
    int ret;
    dev_t dev_no;
    // NOTE(acol): We are using 0 1 2 why are we using 3 bits instead of 2
    // ???
    unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

    debug("initializing character device\n");
    cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
    lunix_chrdev_cdev.owner = THIS_MODULE;

    dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
    ret = register_chrdev_region(dev_no, lunix_minor_cnt, "Lunix:TNG");

    if (ret < 0) {
        debug("failed to register region, ret = %d\n", ret);
        goto out;
    }
    // FIX(acol): this is maybe wrong, it could need lunix_minor_cnt here
    //             or N_LUNIX_MSR or maybe even 2^8
    //             not sure if it's per device or overall number BUG
    ret = cdev_add(&lunix_chrdev_cdev, dev_no, 1);
    if (ret < 0) {
        debug("failed to add character device\n");
        goto out_with_chrdev_region;  // goto lmao
    }
    debug("completed successfully\n");
    return 0;

out_with_chrdev_region:
    unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
    return ret;
}

void lunix_chrdev_destroy(void) {
    dev_t dev_no;
    unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

    debug("entering\n");
    dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
    cdev_del(&lunix_chrdev_cdev);
    unregister_chrdev_region(dev_no, lunix_minor_cnt);
    debug("leaving\n");
}
