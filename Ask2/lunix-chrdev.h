/*
 * lunix-chrdev.h
 *
 * Definition file for the
 * Lunix:TNG character device
 *
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 */

#ifndef _LUNIX_CHRDEV_H
#define _LUNIX_CHRDEV_H

/*
 * Lunix:TNG character device
 */
#define LUNIX_CHRDEV_MAJOR 60 /* Reserved for local / experimental use */
#define LUNIX_CHRDEV_BUFSZ 20 /* Buffer size used to hold textual info */

/* Compile-time parameters */

#ifdef __KERNEL__

#include <asm/semaphore.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "lunix.h"

/*
 * Private state for an open character device node
 */
struct lunix_chrdev_state_struct {
    enum lunix_msr_enum type;  // BATT TEMP LIGHT
    struct lunix_sensor_struct *sensor;
    //  {
    //      struct lunix_msr_data_struct *msr_data[N_LUNIX_MSR];
    //          {
    //            uint32_t magic;
    //            uint32_t last_update;
    //            uint32_t values[];
    //          }
    //      spinlock_t lock;
    //      wait_queue_head_t wq;
    //  }

    /* A buffer used to hold cached textual info */
    int buf_lim;  // just using this as a counter of how many bytes were last
                  // written to buf_data (mainly for copy_to_user)
    unsigned char buf_data[LUNIX_CHRDEV_BUFSZ];
    uint32_t buf_timestamp;

    struct semaphore lock;

    /*
     * Fixme: Any mode settings? e.g. blocking vs. non-blocking
     */
};

/*
 * Function prototypes
 */
int lunix_chrdev_init(void);
void lunix_chrdev_destroy(void);

#endif /* __KERNEL__ */

#include <linux/ioctl.h>

/*
 * Definition of ioctl commands
 */
#define LUNIX_IOC_MAGIC LUNIX_CHRDEV_MAJOR
// #define LUNIX_IOC_EXAMPLE		_IOR(LUNIX_IOC_MAGIC, 0, void *)

#define LUNIX_IOC_MAXNR 0

#endif /* _LUNIX_H */
