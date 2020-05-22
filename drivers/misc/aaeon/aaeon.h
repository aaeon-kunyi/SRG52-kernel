#ifndef _AAEON_ANDROID_H_
#define _AAEON_ANDROID_H_

#include <linux/cdev.h>
#include <linux/semaphore.h>

#define IOC_MAGIC 'd'
#define AAEON_DEVICE_NODE_NAME  "aaeon_node"
#define AAEON_DEVICE_FILE_NAME  "aaeon_file"
#define AAEON_DEVICE_PROC_NAME  "aaeon_proc"
#define AAEON_DEVICE_CLASS_NAME "aaeon_class"

struct aaeon_android_dev {
    int val;
    struct semaphore sem;
    struct cdev dev;
};

struct ioctl_cmd {
    unsigned int val;
    unsigned int data;
    unsigned int port;
    unsigned char* inf;
};

#define IOCTL_IORW8		_IOW(IOC_MAGIC, 1, struct ioctl_cmd)
#define IOCTL_RED_LED		_IOW(IOC_MAGIC, 2, struct ioctl_cmd)
#define IOCTL_YELLOW_LED	_IOW(IOC_MAGIC, 3, struct ioctl_cmd)
#define IOCTL_GREEN_LED		_IOW(IOC_MAGIC, 4, struct ioctl_cmd)
#define IOCTL_BOARD_INFO	_IOW(IOC_MAGIC, 5, struct ioctl_cmd)
#define IOCTL_RED_PWM           _IOW(IOC_MAGIC, 6, struct ioctl_cmd)

#define YELLOW_LED_GPIO 403
#define RED_LED_GPIO 430
#define GREEN_LED_GPIO 404

#endif

