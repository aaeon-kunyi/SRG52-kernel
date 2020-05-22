#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/gpio.h>
//========== TEST ==========
#include <linux/dmi.h>
#include <linux/pwm.h>
//========== TEST ==========
#include <aaeon/aaeon_api.h>
#include "aaeon.h"

static int aaeon_major = 0;
static int aaeon_minor = 0;

static struct class* aaeon_class = NULL;
static struct aaeon_android_dev* aaeon_dev = NULL;

static int aaeon_open(struct inode* inode, struct file* filp);
static int aaeon_release(struct inode* inode, struct file* filp);
static ssize_t aaeon_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos);
static ssize_t aaeon_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos);
static long aaeon_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
//========== TEST ==========
static struct pwm_device* pwm0 = NULL;
//========== TEST ==========

static struct file_operations aaeon_fops = {
    .owner = THIS_MODULE,
    .open = aaeon_open,
    .release = aaeon_release,
    .read = aaeon_read,
    .write = aaeon_write,
    .unlocked_ioctl = aaeon_ioctl,
};

static int aaeon_open(struct inode* inode, struct file* filp) {
    struct aaeon_android_dev* dev;
    int err = 0;

    printk("aaeon driver: aaeon_open\n");

    dev = container_of(inode->i_cdev, struct aaeon_android_dev, dev);
    filp->private_data = dev;

    printk("aaeon driver: request gpio (red) %d.\n ", RED_LED_GPIO);
    err = gpio_request(RED_LED_GPIO, "RED_LED");
    if (err < 0) {
        printk("aaeon driver: failed to request gpio (red) %d, err = %d.\n", RED_LED_GPIO, err);
    }
    gpio_direction_output(RED_LED_GPIO, 1);
    gpio_set_value(RED_LED_GPIO, 1);

    printk("aaeon driver: request gpio (yellow) %d.\n ", YELLOW_LED_GPIO);
    err = gpio_request(YELLOW_LED_GPIO, "YELLOW_LED");
    if (err < 0) {
        printk("aaeon driver: failed to request gpio (yellow) %d, err = %d.\n", YELLOW_LED_GPIO, err);
    }
    gpio_direction_output(YELLOW_LED_GPIO, 1);
    gpio_set_value(YELLOW_LED_GPIO, 1);

    printk("aaeon driver: request gpio (green) %d.\n ", GREEN_LED_GPIO);
    err = gpio_request(GREEN_LED_GPIO, "GREEN_LED");
    if (err < 0) {
        printk("aaeon driver: failed to request gpio (green) %d, err = %d.\n", GREEN_LED_GPIO, err);
    }
    gpio_direction_output(GREEN_LED_GPIO, 1);
    gpio_set_value(GREEN_LED_GPIO, 1);

    pwm0 = pwm_request(0, "RED_LED_PWM");
    if (pwm0 == NULL)
    printk("aaeon driver: failed to request pwm0\n");

    err = pwm_config(pwm0, 3413333, 3413333);
    if (err < 0) {
    printk("aaeon driver: failed to config pwm, err = %d\n", err);
    }

    err = pwm_enable(pwm0);
    if (err < 0) {
	    printk("aaeon driver: failed to enable pwm, err = %d\n", err);
    }

    return 0;
}

static int aaeon_release(struct inode* inode, struct file* filp) {
    printk("aaeon driver: aaeon_release\n");

    //ADD
    gpio_free(RED_LED_GPIO);
    gpio_free(GREEN_LED_GPIO);
    gpio_free(YELLOW_LED_GPIO);

    pwm_disable(pwm0);
    return 0;
}

static ssize_t aaeon_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos) {
    ssize_t err = 0;
    struct aaeon_android_dev* dev = filp->private_data;
    static bool finished = false;

    const char *dmi_product_name;

    printk("aaeon driver: aaeon_read\n");

    //============= TEST ==============
/*  
    printk("aaeon driver: get DMI_PRODUCT_NAME");
    dmi_product_name = dmi_get_system_info(DMI_PRODUCT_NAME);
    printk("aaeon driver: DMI_SYS_VENDOR = %s\n", dmi_product_name);
*/  
  //============= TEST ==============

    if(down_interruptible(&(dev->sem))) {
        return -ERESTARTSYS;
    }

//    printk("aaeon driver: count = %d, sizeof(dev->val) = %d, err = %d, finished = %d\n", count, sizeof(dev->val), err, finished);

    if(count < sizeof(dev->val)) {
        printk("aaeon driver: count > sizeof val, go to out\n");
        goto out;
    }

    if(copy_to_user((int __user *)buf, &(dev->val), sizeof(dev->val))) {
        printk("aaeon driver: copy to user failed\n");
        err = -EFAULT;
        goto out;
    }

    if(!finished) {
        err = sizeof(dev->val);
        finished = true;
    } else {
        finished = false;
    }
//    printk("aaeon driver: count = %d, sizeof(dev->val) = %d, err = %d, finished = %d\n", count, sizeof(dev->val), err, finished);

    //========== TEST ==========
/*
    gpio_set_value(RED_LED_GPIO, 0);
    printk("aaeon driver: red led on.\n");

    printk("aaeon driver: config red led pwm\n");
    pwm_config(pwm0, 3413333, 3413333);
    mdelay(500);
    pwm_config(pwm0, 2700000, 3413333);
    mdelay(500);
    pwm_config(pwm0, 1500000, 3413333);
    mdelay(500);
    pwm_config(pwm0, 2700000, 3413333);
    mdelay(500);
    pwm_config(pwm0, 3413333, 3413333);
*/
    //========== TEST ==========
out:
    up(&(dev->sem));
    return err;
}

static ssize_t aaeon_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos) {
    struct aaeon_android_dev* dev = filp->private_data;
    ssize_t err = 0;

    printk("aaeon driver: aaeon_write\n");

    if(down_interruptible(&(dev->sem))) {
        return -ERESTARTSYS;
    }

    printk("aaeon driver: sizeof(dev->val) = %d, count = %d\n", sizeof(dev->val), count);

    if(copy_from_user(&(dev->val), buf, count)) {
        printk("aaeon driver: copy from user failed\n");
        err = -EFAULT;
        goto out;
    }

    //========== TEST ==========
    gpio_set_value(RED_LED_GPIO, 1);
    printk("aaeon driver: red led off.\n");
    //========== TEST ==========
    err = sizeof(dev->val);

out:
    up(&(dev->sem));
    return err;
}

static ssize_t aaeon_val_show(struct device* dev, struct device_attribute* attr,  char* buf);
static ssize_t aaeon_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count);
static DEVICE_ATTR(aaeon_val, S_IRUGO | S_IWUSR, aaeon_val_show, aaeon_val_store);

static ssize_t __aaeon_get_val(struct aaeon_android_dev* dev, char* buf) {
    int val = 0;

    printk("aaeon driver: __aaeon_get_val\n");

    if(down_interruptible(&(dev->sem))) {
        return -ERESTARTSYS;
    }

    val = dev->val;
    up(&(dev->sem));

    return snprintf(buf, sizeof(val)+1, "%d\n", val);
}

static ssize_t __aaeon_set_val(struct aaeon_android_dev* dev, const char* buf, size_t count) {
    int val = 0;

    printk("aaeon driver: __aaeon_set_val\n");

    val = simple_strtol(buf, NULL, 10);

    if(down_interruptible(&(dev->sem))) {
        return -ERESTARTSYS;
    }

    dev->val = val;
    up(&(dev->sem));

    return count;
}

static ssize_t aaeon_val_show(struct device* dev, struct device_attribute* attr, char* buf) {
    struct aaeon_android_dev* adev = (struct aaeon_android_dev*)dev_get_drvdata(dev);
    printk("aaeon driver: aaeon_val_show\n");

    return __aaeon_get_val(adev, buf);
}

static ssize_t aaeon_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    struct aaeon_android_dev* adev = (struct aaeon_android_dev*)dev_get_drvdata(dev);
    printk("aaeon driver: aaeon_val_store\n");

    return __aaeon_set_val(adev, buf, count);
}

static ssize_t aaeon_proc_read(struct file* filp, char *buff, size_t count, loff_t *pos) {
    static bool finished = false;
    int ret = 0;
    printk("aaeon driver: aaeon_proc_read\n");

    ret = __aaeon_get_val(aaeon_dev, buff);

    if (ret < 0) {
        finished = true;
    return -EFAULT;
    }

    if (!finished) {
        finished = true;
        return ret;
    } else {
        finished = false;
        return 0;
    }
}

static ssize_t aaeon_proc_write(struct file* filp, const char __user *buff, size_t len, loff_t *pos) {
    int err = 0;
    char* page = NULL;
    printk("aaeon driver: aaeon_proc_write\n");

    if(len > PAGE_SIZE) {
        printk("aaeon driver: the buff is too large: %lu.\n", len);
        return -EFAULT;
    }

    page = (char*)__get_free_page(GFP_KERNEL);
    if(!page) {
        printk("aaeon driver: failed to alloc page.\n");
        return -ENOMEM;
    }

    if(copy_from_user(page, buff, len)) {
        printk("aaeon driver: failed to copy buff from user.\n");
        err = -EFAULT;
        goto out;
    }

    err = __aaeon_set_val(aaeon_dev, page, len);

out:
    free_page((unsigned long)page);
    printk("aaeon driver: return err %d\n", err);
    return err;
}

static const struct file_operations aaeon_proc_fops = {
    .owner = THIS_MODULE,
    .write = aaeon_proc_write,
    .read = aaeon_proc_read,
};

static void aaeon_create_proc(void) {
    struct proc_dir_entry* entry;
    printk("aaeon driver: aaeon_create_proc\n");

    entry = proc_create(AAEON_DEVICE_PROC_NAME, 0x0644, aaeon_proc_root, &aaeon_proc_fops);

    if (entry == NULL) {
        printk("aaeon driver: proc create failed\n");
    }
}

static void aaeon_remove_proc(void) {
    printk("aaeon driver: aaeon_remove_proc\n");
    remove_proc_entry(AAEON_DEVICE_PROC_NAME, NULL);
}

static int  __aaeon_setup_dev(struct aaeon_android_dev* dev) {
    int err;
    printk("aaeon driver: __aaeon_setup_dev\n");

    dev_t devno = MKDEV(aaeon_major, aaeon_minor);

    memset(dev, 0, sizeof(struct aaeon_android_dev));

    cdev_init(&(dev->dev), &aaeon_fops);
    dev->dev.owner = THIS_MODULE;
    dev->dev.ops = &aaeon_fops;

    err = cdev_add(&(dev->dev),devno, 1);
    if(err) {
        printk("aaeon driver: cdev_add failed\n");
        return err;
    }

    sema_init((&dev->sem),1);
    dev->val = 0;

    return 0;
}

static int __init aaeon_init(void){
    int err = -1;
    dev_t dev = 0;
    struct device* temp = NULL;

    printk("aaeon driver: initializing aaeon device.\n");

    err = alloc_chrdev_region(&dev, 0, 1, AAEON_DEVICE_NODE_NAME);
    if(err < 0) {
        printk("aaeon driver: failed to alloc char dev region.\n");
        goto fail;
    }

    aaeon_major = MAJOR(dev);
    aaeon_minor = MINOR(dev);

    aaeon_dev = kmalloc(sizeof(struct aaeon_android_dev), GFP_KERNEL);
    if(!aaeon_dev) {
        err = -ENOMEM;
        printk("aaeon driver: failed to alloc aaeon_dev.\n");
        goto unregister;
    }

    err = __aaeon_setup_dev(aaeon_dev);
    if(err) {
        printk("aaeon driver: failed to setup dev: %d.\n", err);
        goto cleanup;
    }

    aaeon_class = class_create(THIS_MODULE, AAEON_DEVICE_CLASS_NAME);
    if(IS_ERR(aaeon_class)) {
        err = PTR_ERR(aaeon_class);
        printk("aaeon driver: failed to create aaeon class.\n");
        goto destroy_cdev;
    }

    temp = device_create(aaeon_class, NULL, dev, "%s", AAEON_DEVICE_FILE_NAME);
    if(IS_ERR(temp)) {
        err = PTR_ERR(temp);
        printk("aaeon driver: failed to create aaeon device.\n");
        goto destroy_class;
    }

    err = device_create_file(temp, &dev_attr_aaeon_val);
    if(err < 0) {
        printk("aaeon driver: failed to create attribute aaeon_val.\n");
        goto destroy_device;
    }

    dev_set_drvdata(temp, aaeon_dev);

    aaeon_create_proc();

    printk("aaeon driver: succedded to initialize aaeon device.\n");
    return 0;

destroy_device:
    device_destroy(aaeon_class, dev);

destroy_class:
    class_destroy(aaeon_class);

destroy_cdev:
    cdev_del(&(aaeon_dev->dev));

cleanup:
    kfree(aaeon_dev);

unregister:
    unregister_chrdev_region(MKDEV(aaeon_major, aaeon_minor), 1);

fail:
    return err;
}

static void __exit aaeon_exit(void) {
    dev_t devno = MKDEV(aaeon_major, aaeon_minor);

    printk("aaeon driver: destroy aaeon device.\n");

    aaeon_remove_proc();

    if(aaeon_class) {
        device_destroy(aaeon_class, MKDEV(aaeon_major, aaeon_minor));
        class_destroy(aaeon_class);
    }

    if(aaeon_dev) {
        cdev_del(&(aaeon_dev->dev));
        kfree(aaeon_dev);
    }

    unregister_chrdev_region(devno, 1);
}

static void aaeon_led_switch(int gpio, bool on) {
    if (on)
    gpio_set_value(gpio, 0);
    else
    gpio_set_value(gpio, 1);
}

static long aaeon_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
  int retval = 0;
  struct ioctl_cmd input_data;
  char* info;

  memset(&input_data, 0, sizeof(input_data));

  printk("aaeon_driver: aaeon ioctl!!!!\n");

  switch (cmd) {

    case IOCTL_IORW8:
    printk("aaeon_driver: IOCTL_IORW8\n");
        if (copy_from_user(&input_data, (int __user *)arg, sizeof(input_data))) {
            printk("aaeon driver: copy from user failed\n");
            retval = -EFAULT;
            goto done;
        }

    printk("aaeon driver: aaeon_ioctl, R/W = %d (R=0,W=1)\n", input_data.val);

        if(input_data.val == 1) {
            printk("aaeon driver: outb, data = %d, port = 0x%x\n", input_data.data, input_data.port);
            outb(input_data.data, input_data.port);
        } else {
            printk("aaeon driver: inb, port = 0x%x\n", input_data.port);
            input_data.data = inb(input_data.port);
        printk("aaeon driver: inb, data = %d, port = 0x%x\n", input_data.data, input_data.port);
            if (copy_to_user((int __user *)arg, &input_data, sizeof(input_data))) {
                printk("aaeon drivr: copy to user failed\n");
                retval = -EFAULT;
                goto done;
            }
        }
        break;

    case IOCTL_RED_LED:
    printk("aaeon_driver: IOCTL_RED_LED\n");
        if (copy_from_user(&input_data, (int __user *)arg, sizeof(input_data))) {
            printk("aaeon driver: copy from user failed\n");
            retval = -EFAULT;
            goto done;
        }

        if (input_data.val == 1) {
        printk("aaeon_driver: R/W = %d (R=0,W=1)\n", input_data.val);
        if (input_data.data == 1) {
            printk("aaeon_driver: RED LED ON\n");
            aaeon_led_switch(RED_LED_GPIO, true);
        } else {
            printk("aaeon_driver: RED LED OFF\n");
            aaeon_led_switch(RED_LED_GPIO, false);
        }
        } else {
        printk("aaeon driver: R/W = %d (R=0,W=1), not support!\n", input_data.val);
        }
    break;
    case IOCTL_YELLOW_LED:
    printk("aaeon_driver: IOCTL_YELLOW_LED\n");
        if (copy_from_user(&input_data, (int __user *)arg, sizeof(input_data))) {
            printk("aaeon driver: copy from user failed\n");
            retval = -EFAULT;
            goto done;
        }

        if (input_data.val == 1) {
        printk("aaeon_driver: R/W = %d (R=0,W=1)\n", input_data.val);
        if (input_data.data == 1) {
            printk("aaeon_driver: YELLOW LED ON\n");
            aaeon_led_switch(YELLOW_LED_GPIO, true);
        } else {
            printk("aaeon_driver: YELLOW LED OFF\n");
            aaeon_led_switch(YELLOW_LED_GPIO, false);
        }
        } else {
        printk("aaeon driver: R/W = %d (R=0,W=1), not support!\n", input_data.val);
        }
    break;
    case IOCTL_GREEN_LED:
    printk("aaeon_driver: IOCTL_GREEN_LED\n");
        if (copy_from_user(&input_data, (int __user *)arg, sizeof(input_data))) {
            printk("aaeon driver: copy from user failed\n");
            retval = -EFAULT;
            goto done;
        }

        if (input_data.val == 1) {
        printk("aaeon_driver: R/W = %d (R=0,W=1)\n", input_data.val);
        if (input_data.data == 1) {
            printk("aaeon_driver: GREEN LED ON\n");
            aaeon_led_switch(GREEN_LED_GPIO, true);
        } else {
            printk("aaeon_driver: GREEN LED OFF\n");
            aaeon_led_switch(GREEN_LED_GPIO, false);
        }
        } else {
        printk("aaeon driver: R/W = %d (R=0,W=1), not support!\n", input_data.val);
        }
    break;
    case IOCTL_BOARD_INFO:
    printk("aaeon_driver: IOCTL_BOARD_INFO\n");
        if (copy_from_user(&input_data, (int __user *)arg, sizeof(input_data))) {
            printk("aaeon driver: copy from user failed\n");
            retval = -EFAULT;
            goto done;
        }

        printk("aaeon_driver: input_data.info = %s\n", input_data.inf);
        if (input_data.val == 1) {
        printk("aaeon driver: R/W = %d (R=0,W=1), not support!\n", input_data.val);
        } else {
        printk("aaeon_driver: R/W = %d (R=0,W=1)\n", input_data.val);

        if (input_data.data == 0) {
            printk("aaeon driver: GET BIOS VENDOR\n");
	    sprintf(input_data.inf, dmi_get_system_info(DMI_BIOS_VENDOR));
        } else if (input_data.data == 1) {
            printk("aaeon driver: GET BIOS VERSION\n");
            sprintf(input_data.inf, dmi_get_system_info(DMI_BIOS_VERSION));
        } else if (input_data.data == 2) {
            printk("aaeon driver: GET BIOS DATE\n");
            sprintf(input_data.inf, dmi_get_system_info(DMI_BIOS_DATE));
        } else if (input_data.data == 3) {
            printk("aaeon driver: GET PRODUCT NAME\n");
 	    sprintf(input_data.inf, dmi_get_system_info(DMI_PRODUCT_NAME));
        } else if (input_data.data == 4) {
            printk("aaeon driver: GET PRODUCT VERSION\n");
            sprintf(input_data.inf, dmi_get_system_info(DMI_PRODUCT_VERSION));
        } else if (input_data.data == 5) {
            printk("aaeon driver: GET BOARD VENDOR\n");
            sprintf(input_data.inf, dmi_get_system_info(DMI_BOARD_VENDOR));
        } else if (input_data.data == 6) {
            printk("aaeon driver: GET BOARD NAME\n");
	    sprintf(input_data.inf, dmi_get_system_info(DMI_BOARD_NAME));
        } else if (input_data.data == 7) {
            printk("aaeon driver: GET BOARD VERSION\n");
            sprintf(input_data.inf, dmi_get_system_info(DMI_BOARD_VERSION));
        } else {
            printk("aaeon driver: input_data.data = %d not support!\n", input_data.data);
        }
            //    if (copy_to_user((char __user *)arg, info, sizeof(*info))) {
            if (copy_to_user((int __user *)arg, &input_data, sizeof(input_data))) {
                    printk("aaeon drivr: copy to user failed\n");
                    retval = -EFAULT;
                    goto done;
                }
        }
    break;
    //ADD
    case IOCTL_RED_PWM:
    printk("aaeon_driver: IOCTL_RED_PWM\n");
        if (copy_from_user(&input_data, (int __user *)arg, sizeof(input_data))) {
            printk("aaeon driver: copy from user failed\n");
            retval = -EFAULT;
            goto done;
        }

        if (input_data.val == 1) {
        printk("aaeon_driver: R/W = %d (R=0,W=1)\n", input_data.val);
        if (input_data.data > 0 && input_data.data <= 3413333) {
            printk("aaeon_driver: SET RED PWM duty = %d ns\n", input_data.data);
            pwm_config(pwm0, input_data.data, 3413333);
        } else {
            printk("aaeon_driver: input_data.data = %d not support!\n", input_data.data);
        }
        } else {
        printk("aaeon driver: R/W = %d (R=0,W=1)\n", input_data.val);
        printk("aaeon driver: RED LED BLINKY");
            pwm_config(pwm0, 3413333, 3413333);
            mdelay(500);
            pwm_config(pwm0, 2700000, 3413333);
            mdelay(500);
            pwm_config(pwm0, 1500000, 3413333);
            mdelay(500);
            pwm_config(pwm0, 2700000, 3413333);
            mdelay(500);
            pwm_config(pwm0, 3413333, 3413333);
        }
    break;
    default:
        return -1;
}

done:
    return retval;
}

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AAEON Android Driver");

module_init(aaeon_init);
module_exit(aaeon_exit);

