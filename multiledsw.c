#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>     // for copy_to_user
#include <linux/list.h>        // for linked list
#include <linux/slab.h>        // for kmalloc
#include <linux/proc_fs.h>     // for proc
#include <linux/miscdevice.h>  // for miscdevice
#include <linux/fs.h>          // for inode, file
#include <linux/gpio.h>        // for button gpio
#include <linux/interrupt.h>   // for interruptions
#include <linux/jiffies.h>     // for jiffies
#include <asm-generic/errno.h> // errors
#include <linux/pwm.h>         // for pwm
#include <linux/workqueue.h>   // for workqueue
#include <linux/delay.h>       // for delay
#include <linux/kref.h>        // for reference counter
#include <linux/spinlock.h>    // for spinlock

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juan_Giron_Herranz");

#define BUFFER_SIZE 512
#define MAX_DEVICES 3
#define MANUAL_DEBOUNCE

DEFINE_SPINLOCK(list_lock);

/******************************** gpio ********************************/

typedef struct Led_button_pair
{
    int assigned;
    int value;
    int led_gpio;
    int button_gpio;
    int button_irqn;
    struct gpio_desc *button_descriptor;
    struct gpio_desc *led_descriptor;
    spinlock_t device_lock;

} Led_button_pair_t;

Led_button_pair_t led_button_pairs[MAX_DEVICES] = {
    {.assigned = 0, .value = 0, .button_irqn = -1, .led_gpio = 4, .button_gpio = 22},
    {.assigned = 0, .value = 0, .button_irqn = -1, .led_gpio = 27, .button_gpio = 9},
    {.assigned = 0, .value = 0, .button_irqn = -1, .led_gpio = 25, .button_gpio = 11}};

static Led_button_pair_t *get_led_button_pair_free(void);
static void free_gpios(void);
static int initialize_gpios(void);
static int initialize_led_descriptor(Led_button_pair_t *led_button_pair, const char *gpio_str);
static int initialize_button_descriptor(Led_button_pair_t *led_button_pair, const char *gpio_str);
static irqreturn_t button_irq_handler(int irq, void *dev_id);
static int install_button_irq(Led_button_pair_t *led_button_pair);
static void uninstall_button_irq(Led_button_pair_t *led_button_pair);

/*
 * return the first Led_button_pair with assigned = 0.
 * return NULL if all are assigned.
 */
static Led_button_pair_t *get_led_button_pair_free()
{
    int i;
    for (i = 0; i < MAX_DEVICES; i++)
    {
        if (led_button_pairs[i].assigned == 0)
        {
            return &led_button_pairs[i];
        }
    }
    return NULL;
}

static void free_gpios()
{
    int i;
    for (i = 0; i < MAX_DEVICES; i++)
    {
        led_button_pairs[i].assigned = 0;
        led_button_pairs[i].value = 0;
        gpiod_put(led_button_pairs[i].led_descriptor);
        gpiod_put(led_button_pairs[i].button_descriptor);
    }
}

static int initialize_led_descriptor(Led_button_pair_t *led_button_pair, const char *gpio_str)
{
    int err = 0;

    // Requesting the led GPIO
    if ((err = gpio_request(led_button_pair->led_gpio, gpio_str)))
    {
        pr_err("Failed GPIO %d request\n", led_button_pair->led_gpio);
        err = -EBUSY;
        return err;
    }

    // Transforming into descriptor
    if (!(led_button_pair->led_descriptor = gpio_to_desc(led_button_pair->led_gpio)))
    {
        pr_err("GPIO %d is not valid\n", led_button_pair->led_gpio);
        err = -EINVAL;
        return err;
    }

    gpiod_direction_output(led_button_pair->led_descriptor, 0);

    gpiod_set_value(led_button_pair->led_descriptor, 0);

    return err;
}

static int initialize_button_descriptor(Led_button_pair_t *led_button_pair, const char *gpio_str)
{
    int err = 0;

    // Requesting Button's GPIO
    if ((err = gpio_request(led_button_pair->button_gpio, gpio_str)))
    {
        pr_err("Failed GPIO %d request\n", led_button_pair->button_gpio);
        err = -EBUSY;
        return err;
    }

    // Configure Button
    if (!(led_button_pair->button_descriptor = gpio_to_desc(led_button_pair->button_gpio)))
    {
        pr_err("GPIO %d is not valid\n", led_button_pair->button_gpio);
        err = -EINVAL;
        return err;
    }

    // configure the BUTTON GPIO as input
    gpiod_direction_input(led_button_pair->button_descriptor);

#ifndef MANUAL_DEBOUNCE
    // Debounce the button with a delay of 200ms
    if (gpiod_set_debounce(led_button_pair->button_descriptor, 200) < 0)
    {
        pr_err("ERROR: gpio_set_debounce - %d\n", led_button_pair->button_gpio);
        err = -EINVAL;
        return err;
    }
#endif

    return err;
}

static int initialize_gpios()
{
    int i;
    int err = 0;
    char gpio_str[10];

    for (i = 0; i < MAX_DEVICES; i++)
    {
        sprintf(gpio_str, "led_%d", i);
        err = initialize_led_descriptor(&led_button_pairs[i], gpio_str);

        if (err)
        {
            printk(KERN_WARNING "Error initialize_gpios : initialize_led_descriptor\n");
            goto err_handle;
        }

        sprintf(gpio_str, "button_%d", i);
        err = initialize_button_descriptor(&led_button_pairs[i], gpio_str);

        if (err)
        {
            printk(KERN_WARNING "Error initialize_gpios : initialize_button_descriptor\n");
            goto err_handle;
        }
    }

    return 0;

err_handle:
    free_gpios();

    return err;
}

/******************************** melody ********************************/

#define C4 26163
#define D4 29366
#define E4 32963
#define F4 34923
#define G4 39200
#define C5 52325

#define PWM_DEVICE_NAME "pwmchip0"

struct pwm_device *pwm_device = NULL;
struct pwm_state pwm_state;

struct work_struct melody_work;

struct music_step
{
    unsigned int freq : 24; // Frequency in centihertz
    unsigned int len : 8;   // Duration of the note
};

static void melody_work_function(struct work_struct *work);
static int init_melody_module(void);
static void free_melody_module(void);

// Transform frequency in centiHZ into period in nanoseconds
static inline unsigned int freq_to_period_ns(unsigned int frequency)
{
    if (frequency == 0)
        return 0;
    else
        return DIV_ROUND_CLOSEST_ULL(100000000000UL, frequency);
}

// Check if the current step is and end marker
static inline int is_end_marker(struct music_step *step)
{
    return (step->freq == 0 && step->len == 0);
}

/**
 *  Transform note length into ms, taking the beat of a quarter note as reference
 */
static inline int calculate_delay_ms(unsigned int note_len, unsigned int qnote_ref)
{
    unsigned char duration = (note_len & 0x7f);
    unsigned char triplet = (note_len & 0x80);
    unsigned char i = 0;
    unsigned char current_duration;
    int total = 0;

    /* Calculate the total duration of the note as the
     * summation of the figures that make up this note (bits 0-6)
     */
    while (duration)
    {
        current_duration = (duration) & (1 << i);

        if (current_duration)
        {
            // Scale note accordingly
            if (triplet)
                current_duration = (current_duration * 3) / 2;
            // 24000/qnote_ref denote number of ms associated with a whole note (redonda)
            total += (240000) / (qnote_ref * current_duration);
            duration &= ~(1 << i); // Clear bit
        }
        i++;
    }
    return total;
}

static void melody_work_function(struct work_struct *work)
{
    struct music_step melodic_line[] = {
        {C4, 4}, {C5, 4}, {C4, 4}, {0, 0}};
    const int beat = 300; // quarter notes per minute
    struct music_step *next;

    pwm_init_state(pwm_device, &pwm_state);

    // Play notes sequentially until end marker is found
    for (next = melodic_line; !is_end_marker(next); next++)
    {

        pwm_state.period = freq_to_period_ns(next->freq);

        // Disable temporarily to allow repeating the same consecutive notes in the melodic line
        pwm_disable(pwm_device);

        // If period==0, its a rest (silent note)
        if (pwm_state.period > 0)
        {
            // Set duty cycle to 70 to maintain the same timbre
            pwm_set_relative_duty_cycle(&pwm_state, 70, 100);
            pwm_state.enabled = true;
            pwm_apply_state(pwm_device, &pwm_state);
        }
        else
        {
            pwm_disable(pwm_device); // Disable for rest
        }

        // Wait for duration of the note or reset
        msleep(calculate_delay_ms(next->len, beat));
    }

    pwm_disable(pwm_device);
}

static int init_melody_module(void)
{
    pwm_device = pwm_request(0, PWM_DEVICE_NAME);

    if (IS_ERR(pwm_device))
        return PTR_ERR(pwm_device);

    INIT_WORK(&melody_work, melody_work_function);

    return 0;
}

static void free_melody_module(void)
{
    flush_work(&melody_work);
    pwm_free(pwm_device);
}

/******************************** button ********************************/

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
    Led_button_pair_t *led_button_pair;
    unsigned long flags;

#ifdef MANUAL_DEBOUNCE
    static unsigned long last_interrupt = 0;
    unsigned long diff = jiffies - last_interrupt;
    if (diff < 20)
        return IRQ_HANDLED;

    last_interrupt = jiffies;
#endif

    led_button_pair = (Led_button_pair_t *)dev_id;

    if (!led_button_pair)
    {
        return IRQ_HANDLED;
    }

    spin_lock_irqsave(&led_button_pair->device_lock, flags);

    led_button_pair->value = led_button_pair->value == 0 ? 1 : 0;
    gpiod_set_value(led_button_pair->led_descriptor, led_button_pair->value);

    spin_unlock_irqrestore(&led_button_pair->device_lock, flags);

    schedule_work(&melody_work);

    return IRQ_HANDLED;
}

static int install_button_irq(Led_button_pair_t *led_button_pair)
{
    if (!led_button_pair)
    {
        printk(KERN_WARNING "Error uninstall_button_irq : null led_button_pair\n");
        return -EINVAL;
    }

    if (led_button_pair->button_irqn != -1)
    {
        printk(KERN_WARNING "Error install_button_irq : irq has been previoulsy installed\n");
        return -EINVAL;
    }

    // Get the IRQ number for our GPIO
    led_button_pair->button_irqn = gpiod_to_irq(led_button_pair->button_descriptor);
    pr_info("IRQ Number = %d\n", led_button_pair->button_irqn);

    if (request_irq(led_button_pair->button_irqn, // IRQ number
                    button_irq_handler,           // IRQ handler
                    IRQF_TRIGGER_RISING,          // Handler will be called in raising edge
                    "button_irq",                 // used to identify the device name using this IRQ
                    led_button_pair))
    { // device id for shared IRQ
        pr_err("my_device: cannot register IRQ ");
        return -EINVAL;
    }
    return 0;
}

static void uninstall_button_irq(Led_button_pair_t *led_button_pair)
{
    if (!led_button_pair)
    {
        printk(KERN_WARNING "Error uninstall_button_irq : null led_button_pair\n");
        return;
    }

    if (led_button_pair->button_irqn == -1)
    {
        printk(KERN_WARNING "Error uninstall_button_irq : has been previously uninstalled\n");
        return;
    }
    free_irq(led_button_pair->button_irqn, led_button_pair);
    led_button_pair->button_irqn = -1;
}

/******************************** devices ********************************/
#define MAX_NAME_SIZE 64

struct device_data
{
    struct miscdevice *misc_device;
    Led_button_pair_t *led_button_pair;
    unsigned char is_open;
    char name[MAX_NAME_SIZE];
    struct kref kref; // reference counter
};

static struct device_data *search_device_data_in_list(const int minor);

static ssize_t device_write(struct file *file, const char __user *user_buf, size_t size_user_buf, loff_t *offset)
{
    char local_buf[2];
    struct device_data *ddata = file->private_data;
    unsigned long flags;

    if (!ddata)
        return -ENODEV;

    if (size_user_buf > sizeof(local_buf))
        return -EFAULT; // not enough space in local buffer

    if (copy_from_user(local_buf, user_buf, size_user_buf))
        return -EFAULT;

    // Parse command
    if (local_buf[0] == '0')
    {
        spin_lock_irqsave(&ddata->led_button_pair->device_lock, flags);
        ddata->led_button_pair->value = 0;
        gpiod_set_value(ddata->led_button_pair->led_descriptor, 0);
        spin_unlock_irqrestore(&ddata->led_button_pair->device_lock, flags);
    }
    else if (local_buf[0] == '1')
    {
        spin_lock_irqsave(&ddata->led_button_pair->device_lock, flags);
        ddata->led_button_pair->value = 1;
        gpiod_set_value(ddata->led_button_pair->led_descriptor, 1);
        spin_unlock_irqrestore(&ddata->led_button_pair->device_lock, flags);
    }
    else
    {
        printk(KERN_WARNING "Error de escritura: Unknown value\n");
        return -EINVAL;
    }

    return size_user_buf;
}

static ssize_t device_read(struct file *file, char __user *user_buf, size_t size_user_buf, loff_t *offset)
{
    char local_buf[1];
    int bytes_read = 0;
    struct device_data *ddata = file->private_data;
    unsigned long flags;

    if (!ddata)
        return -ENODEV;

    if (*offset > 0)
        return 0; // not more info to read (EOF)

    spin_lock_irqsave(&ddata->led_button_pair->device_lock, flags);
    local_buf[0] = '0' + ddata->led_button_pair->value;
    spin_unlock_irqrestore(&ddata->led_button_pair->device_lock, flags);

    bytes_read = 1;

    if (bytes_read > size_user_buf)
    {
        return -ENOSPC; // not enough space in user buf
    }

    if (copy_to_user(user_buf, local_buf, bytes_read))
        return -EFAULT; // copy_to_user error

    *offset += bytes_read; // advance pointer with bytes read

    return bytes_read;
}

static int device_open(struct inode *inode, struct file *file)
{
    const int minor = MINOR(inode->i_rdev);
    struct device_data *ddata = search_device_data_in_list(minor);

    if (!ddata)
        return -ENODEV; // No device data associated

    // Prevent multiple opens if needed
    if (ddata->is_open)
        return -EBUSY;

    ddata->is_open = 1;
    kref_get(&ddata->kref);
    file->private_data = ddata;

    return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
    struct device_data *ddata = file->private_data;

    if (!ddata)
        return -ENODEV;

    ddata->is_open = 0;
    kref_put(&ddata->kref, NULL); // decrement reference counter
    file->private_data = NULL;
    return 0;
}

static struct file_operations fops = {
    .owner = THIS_MODULE,
    .write = device_write,
    .read = device_read,
    .open = device_open,
    .release = device_release,
};

/******************************** linked list ********************************/

static int num_devices = 0;

static LIST_HEAD(mylist); // Nodo fantasma (cabecera) de la lista enlazada

struct list_item
{
    struct device_data data;
    struct list_head links;
};

/*
 *  return  0 if device exists
 *          1 if device not exists
 */
static int device_exists_in_list(const char *name)
{

    struct list_item *item;

    spin_lock(&list_lock);

    list_for_each_entry(item, &mylist, links)
    {
        if (strcmp(item->data.misc_device->name, name) == 0)
        {
            spin_unlock(&list_lock);
            return 1; // device found
        }
    }

    spin_unlock(&list_lock);

    return 0; // device not found
}

/*
 *  search the device by minor number and return the associated device_data
 *
 *  return null if not device found
 *          device_data* if devicefound
 */
static struct device_data *search_device_data_in_list(const int minor)
{
    struct list_item *item;

    spin_lock(&list_lock);

    list_for_each_entry(item, &mylist, links)
    {
        if (minor == item->data.misc_device->minor)
        {
            spin_unlock(&list_lock);
            return &(item->data); // device found
        }
    }
    spin_unlock(&list_lock);

    return 0; // device not found
}

/*
 *  return  0 if successful
 *         -1 if error list has reached max capacity
 *         -2 if error, device exists
 *         -3 if other error
 */
static int add_device_to_list(const char *name)
{
    struct list_item *new_item;
    Led_button_pair_t *led_button_pair;

    if (num_devices >= MAX_DEVICES)
        return -1; // list is full

    if (!name)
        return -3; // name is null

    if (device_exists_in_list(name))
        return -2; // device exists

    led_button_pair = get_led_button_pair_free();

    if (!led_button_pair)
        return -1; // list is full, all device are assigned

    new_item = kzalloc(sizeof(struct list_item), GFP_KERNEL);

    if (!new_item)
        return -3; // kzalloc failed

    // initializa misc device

    new_item->data.misc_device = kzalloc(sizeof(struct miscdevice), GFP_KERNEL);

    if (!(new_item->data.misc_device))
    {
        kfree(new_item);
        return -3; // kzalloc failed
    }

    new_item->data.misc_device->minor = MISC_DYNAMIC_MINOR;
    new_item->data.misc_device->fops = &fops;
    new_item->data.misc_device->mode = 0666;
    strncpy(new_item->data.name, name, MAX_NAME_SIZE);
    new_item->data.misc_device->name = new_item->data.name;
    new_item->data.is_open = 0;
    new_item->data.led_button_pair = led_button_pair;
    led_button_pair->assigned = 1;
    led_button_pair->value = 0;

    if (install_button_irq(led_button_pair))
    {
        kfree(new_item->data.misc_device);
        kfree(new_item);
        return -3;
    }

    if (misc_register(new_item->data.misc_device))
    {
        kfree(new_item->data.misc_device);
        kfree(new_item);
        return -3; // misc_register failed
    }

    dev_set_drvdata(new_item->data.misc_device->this_device, &(new_item->data));

    spin_lock_init(&new_item->data.led_button_pair->device_lock); // init spinlock of device

    kref_init(&new_item->data.kref); // init reference counter

    // add to list

    INIT_LIST_HEAD(&new_item->links);

    spin_lock(&list_lock);

    list_add_tail(&new_item->links, &mylist);

    num_devices++;

    spin_unlock(&list_lock);

    return 0;
}

/*
 *  return  0 if successful
 *         -1 if device not found
 */
static int delete_device_from_list(const char *name)
{
    struct list_item *item, *tmp;
    unsigned char device_found = -1;

    spin_lock(&list_lock);

    list_for_each_entry_safe(item, tmp, &mylist, links)
    {
        if (strcmp(item->data.misc_device->name, name) == 0)
        {
            if (!kref_get_unless_zero(&item->data.kref))
            {
                printk(KERN_WARNING "Device %s is still in use\n", name);
                spin_unlock(&list_lock);
                return -EBUSY;
            }

            item->data.led_button_pair->assigned = 0;
            uninstall_button_irq(item->data.led_button_pair);
            misc_deregister(item->data.misc_device);
            list_del(&(item->links));
            kfree(item->data.misc_device);
            kfree(item);
            num_devices--;
            device_found = 0;
            break;
        }
    }
    spin_unlock(&list_lock);

    return device_found;
}

static void cleanup_list(void)
{
    struct list_item *item, *tmp;

    spin_lock(&list_lock);

    list_for_each_entry_safe(item, tmp, &mylist, links)
    {
        item->data.led_button_pair->assigned = 0;
        uninstall_button_irq(item->data.led_button_pair);
        misc_deregister(item->data.misc_device);
        kref_put(&item->data.kref, NULL);
        list_del(&(item->links));
        kfree(item->data.misc_device);
        kfree(item);
    }
    num_devices = 0;

    spin_unlock(&list_lock);
}
/******************************** /proc ********************************/

#define PROC_FILE_NAME "modcontrol"

static ssize_t modcontrol_read(struct file *file, char __user *user_buf, size_t size_user_buf, loff_t *ppos)
{
    char local_buf[BUFFER_SIZE];
    int bytes_read = 0;
    struct list_item *item;

    if (*ppos > 0)
        return 0; // not more info to read (EOF)

    list_for_each_entry(item, &mylist, links)
    {
        bytes_read += snprintf(local_buf + bytes_read, sizeof(local_buf) - bytes_read, "%s ", item->data.misc_device->name);

        if (bytes_read > BUFFER_SIZE)
        {
            return -ENOSPC; // not enough space in local buffer
        }
    }

    if (bytes_read > size_user_buf)
    {
        return -ENOSPC; // not enough space in user buf
    }

    if (copy_to_user(user_buf, local_buf, bytes_read))
        return -EFAULT; // copy_to_user error

    *ppos += bytes_read; // advance pointer with bytes read

    return bytes_read;
}

static ssize_t modcontrol_write(struct file *file, const char __user *user_buf, size_t size_user_buf, loff_t *ppos)
{
    char local_buf[MAX_NAME_SIZE];
    char name[MAX_NAME_SIZE] = {0};
    int add_return_value;

    if (size_user_buf > sizeof(local_buf) - 1)
        return -EFAULT; // not enough space in local buffer

    if (copy_from_user(local_buf, user_buf, size_user_buf))
        return -EFAULT;

    local_buf[size_user_buf] = '\0'; // null terminate the string

    // Parse command
    if (sscanf(local_buf, "new %s", name) == 1)
    {
        add_return_value = add_device_to_list(name);
        if (add_return_value == -1)
        {
            printk(KERN_WARNING "Error de escritura: maximo numero de devices alcanzado\n");
            return -EPERM;
        }
        else if (add_return_value == -2)
        {
            printk(KERN_WARNING "Error de escritura: el device ya existe\n");
            return -EPERM;
        }
        else if (add_return_value != 0)
        {
            printk(KERN_WARNING "Error de escritura: add device ha fallado\n");
            return -EPERM;
        }
    }
    else if (sscanf(local_buf, "delete %s", name) == 1)
    {
        if (delete_device_from_list(name) != 0)
        {
            printk(KERN_WARNING "Error de escritura: device no existe\n");
            return -EPERM;
        }
    }
    else
    {
        printk(KERN_WARNING "Error de escritura: Unknown command\n");
        return -EINVAL;
    }

    return size_user_buf;
}

static const struct proc_ops pops = {
    .proc_read = modcontrol_read,
    .proc_write = modcontrol_write,
};

/******************************** multiledsw ********************************/

int multiledsw_init(void)
{
    int err = 0;

    if ((err = init_melody_module()))
    {
        printk(KERN_WARNING "Error init_melody_module\n");
        return err;
    }

    if ((err = initialize_gpios()))
    {
        printk(KERN_WARNING "Error initialize_gpios\n");
        return err;
    }

    proc_create(PROC_FILE_NAME, 0666, NULL, &pops);

    add_device_to_list("ledsw_def");

    return 0;
}

void multiledsw_exit(void)
{
    remove_proc_entry(PROC_FILE_NAME, NULL);
    cleanup_list();
    free_gpios();
    free_melody_module();
}

module_init(multiledsw_init);
module_exit(multiledsw_exit);