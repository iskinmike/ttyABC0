/*
 *	simple UART driver on port 0x3f8 with IRQ 4
 *
 *	
 */


#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/kfifo.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/serial_reg.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/reboot.h>

#include "common.h"

#undef pr_fmt
#define pr_fmt(fmt)    "%s.c:%d %s " fmt, KBUILD_MODNAME, __LINE__, __func__

#define TTYABC0_DELAY_TIME 5*HZ

static struct timer_list uptime_timer;

static int port = 0x3f8;
module_param(port, int, 0);
MODULE_PARM_DESC(port, "io port number, default 0x3f8 - UART");

static int port_size = 8;
module_param(port_size, int, 0);
MODULE_PARM_DESC(port_size, "number of io ports, default 8");

static int irq = 4;
module_param(irq, int, 0);
MODULE_PARM_DESC(irq, "interrupt request number, default 4 - UART");

static int loopback;
module_param(loopback, int, 0);
MODULE_PARM_DESC(loopback, "loopback mode for testing, default 0");

#define FIFO_SIZE 128		/* must be power of two */

static int bufsize = 8 * PAGE_SIZE;

/**
 * struct ttyabc0_data - the driver data
 * @in_buf:	input buffer for mmap interface
 * @out_buf:	outoput buffer for mmap interface
 * @in_fifo:	input queue for write
 * @out_fifo:	output queue for read
 * @fifo_lock:	lock for queues
 * @readable:	waitqueue for blocking read
 * @writeable:	waitqueue for blocking write
 * @port_ptr:	mapped io port
 * @uart_detected: UART is detected and will be used.
 *	Otherwise emulation mode will be used.
 *
 * stored in static global variable drvdata for simplicity.
 * Can be also retrieved from platform_device with
 * struct ttyabc0_data *drvdata = platform_get_drvdata(pdev);
 */

struct ttyabc0_data {
	void *in_buf;
	void *out_buf;
	DECLARE_KFIFO(in_fifo, char, FIFO_SIZE);
	DECLARE_KFIFO(out_fifo, char, FIFO_SIZE);
	spinlock_t fifo_lock;
	wait_queue_head_t readable, writeable;
	struct mutex read_lock;
	struct mutex write_lock;
	void __iomem *port_ptr;
	int uart_detected;
};

static struct ttyabc0_data *drvdata;

/**
 * ttyabc0_received_buffer	- puts data to receive queue
 * @data: received data
 * @count: number of bytes of data
 */

static void ttyabc0_received_buffer(char* data, int count)
{
	kfifo_in_spinlocked(&drvdata->in_fifo, data,
			sizeof(data)*count, &drvdata->fifo_lock);
	wake_up_interruptible(&drvdata->readable);
}

static inline u8 tx_ready(void)
{
	return ioread8(drvdata->port_ptr + UART_LSR) & UART_LSR_THRE;
}

static inline u8 rx_ready(void)
{
	return ioread8(drvdata->port_ptr + UART_LSR) & UART_LSR_DR;
}

/*
 *	interrupt section
 */

static int isr_counter;

static irqreturn_t ttyabc0_isr(int irq, void *dev_id)
{
	/*
	 *      UART interrupt is not fired in loopback mode,
	 *      therefore fire ttyabc0_tasklet from timer too
	 */
	isr_counter++;
	pr_debug("UART_FCR=0x%02X\n", ioread8(drvdata->port_ptr + UART_FCR));
	pr_debug("UART_IIR=0x%02X\n", ioread8(drvdata->port_ptr + UART_IIR));
	return IRQ_HANDLED;	/* our IRQ */
}



/*
 *	timer section
 */

static char* getUptime(void) {
	struct timespec uptime;
	char *buffer;
	int i;
	const int buffer_len = 35;

	get_monotonic_boottime(&uptime);

	buffer = (char *) kmalloc(sizeof(char)*buffer_len, GFP_KERNEL);
	for (i = 0; i < buffer_len; ++i) {
		buffer[i] = 0x00;
	}

	sprintf(buffer, "%lu.%02lu", (unsigned long) uptime.tv_sec, (uptime.tv_nsec / (NSEC_PER_SEC / 100)));
	return buffer;
}

static void sendDataByTimer(unsigned long timer_data){
	char* uptime_str = NULL;
	uptime_str = getUptime();
	ttyabc0_received_buffer(uptime_str, strlen(uptime_str));
	uptime_timer.expires = jiffies + TTYABC0_DELAY_TIME;
	mod_timer(&uptime_timer, jiffies + TTYABC0_DELAY_TIME);
}

static DEFINE_TIMER(uptime_timer, sendDataByTimer, 0, 0);

/*
 *	file_operations section
 */

static int ttyabc0_open(struct inode *inode, struct file *file)
{
	pr_debug("from %s\n", current->comm);
	/* client related data can be allocated here and
	   stored in file->private_data */
	return 0;
}
static int ttyabc0_release(struct inode *inode, struct file *file)
{
	pr_debug("from %s\n", current->comm);
	/* client related data can be retrived from file->private_data
	   and released here */
	return 0;
}

static ssize_t ttyabc0_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	int ret = 0;
	unsigned int copied;
	char* uptime_str = getUptime();

	pr_debug("+ from %s\n", current->comm);
	if (kfifo_is_empty(&drvdata->in_fifo)) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			pr_debug("+ waiting\n");
			ret = wait_event_interruptible(drvdata->readable,
					!kfifo_is_empty(&drvdata->in_fifo));

			if (ret == -ERESTARTSYS) {
				pr_err("%s\n", "+ interrupted");
				return -EINTR;
			}
		}
	}
	if (mutex_lock_interruptible(&drvdata->read_lock))
		return -EINTR;
	ret = kfifo_to_user(&drvdata->in_fifo, buf, count, &copied);
	mutex_unlock(&drvdata->read_lock);
	kfree(uptime_str);
	return ret ? ret : copied;
}

static ssize_t ttyabc0_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	/* REASTART */
	//wake_up_interruptible(&drvdata->readable);
	kernel_restart(NULL);
	return 0;
}

static unsigned int ttyabc0_poll(struct file *file, poll_table *pt)
{
	unsigned int mask = 0;
	poll_wait(file, &drvdata->readable, pt);
	poll_wait(file, &drvdata->writeable, pt);

	if (!kfifo_is_empty(&drvdata->in_fifo))
		mask |= POLLIN | POLLRDNORM;
	mask |= POLLOUT | POLLWRNORM;
	return mask;
}


/*
 *	pages_flag - set or clear a flag for sequence of pages
 *
 *	more generic solution instead SetPageReserved, ClearPageReserved etc
 *
 *	Poposing to move pages_flag to linux/page-flags.h
 */

static void pages_flag(struct page *page, int page_num, int mask, int value)
{
	for (; page_num; page_num--, page++)
		if (value)
			__set_bit(mask, &page->flags);
		else
			__clear_bit(mask, &page->flags);
}

static const struct file_operations ttyabc0_fops = {
	.owner	= THIS_MODULE,
	.open	= ttyabc0_open,
	.release = ttyabc0_release,
	.read	= ttyabc0_read,
	.write	= ttyabc0_write,
	.poll = ttyabc0_poll,
};

static struct miscdevice ttyabc0_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= KBUILD_MODNAME,
	.fops	= &ttyabc0_fops,
};

/*
 *	UART initialization section
 */

static struct resource *port_r;

static int uart_probe(void)
{
	int ret = 0;

	if (port) {
		drvdata->port_ptr = ioport_map(port, port_size);
		pr_debug("drvdata->port_ptr=%p\n", drvdata->port_ptr);
		if (!drvdata->port_ptr) {
			pr_err("%s\n", "ioport_map failed");
			return -ENODEV;
		}
	}
	if (!irq || !drvdata->port_ptr)
		goto exit;
	/*
	 *	Minimal configuration of UART for trivial I/O opertaions
	 *	and ISR just to porform basic tests.
	 *	Some configuration of UART is not touched and reused.
	 *
	 *	This minimal configiration of UART is based on
	 *	full UART driver drivers/tty/serial/8250/8250.c
	 */
	ret = request_irq(irq, ttyabc0_isr,
			IRQF_SHARED, KBUILD_MODNAME, THIS_MODULE);
	if (ret < 0) {
		pr_err("%s\n", "request_irq failed");
		return ret;
	}
	iowrite8(UART_MCR_RTS | UART_MCR_OUT2 | UART_MCR_LOOP,
			drvdata->port_ptr + UART_MCR);
	drvdata->uart_detected = (ioread8(drvdata->port_ptr + UART_MSR) & 0xF0)
		== (UART_MSR_DCD | UART_MSR_CTS);

	if (drvdata->uart_detected) {
		iowrite8(UART_IER_RDI | UART_IER_RLSI | UART_IER_THRI,
				drvdata->port_ptr + UART_IER);
		iowrite8(UART_MCR_DTR | UART_MCR_RTS | UART_MCR_OUT2,
				drvdata->port_ptr + UART_MCR);
		iowrite8(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT,
				drvdata->port_ptr + UART_FCR);
		pr_debug("loopback=%d\n", loopback);
		if (loopback)
			iowrite8(ioread8(drvdata->port_ptr + UART_MCR) | UART_MCR_LOOP,
					drvdata->port_ptr + UART_MCR);
	}
	if (!drvdata->uart_detected && loopback)
		pr_warn("Emulating loopback in software\n");
exit:
	return ret;
}

/*
 *	main initialization and cleanup section
 */

static struct dentry *debugfs;

static void ttyabc0_cleanup(void)
{
	debugfs_remove(debugfs);
	if (ttyabc0_miscdev.this_device)
		misc_deregister(&ttyabc0_miscdev);

	del_timer(&uptime_timer);
	if (irq) {
		if (drvdata->uart_detected) {
			iowrite8(0, drvdata->port_ptr + UART_IER);
			iowrite8(0, drvdata->port_ptr + UART_FCR);
			iowrite8(0, drvdata->port_ptr + UART_MCR);
			ioread8(drvdata->port_ptr + UART_RX);
		}
		free_irq(irq, THIS_MODULE);
	}
	if (drvdata->in_buf) {
		pages_flag(virt_to_page(drvdata->in_buf), PFN_UP(bufsize), PG_reserved, 0);
		free_pages_exact(drvdata->in_buf, bufsize);
	}
	if (drvdata->out_buf) {
		pages_flag(virt_to_page(drvdata->out_buf), PFN_UP(bufsize), PG_reserved, 0);
		free_pages_exact(drvdata->out_buf, bufsize);
	}

	pr_debug("isr_counter=%d\n", isr_counter);
	if (drvdata->port_ptr)
		ioport_unmap(drvdata->port_ptr);
	if (port_r)
		release_region(port, port_size);
	kfree(drvdata);
}

static struct ttyabc0_data *ttyabc0_data_init(void)
{
	struct ttyabc0_data *drvdata;

	drvdata = kzalloc(sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return NULL;
	init_waitqueue_head(&drvdata->readable);
	init_waitqueue_head(&drvdata->writeable);
	INIT_KFIFO(drvdata->in_fifo);
	INIT_KFIFO(drvdata->out_fifo);
	mutex_init(&drvdata->read_lock);
	mutex_init(&drvdata->write_lock);
	return drvdata;
}

static __devinit int ttyabc0_init(void)
{
	int ret = 0;

	pr_debug("MODNAME=%s\n", KBUILD_MODNAME);
	pr_debug("port = %d irq = %d\n", port, irq);

	drvdata = ttyabc0_data_init();
	if (!drvdata) {
		pr_err("ttyabc0_data_init failed\n");
		goto exit;
	}
	// Start uptime timer
	mod_timer(&uptime_timer, jiffies + TTYABC0_DELAY_TIME);

	/*
	 *	Allocating buffers and pinning them to RAM
	 *	to be mapped to user space in ttyabc0_mmap
	 */
	drvdata->in_buf = alloc_pages_exact(bufsize, GFP_KERNEL | __GFP_ZERO);
	if (!drvdata->in_buf) {
		ret = -ENOMEM;
		goto exit;
	}
	pages_flag(virt_to_page(drvdata->in_buf), PFN_UP(bufsize), PG_reserved, 1);
	drvdata->out_buf = alloc_pages_exact(bufsize, GFP_KERNEL | __GFP_ZERO);
	if (!drvdata->out_buf) {
		ret = -ENOMEM;
		goto exit;
	}
	pages_flag(virt_to_page(drvdata->out_buf), PFN_UP(bufsize), PG_reserved, 1);
	isr_counter = 0;
	/*
	 *	This drivers without UART can be sill used
	 *	in emulation mode for testing and demonstation of work
	 */
	ret = uart_probe();
	if (ret < 0) {
		pr_err("uart_probe failed\n");
		goto exit;
	}

	debugfs = debugfs_create_file(KBUILD_MODNAME, S_IRUGO, NULL, NULL, &ttyabc0_fops);
	if (IS_ERR(debugfs)) {
		ret = PTR_ERR(debugfs);
		pr_err("debugfs_create_file failed\n");
		goto exit;
	}
	ret = misc_register(&ttyabc0_miscdev);
	if (ret < 0) {
		pr_err("misc_register failed\n");
		goto exit;
	}
	pr_debug("ttyabc0_miscdev.minor=%d\n", ttyabc0_miscdev.minor);

exit:
	pr_debug("ret=%d\n", ret);
	if (ret < 0)
		ttyabc0_cleanup();

	return ret;
}

module_init(ttyabc0_init);
module_exit(ttyabc0_cleanup);

MODULE_DESCRIPTION("Test Uptime Driver");
MODULE_AUTHOR("Mikhail Myasnikov <myasnikov.mike@gmail.com>");
MODULE_LICENSE("GPL");