/*
 * ADS1298 Linux kernel driver (SPI + GPIO)
 * 8-channel 24-bit ADC; 27-byte frames on DRDY.
 * Exposes raw packets via /dev/ads1298.
 *
 * Compatible: "ti,ads1298"
 * DT: under SPI controller; cs-gpios, drdy-gpios, start-gpios, reset-gpios.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kernel.h>

#define ADS1298_PACKET_SIZE   27
#define ADS1298_FIFO_PACKETS  512
#define ADS1298_FIFO_SIZE     (ADS1298_FIFO_PACKETS * ADS1298_PACKET_SIZE)

/* Commands */
#define ADS1298_WAKEUP   0x02
#define ADS1298_STANDBY  0x04
#define ADS1298_RESET    0x06
#define ADS1298_START    0x08
#define ADS1298_STOP     0x0A
#define ADS1298_RDATAC   0x10
#define ADS1298_SDATAC   0x11
#define ADS1298_RDATA    0x12
#define ADS1298_RREG     0x20
#define ADS1298_WREG     0x40

#define ADS1298_REG_DEVID    0x00
#define ADS1298_REG_CONFIG1  0x01
#define ADS1298_REG_CONFIG2  0x02
#define ADS1298_REG_CONFIG3  0x03
#define ADS1298_REG_CH1SET   0x05

struct ads1298_dev {
	struct spi_device *spi;
	struct gpio_desc *drdy;
	struct gpio_desc *start;
	struct gpio_desc *reset;
	struct miscdevice misc;
	DECLARE_KFIFO(fifo, u8, ADS1298_FIFO_SIZE);
	wait_queue_head_t wait;
	spinlock_t fifo_lock;
	bool running;
};

static struct ads1298_dev *ads1298_device;

static void ads1298_cs_assert(struct ads1298_dev *dev)
{
	/* CS is handled by SPI core via cs-gpios */
}

static void ads1298_cs_deassert(struct ads1298_dev *dev)
{
	/* CS deassert + 4 tCLK delay done by SPI core with transfer delay */
}

static int ads1298_spi_read_packet(struct ads1298_dev *dev, u8 *buf)
{
	struct spi_transfer t = {
		.rx_buf     = buf,
		.len        = ADS1298_PACKET_SIZE,
		.delay = {
			.unit = SPI_DELAY_UNIT_USECS,
			.value = 2,
		},
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(dev->spi, &m);
}

static int ads1298_send_cmd(struct ads1298_dev *dev, u8 cmd)
{
	return spi_write(dev->spi, &cmd, 1);
}

static int ads1298_reg_write(struct ads1298_dev *dev, u8 addr, u8 val)
{
	u8 tx[3] = { addr | ADS1298_WREG, 0x00, val };
	return spi_write(dev->spi, tx, sizeof(tx));
}

static int ads1298_reg_read(struct ads1298_dev *dev, u8 addr, u8 *val)
{
	u8 tx[3] = { addr | ADS1298_RREG, 0x00, 0x00 };
	u8 rx[3];
	int ret;

	ret = spi_write_then_read(dev->spi, tx, 3, rx, 3);
	if (ret == 0)
		*val = rx[2];
	return ret;
}

static void ads1298_hw_reset(struct ads1298_dev *dev)
{
	if (!dev->reset)
		return;
	gpiod_set_value_cansleep(dev->reset, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(dev->reset, 0);
	usleep_range(2, 10);
	gpiod_set_value_cansleep(dev->reset, 1);
	usleep_range(10, 50);
}

static void ads1298_start_conversion(struct ads1298_dev *dev)
{
	if (dev->start)
		gpiod_set_value_cansleep(dev->start, 1);
	usleep_range(10, 20);
	ads1298_send_cmd(dev, ADS1298_START);
}

static void ads1298_stop_conversion(struct ads1298_dev *dev)
{
	ads1298_send_cmd(dev, ADS1298_STOP);
	if (dev->start)
		gpiod_set_value_cansleep(dev->start, 0);
}

static irqreturn_t ads1298_drdy_threaded(int irq, void *data)
{
	struct ads1298_dev *dev = data;
	u8 packet[ADS1298_PACKET_SIZE];
	unsigned int pushed;
	unsigned long flags;

	if (!dev->running)
		return IRQ_HANDLED;

	if (ads1298_spi_read_packet(dev, packet) != 0)
		return IRQ_HANDLED;

	spin_lock_irqsave(&dev->fifo_lock, flags);
	pushed = kfifo_in(&dev->fifo, packet, ADS1298_PACKET_SIZE);
	spin_unlock_irqrestore(&dev->fifo_lock, flags);

	if (pushed == ADS1298_PACKET_SIZE)
		wake_up_interruptible(&dev->wait);

	return IRQ_HANDLED;
}

static int ads1298_open(struct inode *inode, struct file *filp)
{
	struct ads1298_dev *dev = container_of(filp->private_data,
					      struct ads1298_dev, misc);
	filp->private_data = dev;
	return 0;
}

static ssize_t ads1298_read(struct file *filp, char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct ads1298_dev *dev = filp->private_data;
	u8 packet[ADS1298_PACKET_SIZE];
	unsigned int copied;
	unsigned long flags;
	size_t total = 0;

	if (count < ADS1298_PACKET_SIZE)
		return -EINVAL;

	while (total + ADS1298_PACKET_SIZE <= count) {
		if (wait_event_interruptible(dev->wait,
				!kfifo_is_empty(&dev->fifo) || !dev->running))
			return total ? (ssize_t)total : -ERESTARTSYS;
		if (!dev->running)
			break;
		spin_lock_irqsave(&dev->fifo_lock, flags);
		copied = kfifo_out(&dev->fifo, packet, ADS1298_PACKET_SIZE);
		spin_unlock_irqrestore(&dev->fifo_lock, flags);
		if (copied != ADS1298_PACKET_SIZE)
			break;
		if (copy_to_user(buf + total, packet, ADS1298_PACKET_SIZE))
			return total ? (ssize_t)total : -EFAULT;
		total += ADS1298_PACKET_SIZE;
	}
	return total;
}

static __poll_t ads1298_poll(struct file *filp, poll_table *wait)
{
	struct ads1298_dev *dev = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &dev->wait, wait);
	if (!kfifo_is_empty(&dev->fifo))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations ads1298_fops = {
	.owner   = THIS_MODULE,
	.open    = ads1298_open,
	.read    = ads1298_read,
	.poll    = ads1298_poll,
};

static int ads1298_init_device(struct ads1298_dev *dev)
{
	u8 id;
	int i;

	ads1298_send_cmd(dev, ADS1298_SDATAC);
	usleep_range(2000, 5000);

	if (ads1298_reg_read(dev, ADS1298_REG_DEVID, &id) != 0)
		return -EIO;
	if (id != 0x92)
		dev_warn(&dev->spi->dev, "ADS1298 ID 0x%02X (expected 0x92)\n", id);

	ads1298_reg_write(dev, ADS1298_REG_CONFIG1, 0xA4);
	ads1298_reg_write(dev, ADS1298_REG_CONFIG2, 0x31);
	ads1298_reg_write(dev, ADS1298_REG_CONFIG3, 0xCC);
	ads1298_reg_write(dev, 0x0D, 0xFF);
	ads1298_reg_write(dev, 0x0E, 0xFF);
	ads1298_reg_write(dev, 0x15, 0x01);
	ads1298_reg_write(dev, 0x18, 0x09);
	ads1298_reg_write(dev, 0x19, 0xD0);
	ads1298_reg_write(dev, 0x04, 0x07);
	ads1298_reg_write(dev, 0x17, 0x02);
	ads1298_reg_write(dev, 0x0F, 0xFF);
	ads1298_reg_write(dev, 0x10, 0xFF);
	for (i = 0; i < 8; i++)
		ads1298_reg_write(dev, ADS1298_REG_CH1SET + i, 0x00);

	ads1298_send_cmd(dev, ADS1298_RDATAC);
	ads1298_start_conversion(dev);
	return 0;
}

static int ads1298_probe(struct spi_device *spi)
{
	struct ads1298_dev *dev;
	int irq, ret;

	dev = devm_kzalloc(&spi->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->spi = spi;
	spi_set_drvdata(spi, dev);

	dev->drdy = devm_gpiod_get_optional(&spi->dev, "drdy", GPIOD_IN);
	if (IS_ERR(dev->drdy))
		return PTR_ERR(dev->drdy);

	dev->start = devm_gpiod_get_optional(&spi->dev, "start", GPIOD_OUT_LOW);
	dev->reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_HIGH);

	INIT_KFIFO(dev->fifo);
	init_wait_queue_head(&dev->wait);
	spin_lock_init(&dev->fifo_lock);

	dev->misc.minor = MISC_DYNAMIC_MINOR;
	dev->misc.name = "ads1298";
	dev->misc.fops = &ads1298_fops;
	ret = misc_register(&dev->misc);
	if (ret)
		return ret;

	ads1298_hw_reset(dev);
	usleep_range(10000, 20000);

	ret = ads1298_init_device(dev);
	if (ret) {
		misc_deregister(&dev->misc);
		return ret;
	}

	dev->running = true;

	if (dev->drdy) {
		irq = gpiod_to_irq(dev->drdy);
		if (irq >= 0) {
			ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
							ads1298_drdy_threaded,
							IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
							"ads1298-drdy", dev);
			if (ret)
				dev_warn(&spi->dev, "DRDY IRQ request failed: %d\n", ret);
		}
	}

	ads1298_device = dev;
	dev_info(&spi->dev, "ADS1298 probed, /dev/ads1298 ready\n");
	return 0;
}

static void ads1298_remove(struct spi_device *spi)
{
	struct ads1298_dev *dev = spi_get_drvdata(spi);

	ads1298_device = NULL;
	dev->running = false;
	wake_up_interruptible(&dev->wait);
	ads1298_stop_conversion(dev);
	ads1298_send_cmd(dev, ADS1298_SDATAC);
	misc_deregister(&dev->misc);
}

static const struct of_device_id ads1298_of_match[] = {
	{ .compatible = "ti,ads1298" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads1298_of_match);

static const struct spi_device_id ads1298_id[] = {
	{ "ads1298", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads1298_id);

static struct spi_driver ads1298_driver = {
	.driver = {
		.name = "ads1298",
		.of_match_table = ads1298_of_match,
	},
	.probe  = ads1298_probe,
	.remove = ads1298_remove,
	.id_table = ads1298_id,
};

module_spi_driver(ads1298_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("ADS1298 driver");
MODULE_DESCRIPTION("Texas Instruments ADS1298 SPI + GPIO kernel driver");
