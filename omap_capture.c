/*
 * OMAP 35xx Capture module driver
 *
 * Copyright (C) 2010 iRobot Corporation
 * Frank Agius <fgius@irobot.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/*
 * This driver can set up a pin on the OMAP 35xx processor as an input
 * capture source. The original purpose was to measure the frequency of
 * a signal that corresponded to the RPM of a motor.
 * Only 4 pins on the 35XX can be used  for capture: 144,
 * 145, 146 and 147.  The driver assumes that the capture
 * pin is in  "Mode 2" (pwm_evt).  The driver default is to set up
 * pin 147 for capture.  To set up one of the other pins as capture,
 * invoke the driver with "capture_pin=xxx", where "xxx"
 * is one of 144, 145, 146 and 147. Pulse width (rising edge to falling
 * edge) and frequency measurements can be made on a signal connected
 * to the capture pin. The driver continuously captures width and
 * frequency counts.  If there is no input on the pin, or the input goes
 * away, the driver will report 0 for pulse width and frequency counts.
 * The ioctl interface will provide an integer count for pulse width,
 * frequency and clock source.  To determine the values in milliseconds,
 * divide the returned count of width or frequency by the clock source
 * count and multiply by 1000.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <plat/dmtimer.h>
#include <mach/hardware.h>
#include <plat/mux.h>

#define _OMAP_TIMER_STAT_OFFSET         0x18
#define _OMAP_TIMER_INT_EN_OFFSET       0x1c
#define _OMAP_TIMER_CTRL_OFFSET         0x24
#define _OMAP_TIMER_CAPTURE_OFFSET      0x3c
#define _OMAP_TIMER_IF_CTRL_OFFSET      0x40
#define _OMAP_TIMER_CAPTURE2_OFFSET     0x44    /* TCAR2, 34xx only */

#define OMAP_TIMER_CTRL_REG     (_OMAP_TIMER_CTRL_OFFSET | (WP_TCLR << WPSHIFT))
#define WP_TCLR                 (1 << 0)
/* register offsets with the write pending bit encoded */
#define WPSHIFT                 16
#define _OMAP_TIMER_WRITE_PEND_OFFSET   0x34
#define         WP_NONE         0       /* no write pending bit */
#define OMAP_TIMER_STAT_REG             (_OMAP_TIMER_STAT_OFFSET \
                                                | (WP_NONE << WPSHIFT))
#define OMAP_TIMER_WRITE_PEND_REG       (_OMAP_TIMER_WRITE_PEND_OFFSET \
                                                 | (WP_NONE << WPSHIFT))
#define OMAP_TIMER_INT_EN_REG           (_OMAP_TIMER_INT_EN_OFFSET \
                                                 | (WP_NONE << WPSHIFT))
#define OMAP_TIMER_CAPTURE_REG          (_OMAP_TIMER_CAPTURE_OFFSET \
                                                  | (WP_NONE << WPSHIFT))
#define OMAP_TIMER_IF_CTRL_REG          (_OMAP_TIMER_IF_CTRL_OFFSET \
                                                 | (WP_NONE << WPSHIFT))
#define OMAP_TIMER_CAPTURE2_REG         (_OMAP_TIMER_CAPTURE2_OFFSET \
                                                        | (WP_NONE << WPSHIFT))
#define OMAP_TIMER_CTRL_GPOCFG          (1 << 14)
#define OMAP_TIMER_CTRL_CAPTMODE        (1 << 13)
#define OMAP_TIMER_CTRL_PT              (1 << 12)
#define OMAP_TIMER_CTRL_TRG_OVERFLOW    (0x1 << 10)
#define OMAP_TIMER_CTRL_TRG_MATCH       (0x2 << 10)
#define OMAP_TIMER_CTRL_TCM_LOWTOHIGH   (0x1 << 8)
#define OMAP_TIMER_CTRL_TCM_HIGHTOLOW   (0x2 << 8)
#define OMAP_TIMER_CTRL_TCM_BOTHEDGES   (0x3 << 8)
#define OMAP_TIMER_CTRL_SCPWM           (1 << 7)
#define OMAP_TIMER_CTRL_CE              (1 << 6) /* compare enable */
#define OMAP_TIMER_CTRL_PRE             (1 << 5) /* prescaler enable */
#define OMAP_TIMER_CTRL_PTV_SHIFT       2 /* prescaler value shift */
#define OMAP_TIMER_CTRL_POSTED          (1 << 2)
#define OMAP_TIMER_CTRL_AR              (1 << 1) /* auto-reload enable */
#define OMAP_TIMER_CTRL_ST              (1 << 0) /* start timer */
#define OMAP34XX_PADCONF_START	        0x48002030
#define OMAP34XX_PADCONF_SIZE	        0x05cc
#define PREFIX "OMAP_CAPTURE: "
#define CAPTURE_IOC_MAGIC ';'
#define CAPTURE_IOCX_READ          _IO(CAPTURE_IOC_MAGIC, 0)


/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 */
#define IEN     (1 << 8)
#define IDIS    (0 << 8)
#define PTU     (1 << 4)
#define PTD     (0 << 4)
#define EN      (1 << 3)
#define DIS     (0 << 3)

// default pin for capture
static int  capture_pin=147;
module_param(capture_pin, int, 0644);
MODULE_PARM_DESC(capture_pin, "The pin number to be used as a capture input.  Must be one of 144,145,146,147. Default is 147");
static int setup_pulse_width_capture(void);
static int setup_frequency_capture(void);
static ssize_t omap_capture_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp);
// the IRQ # for our gp timer
static int32_t timer_irq;
static inline u32 omap_dm_timer_read_reg(struct omap_dm_timer *timer, u32 reg);
static void omap_dm_timer_write_reg(struct omap_dm_timer *timer, u32 reg, u32 value);
struct mux_dev {
  dev_t devt;
  struct semaphore sem;
  struct class *class;
};

struct omap_capture_data {
  struct device		*dev;
  struct mutex		lock;
  struct work_struct	ws;
  struct omap_dm_timer *timer;
  unsigned long clk_rate;
  unsigned long pulse_width;
  unsigned long frequency;
  unsigned long last_capture_time;
  unsigned long last_check_time;
  unsigned long overflow;
};

struct omap_capture_user_parms {
  int channel;
  int average;
  int status;
  unsigned long clk_rate;
  unsigned long pulse_width;
  unsigned long frequency;
};

static struct omap_capture_data *the_capture;

struct omap_dm_timer {
  unsigned long phys_base;
  int irq;
#ifdef CONFIG_ARCH_OMAP2PLUS
  struct clk *iclk, *fclk;
#endif
  void __iomem *io_base;
  unsigned reserved:1;
  unsigned enabled:1;
  unsigned posted:1;
};

static irqreturn_t omap_capture_irq_handler(int irq, void *_capture)
{
  unsigned long l, edge_1, edge_2, width;
  l = omap_dm_timer_read_reg(the_capture->timer, OMAP_TIMER_STAT_REG);
  if (l == OMAP_TIMER_INT_OVERFLOW)
  {
    // this is an interrupt caused by timer overflow.
    // disable interrupts
    omap_dm_timer_set_int_enable(the_capture->timer, 0);
    // stop the timer!
    omap_dm_timer_stop(the_capture->timer);
    // clear the overflow interrupt
    omap_dm_timer_write_status(the_capture->timer, ( OMAP_TIMER_INT_OVERFLOW));
    if (the_capture->last_capture_time == the_capture->last_check_time)
    {
      // we have not received a capture interupt since the last time we
      // received a timer overflow interrupt. the pulse we're measuring
      // is no longer there.  zero out the capture data
      the_capture->frequency = 0;
      the_capture->pulse_width = 0;
      printk(KERN_DEBUG PREFIX "Pulse not detected%lx\n", the_capture->last_capture_time);
    }
    else
    {
      // save the last capture time as the time we made the last check.
      // the next time we get an timer expired interrupt, and the values are still equal,
      // the pulses must have stopped.
      the_capture->last_check_time = the_capture->last_capture_time;
      printk(KERN_DEBUG PREFIX "Pulse detected %lx %lx\n", the_capture->last_capture_time, the_capture->last_check_time);
    }

    omap_dm_timer_start(the_capture->timer);
    // enable interrupts
    omap_dm_timer_set_int_enable(the_capture->timer, OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW);
    return IRQ_HANDLED;
  }

  // disable int
  omap_dm_timer_set_int_enable(the_capture->timer, 0);
  // stop the timer!
  omap_dm_timer_stop(the_capture->timer);

  // clear the interrupts
  omap_dm_timer_write_status(the_capture->timer, ( OMAP_TIMER_INT_CAPTURE | OMAP_TIMER_INT_MATCH| OMAP_TIMER_INT_OVERFLOW));
  l = omap_dm_timer_read_status(the_capture->timer); // YES, you really need to do this 'wasteful' read
  edge_1 = omap_dm_timer_read_reg(the_capture->timer, OMAP_TIMER_CAPTURE_REG);
  edge_2 = omap_dm_timer_read_reg(the_capture->timer, OMAP_TIMER_CAPTURE2_REG);

  if (edge_1 <= edge_2)
  {
     // normal case
     width = edge_2 - edge_1;
  }
  else
  {
     // timer wrap
     width = ((edge_2 - the_capture->overflow) + (0xFFFFFFFF - edge_1));
  }

  l = omap_dm_timer_read_reg(the_capture->timer, OMAP_TIMER_CTRL_REG);
  if ((l&OMAP_TIMER_CTRL_TCM_BOTHEDGES) == OMAP_TIMER_CTRL_TCM_BOTHEDGES)
  {
    // just completed a pulse measurement
    the_capture->pulse_width = width;
    // set up the next capture for a frequency measurment
    setup_frequency_capture();
  }
  else
  {
    // just completed a frequency measurement
    the_capture->frequency = width;
    // set up the next capture for a pulse measurement
    setup_pulse_width_capture();
  }

  // save the time of this capture
  the_capture->last_capture_time = omap_dm_timer_read_counter(the_capture->timer);

  // start the timer!
  omap_dm_timer_start(the_capture->timer);
  // enable int
  omap_dm_timer_set_int_enable(the_capture->timer, OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW);

  return IRQ_HANDLED;
}

/*
 * Reads timer registers
 */
static inline u32 omap_dm_timer_read_reg(struct omap_dm_timer *timer, u32 reg)
{
  if (timer->posted)
    while (readl(timer->io_base + (OMAP_TIMER_WRITE_PEND_REG & 0xff))
                                & (reg >> WPSHIFT))
                        cpu_relax();
  return readl(timer->io_base + (reg & 0xff));
}

/*
 * Write timer registers
 */
static void omap_dm_timer_write_reg(struct omap_dm_timer *timer, u32 reg,
                                                u32 value)
{
  if (timer->posted)
  while (readl(timer->io_base + (OMAP_TIMER_WRITE_PEND_REG & 0xff))
                                & (reg >> WPSHIFT))
                        cpu_relax();
  writel(value, timer->io_base + (reg & 0xff));
}

/*
 * ioctl interface for reads
 */

static long omap_capture_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
  struct omap_capture_user_parms par;
  int ret;

  ret = copy_from_user(&par, (void __user *) arg, sizeof(par));
  if (ret)
  {
    dev_dbg(the_capture->dev, "copy_from_user: %d\n", ret);
    return -EACCES;
  }

  switch (cmd)
  {
    case CAPTURE_IOCX_READ:
    {
      par.clk_rate = the_capture->clk_rate;
      par.pulse_width = the_capture->pulse_width;
      par.frequency = the_capture->frequency;
    }
    break;
    default:
      printk(KERN_ERR PREFIX "Unknown ioctl: %d\n",cmd);
      return -EINVAL;
  }

  ret = copy_to_user((void __user *) arg, &par, sizeof(par));
  if (ret)
  {
    printk(KERN_DEBUG PREFIX "copy_to_user: %d\n", ret);
    return -EACCES;
  }

  return 0;
}

/*
 * ioctl interface for debug
 */
static ssize_t omap_capture_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
  size_t status = count;
  printk(KERN_ALERT PREFIX "pulse_width = 0x%lx frequency = 0x%lx clk_rate = 0x%lx\n", the_capture->pulse_width, the_capture->frequency, the_capture->clk_rate);

  return status;
}

static struct file_operations omap_capture_fileops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = omap_capture_ioctl,
  .write = omap_capture_write,
};

static struct miscdevice omap_capture_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = "omap-capture",
  .fops = &omap_capture_fileops
};

static int setup_pulse_width_capture(void)
{
  unsigned long l;
  // set up capture for two rising edges
  l = OMAP_TIMER_CTRL_GPOCFG |\
      OMAP_TIMER_CTRL_CAPTMODE |\
      OMAP_TIMER_CTRL_PT|\
      OMAP_TIMER_CTRL_TRG_MATCH|\
      OMAP_TIMER_CTRL_TCM_BOTHEDGES|\
      OMAP_TIMER_CTRL_CE|\
      OMAP_TIMER_CTRL_AR;

  omap_dm_timer_write_reg(the_capture->timer, OMAP_TIMER_CTRL_REG, l);

  return 0;
}

static int setup_frequency_capture(void)
{
  unsigned long l;
  // set up capture for a rising edges
  l = OMAP_TIMER_CTRL_GPOCFG |\
      OMAP_TIMER_CTRL_CAPTMODE |\
      OMAP_TIMER_CTRL_PT|\
      OMAP_TIMER_CTRL_TRG_MATCH|\
      OMAP_TIMER_CTRL_TCM_LOWTOHIGH|\
      OMAP_TIMER_CTRL_CE|\
      OMAP_TIMER_CTRL_AR;

  omap_dm_timer_write_reg(the_capture->timer, OMAP_TIMER_CTRL_REG, l);

  return 0;

}

static int __init omap_capture_init(void)
{
  struct omap_capture_data *capture;
  int ret;
  int capture_timer=8;
  unsigned int reg;
  uint32_t clk_rate;
  void __iomem *base;
  struct clk *gt_fclk;

  // validate the capture pin. only pins 144,145, 146 and 147 can be capture
  // input pins.
  switch (capture_pin)
  {
    case 144:
      capture_timer=9;
      break;
    case 145:
      capture_timer=10;
      break;
    case 146:
      capture_timer=11;
      break;
    case 147:
      capture_timer=8;
      break;
    default:
      printk(KERN_ERR PREFIX "Invalid capture pin specified: %d\n", capture_pin);
      printk(KERN_ERR PREFIX "Valid pin numbers are 144, 145, 146 and 147\n");
      return -1;
      break;

  }

  printk(KERN_ERR PREFIX "Pin %d setup for capture\n", capture_pin);

  capture = kzalloc(sizeof *capture, GFP_KERNEL);
  if (!capture)
  {
    printk(KERN_ERR PREFIX "No free memory available\n");
    return -ENOMEM;
  }

  the_capture = capture;

  ret = misc_register(&omap_capture_device);
  if (ret)
  {
    printk(KERN_ERR PREFIX "Could not register the device\n");
    goto err_misc;
  }

  // get a pointer to the timer
  capture->timer = omap_dm_timer_request_specific(capture_timer);
  if(capture->timer == NULL)
  {
    //  no timers available
    printk(KERN_ERR PREFIX "gp timer not available, bailing out\n");
    ret = -1;
    goto err_misc;
  }

  // set the clock source to system clock
  omap_dm_timer_set_source(capture->timer, OMAP_TIMER_SRC_SYS_CLK);

  // stop the timer
  omap_dm_timer_stop(capture->timer);

  // determine the timers clock rate
  gt_fclk = omap_dm_timer_get_fclk(capture->timer);
  clk_rate = clk_get_rate(gt_fclk);
  printk(KERN_DEBUG PREFIX "Clock Rate 0x%x\n", clk_rate);

  // save the clock rate
  capture->clk_rate = clk_rate;
  // calculate the value to overflow every 1 second.
  capture->overflow = 0xffffffff - (1 * clk_rate);

  // set the overflow value, with auto reload turned on
  omap_dm_timer_set_load(capture->timer, 1, capture->overflow);

  // set the value of timer counter to 0xffffffff, with auto reload off
  omap_dm_timer_set_load_start(capture->timer,0,0xffffffff);

  // figure out what IRQ our timer triggers
  timer_irq = omap_dm_timer_get_irq(capture->timer);

  // install our IRQ handler for our timer
  ret = request_irq(timer_irq, omap_capture_irq_handler, IRQF_DISABLED | IRQF_TIMER , "capture_timer", omap_capture_irq_handler);

  if (ret)
  {
    printk(KERN_ERR PREFIX "Capture: request_irq failed (on irq %d), bailing out\n", timer_irq);
    goto err_irq;
  }

  capture->pulse_width = 0;
  capture->frequency = 0;
  capture->last_capture_time = 0;
  capture->last_check_time = 0;

  mutex_init(&capture->lock);
  // setup timer to trigger our IRQ on the capture event or an overflow
  // event.  overflow allows checking of the last capture, to determine
  // if capture events have ended, so that we can zero the width and
  // pulse counts.
  omap_dm_timer_set_int_enable(capture->timer, OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW);
  // setup for a frequency capture
  setup_frequency_capture();
  // start the timer!
  omap_dm_timer_start(capture->timer);

  return 0;

err_irq:
err_misc:
  kfree(capture);

  return ret;
}

module_init(omap_capture_init);

static void __exit omap_capture_exit(void)
{
  omap_dm_timer_free(the_capture->timer);
  free_irq(timer_irq, omap_capture_irq_handler);
  misc_deregister(&omap_capture_device);
}

module_exit(omap_capture_exit);

MODULE_ALIAS("omap-capture");
MODULE_AUTHOR("iRobot Corporation");
MODULE_DESCRIPTION("OMAP capture driver");
MODULE_LICENSE("GPL");

