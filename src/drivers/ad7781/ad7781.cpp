/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ad7781.cpp
 * Driver for the Analog Devices AD7781 20bit ADC connected via SPI.
 *
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/spi.h>
#include <drivers/drv_gyro.h>
#include <drivers/device/ringbuffer.h>

#include <board_config.h>

#define AD7781_DEVICE_PATH "/dev/ad7781"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;


#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

extern "C" { __EXPORT int ad7781_main(int argc, char *argv[]); }

class AD7781 : public device::SPI
{
public:
	AD7781(int bus, const char* path, spi_dev_e device);
	virtual ~AD7781();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();


protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;

	RingBuffer		*_reports;

	orb_advert_t		_gyro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	unsigned		_current_rate;
	unsigned		_current_bandwidth;

	unsigned		_read;

	perf_counter_t		_sample_perf;
	perf_counter_t		_reschedules;
	perf_counter_t		_errors;
	perf_counter_t		_bad_registers;

	uint8_t			_register_wait;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset the driver
	 */
	void			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Read a register from the AD7781
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/* this class does not allow copying */
	AD7781(const AD7781&);
	AD7781 operator=(const AD7781&);
};


AD7781::AD7781(int bus, const char* path, spi_dev_e device) :
	SPI("AD7781", path, bus, device, SPIDEV_MODE3, 11*1000*1000 /* will be rounded to 10.4 MHz, within margins for AD7781 */),
	_call{},
	_call_interval(0),
	_reports(nullptr),
	_gyro_topic(-1),
	_orb_class_instance(-1),
	_class_instance(-1),
	_current_rate(0),
	_current_bandwidth(50),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "l3gd20_read")),
	_reschedules(perf_alloc(PC_COUNT, "l3gd20_reschedules")),
	_errors(perf_alloc(PC_COUNT, "l3gd20_errors")),
	_bad_registers(perf_alloc(PC_COUNT, "l3gd20_bad_registers")),
	_register_wait(0),
	_checked_next(0)
{
	// enable debug() calls
	_debug_enabled = true;

	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_L3GD20;
}

AD7781::~AD7781()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;

	if (_class_instance != -1)
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _class_instance);

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_reschedules);
	perf_free(_errors);
	perf_free(_bad_registers);
}

int
AD7781::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(gyro_report));

	if (_reports == nullptr)
		goto out;

	_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	reset();

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_reports->get(&grp);

	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
		&_orb_class_instance, ORB_PRIO_VERY_HIGH);

	if (_gyro_topic < 0) {
		debug("failed to create sensor_gyro publication");
	}

	ret = OK;
out:
	return ret;
}

int
AD7781::probe()
{
	/* read dummy value to void to clear SPI statemachine on sensor */
	(void)read_reg(ADDR_WHO_AM_I);

	/* verify that the device is attached and functioning, accept
	 * AD7781, */
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {
		return OK;
	}

	return -EIO;
}

ssize_t
AD7781::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct gyro_report);
	struct gyro_report *gbuf = reinterpret_cast<struct gyro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the measurement code while we are doing this;
		 * we are careful to avoid racing with it.
		 */
		while (count--) {
			if (_reports->get(gbuf)) {
				ret += sizeof(*gbuf);
				gbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_reports->flush();
	measure();

	/* measurement will have generated a report, copy it out */
	if (_reports->get(gbuf)) {
		ret = sizeof(*gbuf);
	}

	return ret;
}

int
AD7781::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, AD7781_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
AD7781::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
AD7781::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&AD7781::measure_trampoline, this);
}

void
AD7781::stop()
{
	hrt_cancel(&_call);
}

void
AD7781::reset()
{
	_read = 0;
}

void
AD7781::measure_trampoline(void *arg)
{
	AD7781 *dev = (AD7781 *)arg;

	/* make another measurement */
	dev->measure();
}

#ifdef GPIO_EXTI_GYRO_DRDY
# define AD7781_USE_DRDY 1
#else
# define AD7781_USE_DRDY 0
#endif

void
AD7781::measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		int8_t		temp;
		uint8_t		status;
		int16_t		x;
		int16_t		y;
		int16_t		z;
	} raw_report;
#pragma pack(pop)

	gyro_report report;

	/* start the performance counter */
	perf_begin(_sample_perf);

        check_registers();

#if AD7781_USE_DRDY
	// if the gyro doesn't have any data ready then re-schedule
	// for 100 microseconds later. This ensures we don't double
	// read a value and then miss the next value
	if (_bus == PX4_SPI_BUS_SENSORS && stm32_gpioread(GPIO_EXTI_GYRO_DRDY) == 0) {
		perf_count(_reschedules);
		hrt_call_delay(&_call, 100);
                perf_end(_sample_perf);
		return;
	}
#endif

	/* fetch data from the sensor */
	memset(&raw_report, 0, sizeof(raw_report));
	raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
	transfer((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

#if AD7781_USE_DRDY
        if (_bus == PX4_SPI_BUS_SENSORS && (raw_report.status & 0xF) != 0xF) {
            /*
              we waited for DRDY, but did not see DRDY on all axes
              when we captured. That means a transfer error of some sort
             */
            perf_count(_errors);
            return;
        }
#endif
	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */
	report.timestamp = hrt_absolute_time();
        report.error_count = perf_event_count(_bad_registers);

	report.x_raw = raw_report.x;
	report.y_raw = raw_report.y;
	report.z_raw = raw_report.z;

	report.temperature_raw = raw_report.temp;

	report.x = report.x_raw;
	report.y = report.y_raw;
	report.z = report.z_raw;

	report.temperature = AD7781_TEMP_OFFSET_CELSIUS - raw_report.temp;

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/* publish for subscribers */
	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &report);
	}

	_read++;

	/* stop the perf counter */
	perf_end(_sample_perf);
}

void
AD7781::print_info()
{
	printf("gyro reads:          %u\n", _read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_reschedules);
	perf_print_counter(_errors);
	perf_print_counter(_bad_registers);
	_reports->print_info("report queue");
        ::printf("checked_next: %u\n", _checked_next);
        for (uint8_t i=0; i<AD7781_NUM_CHECKED_REGISTERS; i++) {
            uint8_t v = read_reg(_checked_registers[i]);
            if (v != _checked_values[i]) {
                ::printf("reg %02x:%02x should be %02x\n",
                         (unsigned)_checked_registers[i],
                         (unsigned)v,
                         (unsigned)_checked_values[i]);
            }
        }
}

/**
 * Local functions in support of the shell command.
 */
namespace ad7781
{

AD7781	*g_dev;

void	usage();
void	start(bool external_bus);
void	test();
void	info();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus)
{
	int fd;

	if (g_dev != nullptr)
		errx(0, "already started");

	/* create the driver */
        if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		g_dev = new AD7781(PX4_SPI_BUS_EXT, AD7781_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_EXT_GYRO);
#else
		errx(0, "External SPI not available");
#endif
	} else {
		g_dev = new AD7781(PX4_SPI_BUS_SENSORS, AD7781_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_GYRO);
	}

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(AD7781_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

        close(fd);

	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	int fd_gyro = -1;
	struct gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	fd_gyro = open(AD7781_DEVICE_PATH, O_RDONLY);

	if (fd_gyro < 0)
		err(1, "%s open failed", AD7781_DEVICE_PATH);

	/* reset to manual polling */
	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report))
		err(1, "immediate gyro read failed");

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("temp: \t%d\tC", (int)g_report.temperature);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("temp: \t%d\traw", (int)g_report.temperature_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "reset to default polling");

        close(fd_gyro);

	/* XXX add poll-rate tests here too */
	errx(0, "PASS");
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running\n");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
	warnx("options:");
}

} // namespace

int
ad7781_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;
		default:
			ad7781::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start"))
		ad7781::start(external_bus);

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
		ad7781::test();

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info"))
		ad7781::info();

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump"))
		ad7781::regdump();

	/*
	 * trigger an error
	 */
	if (!strcmp(verb, "testerror"))
		ad7781::test_error();

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'testerror' or 'regdump'");
}
