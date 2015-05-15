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
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

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
	AD7781(int bus, const char* path, spi_dev_e device, enum Rotation rotation);
	virtual ~AD7781();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	// print register dump
	void			print_registers();

	// trigger an error
	void			test_error();

protected:
	virtual int		probe();

private:

	struct hrt_call		_call;
	unsigned		_call_interval;

	RingBuffer		*_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;
	int			_orb_class_instance;
	int			_class_instance;

	unsigned		_current_rate;
	unsigned		_current_bandwidth;
	unsigned		_orientation;

	unsigned		_read;

	perf_counter_t		_sample_perf;
	perf_counter_t		_reschedules;
	perf_counter_t		_errors;
	perf_counter_t		_bad_registers;

	uint8_t			_register_wait;

	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	/* true if an L3G4200D is detected */
	bool	_is_l3g4200d;

	enum Rotation		_rotation;

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
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_bus == EXTERNAL_BUS); }

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

	/**
	 * Write a register in the AD7781
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the AD7781
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();

	/* this class does not allow copying */
	AD7781(const AD7781&);
	AD7781 operator=(const AD7781&);
};


AD7781::AD7781(int bus, const char* path, spi_dev_e device, enum Rotation rotation) :
	SPI("AD7781", path, bus, device, SPIDEV_MODE3, 11*1000*1000 /* will be rounded to 10.4 MHz, within margins for AD7781 */),
	_call{},
	_call_interval(0),
	_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_gyro_topic(-1),
	_orb_class_instance(-1),
	_class_instance(-1),
	_current_rate(0),
	_current_bandwidth(50),
	_orientation(SENSOR_BOARD_ROTATION_DEFAULT),
	_read(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "l3gd20_read")),
	_reschedules(perf_alloc(PC_COUNT, "l3gd20_reschedules")),
	_errors(perf_alloc(PC_COUNT, "l3gd20_errors")),
	_bad_registers(perf_alloc(PC_COUNT, "l3gd20_bad_registers")),
	_register_wait(0),
	_gyro_filter_x(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_filter_y(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(L3GD20_DEFAULT_RATE, L3GD20_DEFAULT_FILTER_FREQ),
	_is_l3g4200d(false),
	_rotation(rotation),
	_checked_next(0)
{
	// enable debug() calls
	_debug_enabled = true;

	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_L3GD20;

	// default scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;
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
		&_orb_class_instance, (is_external()) ? ORB_PRIO_VERY_HIGH : ORB_PRIO_DEFAULT);

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

	bool success = false;
	uint8_t v = 0;

	/* verify that the device is attached and functioning, accept
	 * AD7781, */
	if ((v=read_reg(ADDR_WHO_AM_I)) == WHO_I_AM) {
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;
	} else if ((v=read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_H) {
		_orientation = SENSOR_BOARD_ROTATION_180_DEG;
		success = true;
	} else if ((v=read_reg(ADDR_WHO_AM_I)) == WHO_I_AM_L3G4200D) {
		/* Detect the L3G4200D used on AeroCore */
		_is_l3g4200d = true;
		_orientation = SENSOR_BOARD_ROTATION_DEFAULT;
		success = true;
	}

	if (success) {
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
				if (_is_l3g4200d) {
					return ioctl(filp, SENSORIOCSPOLLRATE, L3G4200D_DEFAULT_RATE);
				}
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

					/* adjust filters */
					float cutoff_freq_hz = _gyro_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f/ticks;
					set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

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

	case SENSORIOCRESET:
		reset();
		return OK;

	case GYROIOCSSAMPLERATE:
		return set_samplerate(arg, _current_bandwidth);

	case GYROIOCGSAMPLERATE:
		return _current_rate;

	case GYROIOCSLOWPASS: {
		// set the software lowpass cut-off in Hz
		float cutoff_freq_hz = arg;
		float sample_rate = 1.0e6f / _call_interval;
		set_driver_lowpass_filter(sample_rate, cutoff_freq_hz);

		return OK;
	}

	case GYROIOCGLOWPASS:
		return _gyro_filter_x.get_cutoff_freq();

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/* arg should be in dps */
		return set_range(arg);

	case GYROIOCGRANGE:
		/* convert to dps and round */
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return self_test();

	case GYROIOCSHWLOWPASS:
		return set_samplerate(_current_rate, arg);

	case GYROIOCGHWLOWPASS:
		return _current_bandwidth;

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
AD7781::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
AD7781::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
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
	// ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();

	/* set default configuration */
	write_checked_reg(ADDR_CTRL_REG1,
                          REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
	write_checked_reg(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
	write_checked_reg(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
	write_checked_reg(ADDR_CTRL_REG4, REG4_BDU);
	write_checked_reg(ADDR_CTRL_REG5, 0);
	write_checked_reg(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	write_checked_reg(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);

	set_samplerate(0, _current_bandwidth); // 760Hz or 800Hz
	set_range(AD7781_DEFAULT_RANGE_DPS);
	set_driver_lowpass_filter(AD7781_DEFAULT_RATE, AD7781_DEFAULT_FILTER_FREQ);

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

	switch (_orientation) {

		case SENSOR_BOARD_ROTATION_000_DEG:
			/* keep axes in place */
			report.x_raw = raw_report.x;
			report.y_raw = raw_report.y;
			break;

		case SENSOR_BOARD_ROTATION_090_DEG:
			/* swap x and y */
			report.x_raw = raw_report.y;
			report.y_raw = raw_report.x;
			break;

		case SENSOR_BOARD_ROTATION_180_DEG:
			/* swap x and y and negate both */
			report.x_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			report.y_raw = ((raw_report.y == -32768) ? 32767 : -raw_report.y);
			break;

		case SENSOR_BOARD_ROTATION_270_DEG:
			/* swap x and y and negate y */
			report.x_raw = raw_report.y;
			report.y_raw = ((raw_report.x == -32768) ? 32767 : -raw_report.x);
			break;
	}

	report.z_raw = raw_report.z;

	report.temperature_raw = raw_report.temp;

	report.x = ((report.x_raw * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	report.y = ((report.y_raw * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	report.z = ((report.z_raw * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	report.x = _gyro_filter_x.apply(report.x);
	report.y = _gyro_filter_y.apply(report.y);
	report.z = _gyro_filter_z.apply(report.z);

	report.temperature = AD7781_TEMP_OFFSET_CELSIUS - raw_report.temp;

	// apply user specified rotation
	rotate_3f(_rotation, report.x, report.y, report.z);

	report.scaling = _gyro_range_scale;
	report.range_rad_s = _gyro_range_rad_s;

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

void
AD7781::print_registers()
{
	printf("AD7781 registers\n");
	for (uint8_t reg=0; reg<=0x40; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ",(unsigned)reg, (unsigned)v);
		if ((reg+1) % 16 == 0) {
			printf("\n");
		}
	}
	printf("\n");
}

int
AD7781::self_test()
{
	/* evaluate gyro offsets, complain if offset -> zero or larger than 6 dps */
	if (fabsf(_gyro_scale.x_offset) > 0.1f || fabsf(_gyro_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.y_offset) > 0.1f || fabsf(_gyro_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.z_offset) > 0.1f || fabsf(_gyro_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f)
		return 1;

	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace ad7781
{

AD7781	*g_dev;

void	usage();
void	start(bool external_bus, enum Rotation rotation);
void	test();
void	reset();
void	info();
void	regdump();
void	test_error();

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * started or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr)
		errx(0, "already started");

	/* create the driver */
        if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		g_dev = new AD7781(PX4_SPI_BUS_EXT, AD7781_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_EXT_GYRO, rotation);
#else
		errx(0, "External SPI not available");
#endif
	} else {
		g_dev = new AD7781(PX4_SPI_BUS_SENSORS, AD7781_DEVICE_PATH, (spi_dev_e)PX4_SPIDEV_GYRO, rotation);
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
 * Reset the driver.
 */
void
reset()
{
	int fd = open(AD7781_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "accel pollrate reset failed");

        close(fd);

	exit(0);
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

/**
 * Dump the register information
 */
void
regdump(void)
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("regdump @ %p\n", g_dev);
	g_dev->print_registers();

	exit(0);
}

/**
 * trigger an error
 */
void
test_error(void)
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("regdump @ %p\n", g_dev);
	g_dev->test_error();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'testerror' or 'regdump'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
ad7781_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
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
		ad7781::start(external_bus, rotation);

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
		ad7781::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset"))
		ad7781::reset();

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
