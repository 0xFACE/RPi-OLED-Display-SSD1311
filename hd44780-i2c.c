/** \file server/drivers/hd44780-i2c.c
 * \c i2c connection type of \c hd44780 driver for Hitachi HD44780 based LCD displays.
 *
 * The LCD is operated in its 4 bit-mode to be connected to the 8 bit-port
 * of a single PCF8574(A) or PCA9554(A) that is accessed by the server via the i2c bus.
 */

/* Copyright (c) 2005 Matthias Goebl <matthias.goebl@goebl.net>
 *		  2000, 1999, 1995 Benjamin Tse <blt@Comports.com>
 *		  2001 Joris Robijn <joris@robijn.net>
 *		  1999 Andrew McMeikan <andrewm@engineer.com>
 *		  1998 Richard Rognlie <rrognlie@gamerz.net>
 *		  1997 Matthias Prinke <m.prinke@trashcan.mcnet.de>
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

#include "hd44780-i2c.h"
#include "hd44780-low.h"

#include "report.h"
#ifdef HAVE_CONFIG_H
# include "config.h"
#endif
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#ifdef HAVE_DEV_IICBUS_IIC_H
#include <dev/iicbus/iic.h>
#else /* HAVE_LINUX_I2C_DEV_H */
#include <linux/i2c-dev.h>
/* I2C_SLAVE is missing in linux/i2c-dev.h from kernel headers of 2.4.x kernels */
#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703  /* Change slave address */
#endif
#endif


// Generally, any function that accesses the LCD control lines needs to be
// implemented separately for each HW design. This is typically (but not
// restricted to):
// HD44780_senddata
// HD44780_readkeypad

void i2c_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch);
void i2c_HD44780_backlight(PrivateData *p, unsigned char state);
void i2c_HD44780_close(PrivateData *p);

#define RS	0x10
#define RW	0x20
#define EN	0x40
#define BL	0x80
// note that the above bits are all meant for the data port of PCF8574

#define I2C_ADDR_MASK 0x7f
#define I2C_PCAX_MASK 0x80

static void
i2c_out(PrivateData *p, unsigned char val[2])
{
//	char data[2];
	int datalen;
	static int no_more_errormsgs=0;
#ifdef HAVE_DEV_IICBUS_IIC_H
	struct iiccmd cmd;
	bzero(&cmd, sizeof(cmd));
#endif

//	if (p->port & I2C_PCAX_MASK) { // we have a PCA9554 or similar, that needs a 2-byte command
//		data[0]=1; // command: read/write output port register
//		data[1]=val;
		datalen=2;
//	} else { // we have a PCF8574 or similar, that needs a 1-byte command
//		data[0]=val;
//		datalen=1;
//	}

#ifdef HAVE_DEV_IICBUS_IIC_H
	cmd.slave = (p->port & I2C_ADDR_MASK) << 1;
	cmd.last = 1;
	cmd.count = datalen;
	cmd.buf = val;

	if (ioctl(p->fd, I2CWRITE, &cmd) < 0) {
#else /* HAVE_LINUX_I2C_DEV_H */
	if (write(p->fd,val,datalen) != datalen) {
#endif
		p->hd44780_functions->drv_report(no_more_errormsgs?RPT_DEBUG:RPT_ERR, "HD44780: I2C: i2c write data %u to address 0x%02X failed: %s",
			val, p->port & I2C_ADDR_MASK, strerror(errno));
		no_more_errormsgs=1;
	}
}

#define DEFAULT_DEVICE		"/dev/i2c-0"


/**
 * Initialize the driver.
 * \param drvthis  Pointer to driver structure.
 * \retval 0       Success.
 * \retval -1      Error.
 */
int
hd_init_i2c(Driver *drvthis)
{
	PrivateData *p = (PrivateData*) drvthis->private_data;
	HD44780_functions *hd44780_functions = p->hd44780_functions;

	int enableLines = EN;
	char device[256] = DEFAULT_DEVICE;
#ifdef HAVE_DEV_IICBUS_IIC_H
	struct iiccmd cmd;
	bzero(&cmd, sizeof(cmd));
#endif

//###??	p->backlight_bit = BL;
	p->backlight_bit = BL;

	/* READ CONFIG FILE */

	/* Get serial device to use */
	strncpy(device, drvthis->config_get_string(drvthis->name, "Device", 0, DEFAULT_DEVICE), sizeof(device));
	device[sizeof(device)-1] = '\0';
	report(RPT_INFO,"HD44780: I2C: Using device '%s' and address 0x%02X for a %s",
		device, p->port & I2C_ADDR_MASK, (p->port & I2C_PCAX_MASK) ? "PCA9554(A)" : "PCF8574(A)");

	// Open the I2C device
	p->fd = open(device, O_RDWR);
	if (p->fd < 0) {
		report(RPT_ERR, "HD44780: I2C: open i2c device '%s' failed: %s", device, strerror(errno));
		return(-1);
	}

	// Set I2C address
#ifdef HAVE_DEV_IICBUS_IIC_H
	cmd.slave = (p->port & I2C_ADDR_MASK) << 1;
	cmd.last = 0;
	cmd.count = 0;
	if (ioctl(p->fd, I2CRSTCARD, &cmd) < 0) {
		report(RPT_ERR, "HD44780: I2C: reset bus failed: %s", strerror(errno));
		return -1;
	}
	if (ioctl(p->fd, I2CSTART, &cmd) < 0) {
		report(RPT_ERR, "HD44780: I2C: set address to 0x%02X: %s", p->port & I2C_ADDR_MASK, strerror(errno));
		return -1;
	}
#else /* HAVE_LINUX_I2C_DEV_H */
	if (ioctl(p->fd,I2C_SLAVE, p->port & I2C_ADDR_MASK) < 0) {
		report(RPT_ERR, "HD44780: I2C: set address to '%i': %s", p->port & I2C_ADDR_MASK, strerror(errno));
		return(-1);
	}
#endif
	hd44780_functions->senddata = i2c_HD44780_senddata;
	hd44780_functions->backlight = i2c_HD44780_backlight;
	hd44780_functions->close = i2c_HD44780_close;
	/* Display init */

	unsigned char data[2];
	data[0] = 0x80;
	i2c_out(p, data);
	
	data[1] = 0x2A;	 // **** Set "RE"=1<--->00101010B
	i2c_out(p, data);
	data[1] = 0x71; 	//# Function Selection A  [71h] (IS = X, RE = 1, SD=0;, 2bajty
	i2c_out(p, data);
	data[1] = 0x5C;  // 0x5C set Vdd
	i2c_out(p, data);
	data[1] = 0x28; 
	i2c_out(p, data);
    
	data[1] = 0x08; 	// **** Set Sleep Mode On
	i2c_out(p, data);
	data[1] = 0x2A; 	// **** Set "RE"=1	00101010B
	i2c_out(p, data);
	data[1] = 0x79; 	// **** Set "SD"=1	01111001B
	i2c_out(p, data);

	data[1] = 0xD5;  //# Set Display Clock Divide Ratio/ Oscillator Frequency (D5h
	i2c_out(p, data);
	data[1] = 0x70; 
	i2c_out(p, data);
	data[1] = 0x78; 	// **** Set "SD"=0
	i2c_out(p, data);

	data[1] = 0x08; 	// **** Set 5-dot, 3 or 4 line(0x09;, 1 or 2 line(0x08;
	i2c_out(p, data);

	data[1] = 0x06; 	// **** Set Com31-->Com0  Seg0-->Seg99
	i2c_out(p, data);

	//**** Set OLED Characterization
	data[1] = 0x2A;   	// **** Set "RE"=1 
	i2c_out(p, data);
	data[1] = 0x79;   	// **** Set "SD"=1
	i2c_out(p, data);

	// **** CGROM/CGRAM Management *** //
	data[1] = 0x72;   	// **** Set ROM
	i2c_out(p, data);
	data[1] = 0x00;   	// **** Set ROM A and 8 CGRAM
	i2c_out(p, data);

	data[1] = 0xDA;  	// **** Set Seg Pins HW Config
	i2c_out(p, data);
	data[1] = 0x10;    
	i2c_out(p, data);

	//data[1] = 0x81;   	// **** Set Contrast
	//i2c_out(p, data);
	//data[1] = 0x50;
	//i2c_out(p, data);

	data[1] = 0xDB;   	// **** Set VCOM deselect level
	i2c_out(p, data);
	data[1] = 0x30;   	// **** VCC x 0.83
	i2c_out(p, data);
	
	data[1] = 0xDC;   	// **** Set gpio - turn EN for 15V generator on.
	i2c_out(p, data);
	data[1] = 0x03; 
	i2c_out(p, data);
	
	data[1] = 0x78;   	// **** Exiting Set OLED Characterization
	i2c_out(p, data);
	data[1] = 0x28;  
	i2c_out(p, data);
	data[1] = 0x2A;  
	i2c_out(p, data);
	data[1] = 0x6;  	// **** Set Entry Mode
	i2c_out(p, data);
	data[1] = 0x08;   
	i2c_out(p, data);
	data[1] = 0x28;  	// **** Set "IS"=0 , "RE" =0 //28
	i2c_out(p, data);
	i2c_out(p, data);
	data[1] = 0x01;  
	i2c_out(p, data);
	data[1] = 0x80;  	// **** Set DDRAM Address to 0x80 (line 1 start;
	i2c_out(p, data);

	//sleep 0.1s;
	data[1] = 0x0C;   	// **** Turn on Display
	i2c_out(p, data);

	return 0;
}

void
i2c_HD44780_close(PrivateData *p) {
	if (p->fd >= 0) {
#ifdef HAVE_DEV_IICBUS_IIC_H
		ioctl(p->fd, I2CSTOP);
#endif
		close(p->fd);
	}
}


/**
 * Send data or commands to the display.
 * \param p          Pointer to driver's private data structure.
 * \param displayID  ID of the display (or 0 for all) to send data to.
 * \param flags      Defines whether to end a command or data.
 * \param ch         The value to send.
 */
void
i2c_HD44780_senddata(PrivateData *p, unsigned char displayID, unsigned char flags, unsigned char ch)
{
	unsigned char data[2];
	if (flags == RS_INSTR)
		data[0] = 0x80;
	else
		data[0] = 0x40;
	data[1]  = ch;
	i2c_out(p, data);
}


/**
 * Turn display backlight on or off.
 * \param p      Pointer to driver's private data structure.
 * \param state  New backlight status.
 */
void i2c_HD44780_backlight(PrivateData *p, unsigned char state)
{

//	printf("Backlight %d",state);
	unsigned char data[2];

	data[0] = 0x80;
	data[1] = 0x2A;
	i2c_out(p, data);

//	data[0] = 0x80;
	data[1] = 0x79;
	i2c_out(p, data);

//	data[0] = 0x80;
	data[1] = 0x81;
	i2c_out(p, data);

//	data[0] = 0x80;
	if (state)
	    data[1] = 240;
	else
	    data[1] = 50;
	i2c_out(p, data);

//	data[0] = 0x80;
	data[1] = 0x78;
	i2c_out(p, data);

//	data[0] = 0x80;
	data[1] = 0x28;
	i2c_out(p, data);

}
