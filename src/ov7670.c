
#include "ov7670.h"
#include "camera.h"
#include "delay.h"

static const struct regval_list vga_ov7670[] = {
	{REG_HREF,0xF6},	// was B6  
	{0x17,0x13},		// HSTART
	{0x18,0x01},		// HSTOP
	{0x19,0x02},		// VSTART
	{0x1a,0x7a},		// VSTOP
	{REG_VREF,0x0a},	// VREF
	{0xff, 0xff},		/* END MARKER */
};
static const struct regval_list qvga_ov7670[] = {
	{REG_COM14, 0x19},
	{0x72, 0x11},
	{0x73, 0xf1},
	{REG_HSTART,0x16},
	{REG_HSTOP,0x04},
	{REG_HREF,0x24},
	{REG_VSTART,0x02},
	{REG_VSTOP,0x7a},
	{REG_VREF,0x0a},
	{0xff, 0xff},	/* END MARKER */
};
static const struct regval_list qqvga_ov7670[] = {
	{REG_COM14, 0x1a},	// divide by 4
	{0x72, 0x22},		// downsample by 4
	{0x73, 0xf2},		// divide by 4
	{REG_HSTART,0x16},
	{REG_HSTOP,0x04},
	{REG_HREF,0xa4},		   
	{REG_VSTART,0x02},
	{REG_VSTOP,0x7a},
	{REG_VREF,0x0a},
	{0xff, 0xff},	/* END MARKER */
};
static const struct regval_list custom_ov7670[] = {
	{REG_COM14, 0x19},
	{0x72, 0x11},
	{0x73, 0xf1},
	{REG_HSTART,0x16},
	{REG_HSTOP,0x04},
	{REG_HREF,0x24},
	{REG_VSTART, 30},
	{REG_VSTOP, 90},
	{REG_VREF,0x0},
	{0xff, 0xff},	/* END MARKER */
};
static const struct regval_list yuv422_ov7670[] = {
	{REG_COM7, 0x0},	/* Selects YUV mode */
	{REG_RGB444, 0},	/* No RGB444 please */
	{REG_COM1, 0},
	{REG_COM15, COM15_R00FF},
	{REG_COM9, 0x6A},	/* 128x gain ceiling; 0x8 is reserved bit */
	{0x4f, 0x80},		/* "matrix coefficient 1" */
	{0x50, 0x80},		/* "matrix coefficient 2" */
	{0x51, 0},			/* vb */
	{0x52, 0x22},		/* "matrix coefficient 4" */
	{0x53, 0x5e},		/* "matrix coefficient 5" */
	{0x54, 0x80},		/* "matrix coefficient 6" */
	{REG_COM13,/*COM13_GAMMA|*/COM13_UVSAT},
	{0xff, 0xff},		/* END MARKER */
};
static const struct regval_list rgb565_ov7670[] = {
	{REG_COM7, COM7_RGB}, /* Selects RGB mode */
	{REG_RGB444, 0},	  /* No RGB444 please */
	{REG_COM1, 0x0},
	{REG_COM15, COM15_RGB565|COM15_R00FF},
	{REG_COM9, 0x6A},	 /* 128x gain ceiling; 0x8 is reserved bit */
	//{REG_COM9, 0x3A},	 /* 16x gain ceiling; 0x8 is reserved bit */
	{0x4f, 0xb3},		 /* "matrix coefficient 1" */
	{0x50, 0xb3},		 /* "matrix coefficient 2" */
	{0x51, 0},		 	 /* vb */
	{0x52, 0x3d},		 /* "matrix coefficient 4" */
	{0x53, 0xa7},		 /* "matrix coefficient 5" */
	{0x54, 0xe4},		 /* "matrix coefficient 6" */
	{REG_COM13, /*COM13_GAMMA|*/COM13_UVSAT},
	{0xff, 0xff},	/* END MARKER */
};
static const struct regval_list bayerRGB_ov7670[] = {
	{REG_COM7, COM7_BAYER},
	{REG_COM13, 0x08}, /* No gamma, magic rsvd bit */
	{REG_COM16, 0x3d}, /* Edge enhancement, denoise */
	{REG_REG76, 0xe1}, /* Pix correction, magic rsvd */
	{0xff, 0xff},	/* END MARKER */
};
static const struct regval_list ov7670_default_regs[] = {//from the linux driver
	{REG_COM7, COM7_RESET},

	{REG_TSLB,  0x04},	/* OV */
	{REG_COM7, 0},	/* VGA */
	/*
	 * Set the hardware window.  These values from OV don't entirely
	 * make sense - hstop is less than hstart.  But they work...
	 */
	{REG_HSTART, 0x13},	{REG_HSTOP, 0x01},
	{REG_HREF, 0xb6},	{REG_VSTART, 0x02},
	{REG_VSTOP, 0x7a},	{REG_VREF, 0x0a},

	{REG_COM3, 0},	{REG_COM14, 0},
	/* Mystery scaling numbers */
	{0x70, 0x3a},		{0x71, 0x35},
	{0x72, 0x11},		{0x73, 0xf0},
	{0xa2,/* 0x02 changed to 1*/1},{REG_COM10, COM10_VS_NEG},
	/* Gamma curve values */
	{0x7a, 0x20},		{0x7b, 0x10},
	{0x7c, 0x1e},		{0x7d, 0x35},
	{0x7e, 0x5a},		{0x7f, 0x69},
	{0x80, 0x76},		{0x81, 0x80},
	{0x82, 0x88},		{0x83, 0x8f},
	{0x84, 0x96},		{0x85, 0xa3},
	{0x86, 0xaf},		{0x87, 0xc4},
	{0x88, 0xd7},		{0x89, 0xe8},
	/* AGC and AEC parameters.  Note we start by disabling those features,
	   then turn them only after tweaking the values. */
	{REG_COM8, COM8_FASTAEC | COM8_AECSTEP},
	{REG_GAIN, 0},	{REG_AECH, 0},
	{REG_COM4, 0x40}, /* magic reserved bit */
	{REG_COM9, 0x18}, /* 4x gain + magic rsvd bit */
	{REG_BD50MAX, 0x05},	{REG_BD60MAX, 0x07},
	{REG_AEW, 0x95},	{REG_AEB, 0x33},
	{REG_VPT, 0xe3},	{REG_HAECC1, 0x78},
	{REG_HAECC2, 0x68},	{0xa1, 0x03}, /* magic */
	{REG_HAECC3, 0xd8},	{REG_HAECC4, 0xd8},
	{REG_HAECC5, 0xf0},	{REG_HAECC6, 0x90},
	{REG_HAECC7, 0x94},
	{REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_AGC|COM8_AEC},
	{0x30,0},{0x31,0},//disable some delays
	/* Almost all of these are magic "reserved" values.  */
	{REG_COM5, 0x61},	{REG_COM6, 0x4b},
	{0x16, 0x02},		{REG_MVFP, 0x07},
	{0x21, 0x02},		{0x22, 0x91},
	{0x29, 0x07},		{0x33, 0x0b},
	{0x35, 0x0b},		{0x37, 0x1d},
	{0x38, 0x71},		{0x39, 0x2a},
	{REG_COM12, 0x78},	{0x4d, 0x40},
	{0x4e, 0x20},		{REG_GFIX, 0},
	/*{0x6b, 0x4a},*/		{0x74,0x10},
	{0x8d, 0x4f},		{0x8e, 0},
	{0x8f, 0},		{0x90, 0},
	{0x91, 0},		{0x96, 0},
	{0x9a, 0},		{0xb0, 0x84},
	{0xb1, 0x0c},		{0xb2, 0x0e},
	{0xb3, 0x82},		{0xb8, 0x0a},

	/* More reserved magic, some of which tweaks white balance */
	{0x43, 0x0a},		{0x44, 0xf0},
	{0x45, 0x34},		{0x46, 0x58},
	{0x47, 0x28},		{0x48, 0x3a},
	{0x59, 0x88},		{0x5a, 0x88},
	{0x5b, 0x44},		{0x5c, 0x67},
	{0x5d, 0x49},		{0x5e, 0x0e},
	{0x6c, 0x0a},		{0x6d, 0x55},
	{0x6e, 0x11},		{0x6f, 0x9E}, /* it was 0x9F "9e for advance AWB" */
	{0x6a, 0x40},		{REG_BLUE, 0x40},
	{REG_RED, 0x60},
	{REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_AGC|COM8_AEC|COM8_AWB},

	/* Matrix coefficients */
	{0x4f, 0x80},		{0x50, 0x80},
	{0x51, 0},		{0x52, 0x22},
	{0x53, 0x5e},		{0x54, 0x80},
	{0x58, 0x9e},

	{REG_COM16, COM16_AWBGAIN},	{REG_EDGE, 0},
	{0x75, 0x05},		{REG_REG76, 0xe1},
	{0x4c, 0},		{0x77, 0x01},
	{REG_COM13, /*0xc3*/0x48},	{0x4b, 0x09},
	{0xc9, 0x60},		/*{REG_COM16, 0x38},*/
	{0x56, 0x40},

	{0x34, 0x11},		{REG_COM11, COM11_EXP|COM11_HZAUTO},
	{0xa4, 0x82/*Was 0x88*/},		{0x96, 0},
	{0x97, 0x30},		{0x98, 0x20},
	{0x99, 0x30},		{0x9a, 0x84},
	{0x9b, 0x29},		{0x9c, 0x03},
	{0x9d, 0x4c},		{0x9e, 0x3f},
	{0x78, 0x04},

	/* Extra-weird stuff.  Some sort of multiplexor register */
	{0x79, 0x01},		{0xc8, 0xf0},
	{0x79, 0x0f},		{0xc8, 0x00},
	{0x79, 0x10},		{0xc8, 0x7e},
	{0x79, 0x0a},		{0xc8, 0x80},
	{0x79, 0x0b},		{0xc8, 0x01},
	{0x79, 0x0c},		{0xc8, 0x0f},
	{0x79, 0x0d},		{0xc8, 0x20},
	{0x79, 0x09},		{0xc8, 0x80},
	{0x79, 0x02},		{0xc8, 0xc0},
	{0x79, 0x03},		{0xc8, 0x40},
	{0x79, 0x05},		{0xc8, 0x30},
	{0x79, 0x26},

	{0xff, 0xff},	/* END MARKER */
};
static void errorLed(void)
{

}

void ov7670_setColorSpace(enum OV7670_COLORSPACE color){
	switch(color){
		case OV7670_YUV422:
			wrSensorRegs8_8(yuv422_ov7670);
		break;
		case OV7670_RGB565:
			wrSensorRegs8_8(rgb565_ov7670);
			uint8_t temp=camera_ReadReg(0x11);
			camera_WriteReg(0x11,temp);//according to the Linux kernel driver rgb565 PCLK needs rewriting
		break;
		case OV7670_BAYER_RGB:
			wrSensorRegs8_8(bayerRGB_ov7670);
		break;
	}
}

void ov7670_setRes(enum OV7670_RESOLUTION res)
{
	switch(res)
	{
		case OV7670_VGA:
			camera_WriteReg(REG_COM3,0);	// REG_COM3
			wrSensorRegs8_8(vga_ov7670);
		break;
		case OV7670_QVGA:
			camera_WriteReg(REG_COM3, 4);	// REG_COM3 enable scaling
			wrSensorRegs8_8(qvga_ov7670);
		break;
		case OV7670_QQVGA:
			camera_WriteReg(REG_COM3, 4);	// REG_COM3 enable scaling
			wrSensorRegs8_8(qqvga_ov7670);
		break;
		case OV7670_CUSTOM:
			camera_WriteReg(REG_COM3, 4);	// REG_COM3 enable scaling
			wrSensorRegs8_8(custom_ov7670);
		break;
	}
}

void ov7670_init(void)
{
	//Reset the camera
	camera_WriteReg(REG_COM7, COM7_RESET);
	delay_ms(100);

	wrSensorRegs8_8(ov7670_default_regs);

	// more drive capability
	camera_WriteReg(REG_COM2, 3);

	//PCLK does not toggle on HBLANK.
	// zakomentovano -> zde jsou PCLK pulsy vyhodne, protoze dojde vzdy k docteni radku
	//i2c_WriteReg(REG_COM10, 32);
}

void ov7670_colorbarpattern(void)
{
	// test pattern
	camera_WriteReg(0x70, 0x80);
	camera_WriteReg(0x71, 0x80);
}
