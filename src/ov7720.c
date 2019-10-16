/*
 * ov7720.c
 *
 *  Created on: 8. 10. 2019
 *      Author: Petr Dvorak
 */

#include "ov7720.h"
#include "camera.h"
#include "delay.h"

#define ENDMARKER { 0xff, 0xff }

/*
 * register setting for window size
 */
static const struct regval_list ov772x_qvga_regs[] = {
		{0x17,0x3F},	// HSTART QVGA
		{0x18,0x50},	// HSIZE QVGA
		{0x19,0x03},	// VSTRT QVGA
		{0x1A,0x78},	// VSIZE QVGA
		{0x32,0x00},	// HREF
		{0x29,0x50},	// Houtsize QVGA
		{0x2C,0x78},	// Voutsize QVGA
		{0x11,0x14},	// CLKRC
		ENDMARKER,
};

static const struct regval_list ov772x_vga_regs[] = {
		{0x17,0x23},	// HSTART VGA
		{0x18,0xA0},	// HSIZE VGA
		{0x19,0x07},	// VSTRT VGA
		{0x1A,0xF0},	// VSIZE VGA
		{0x32,0x00},	// HREF
		{0x29,0xA0},	// Houtsize VGA
		{0x2C,0xF0},	// Voutsize VGA
		{0x11,0x14},	// CLKRC
		ENDMARKER,
};

static const struct regval_list ov7720_default_regs[] = {//from https://github.com/ArduCAM/Arduino/blob/master/ArduCAM/ov7725_regs.h, QVGA RGB565

		  {0x32,0x00},	// HREF
		  {0x2a,0x00},	// EXHCH dummy pixel insert
		  {0x2B,0x00},	// EXHCL dummy pixel insert

		  // velikost obrazu QVGA
		  {0x12,0x46},	// COM7 QVGA RGB565
		  {0x17,0x3F},	// HSTART QVGA
		  {0x18,0x50},	// HSIZE QVGA
		  {0x19,0x03},	// VSTRT QVGA
		  {0x1A,0x78},	// VSIZE QVGA
		  {0x32,0x00},	// HREF
		  {0x29,0x50},	// Houtsize QVGA
		  {0x2C,0x78},	// Voutsize QVGA
		  {0x11,0x14},	// CLKRC 0x14

		  {0x42,0x7f},	// TGT_B BLC Blue Channel Target Value
		  {0x4d,0x00},	// 0x09, fixgain
		  {0x63,0xf0},  // AWB_Ctrl0
		  {0x64,0xff},  // DSP_Ctrl1 *
		  {0x65,0x20},	// DSP_Ctrl2
		  {0x66,0x00},	// DSP_Ctrl3 *
		  {0x67,0x00},  // DSP_Ctrl4
		  {0x69,0x5d},  // AWBCtrl1

		  {0x13,0xff},  // COM8 *
		  {0x0d,0x00},	// COM4 PLL 0x00 *
		  {0x0f,0xc5},  // COM6 auto window setting
		  {0x14,0x11},	// COM9 AGC
		  {0x22,0xFF},	// BDBase 0x7f
		  {0x23,0x01},	// DBStep
		  {0x24,0x34},	// AEW
		  {0x25,0x3c},	// AEB
		  {0x26,0xa1},	// VPT
		  {0x2b,0x00},	// EXHCL
		  {0x6b,0xaa},	// AWBCtrl3
		  {0x13,0xff},	// COM8 *

		  {0x90,0x0A},	// EDGE1 * 0x0A
		  {0x91,0x01},	// DNSOff denoise 0x01
		  {0x92,0x01},	// EDGE2 0x01
		  {0x93,0x01},  // EDGE3 0x01

		  {0x94,0x5f},  // MTX1
		  {0x95,0x53},	// MTX2
		  {0x96,0x11},	// MTX3
		  {0x97,0x1a},	// MTX4
		  {0x98,0x3d},	// MTX5
		  {0x99,0x5a},	// MTX6
		  {0x9a,0x1e},	// MTX_Ctrl

		  {0x9b,0x00},	// BRIGHT set luma
		  {0x9c,0x25},	// CNST set contrast
		  {0xa7,0x65},	// USAT set saturation
		  {0xa8,0x65},	// VSAT set saturation
		  {0xa9,0x80},	// HUE0 set hue
		  {0xaa,0x80},	// HUE1 set hue

		  {0x9e,0x81},	// UVADJ0
		  {0xa6,0x06},	// SDE

		  {0x7e,0x0c},	// GAM1
		  {0x7f,0x16},	// GAM2
		  {0x80,0x2a},	// GAM3
		  {0x81,0x4e},	// GAM4
		  {0x82,0x61},	// GAM5
		  {0x83,0x6f},	// GAM6
		  {0x84,0x7b},	// GAM7
		  {0x85,0x86},	// GAM8
		  {0x87,0x97},	// GAM10
		  {0x88,0xa4},	// GAM11
		  {0x89,0xaf},	// GAM12
		  {0x8a,0xc5},	// GAM13
		  {0x8b,0xd7},	// GAM14
		  {0x8c,0xe8},	// GAM15
		  {0x8d,0x20},	// SLOP

		  {0x33,0x00},	// DM_LNL dummy row L
		  {0x34,0x00},	// DM_LNH dummy row H

		  {0x22,0x99},	// BDBase
		  {0x23,0x03},	// DBStep
		  {0x4a,0x00},	// LCC4
		  {0x49,0x13},  // LCC3
		  {0x47,0x08},	// LCC1
		  {0x4b,0x14},	// LCC5
		  {0x4c,0x17},	// LCC6
		  {0x46,0x05},	// LCC0
		  {0x0e,0x75},	// COM5 *
		  {0x0c,0x90},	// COM3
		  {0x00,0xF0},	// GAIN 0xf0
		  {0xff, 0xff},	//
};

void ov7720_setColorSpace(enum OV7720_COLORSPACE color)
{
	uint8_t reg;

	switch(color){
		case OV7720_YUV422:
			//wrSensorRegs8_8(yuv422_ov7720);
			reg = camera_ReadReg(COM7);
			reg &= ~0x0F;
			reg |= OFMT_YUV;
			camera_WriteReg(COM7, reg);
		break;
		case OV7720_RGB565:
			//wrSensorRegs8_8(rgb565_ov7720);
			reg = camera_ReadReg(COM7);
			reg &= ~0x0F;
			reg |= FMT_RGB565 + OFMT_RGB;
			camera_WriteReg(COM7, reg);
		break;
		case OV7720_BAYER_RGB:
			//wrSensorRegs8_8(bayerRGB_ov7720);
			reg = camera_ReadReg(COM7);
			reg &= ~0x0F;
			reg |= OFMT_BRAW;
			camera_WriteReg(COM7, reg);
		break;
	}
}

void ov7720_setRes(enum OV7720_RESOLUTION res)
{
	uint8_t reg;

	switch(res)
	{
		case OV7720_VGA:
			wrSensorRegs8_8(ov772x_vga_regs);
			reg = camera_ReadReg(COM7);
			reg &= ~SLCT_QVGA;
			reg |= SLCT_VGA;
			camera_WriteReg(COM7, reg);
		break;
		case OV7720_QVGA:
			wrSensorRegs8_8(ov772x_qvga_regs);
			reg = camera_ReadReg(COM7);
			reg |= SLCT_QVGA;
			camera_WriteReg(COM7, reg);
		break;
		case OV7720_QQVGA:

		break;
	}
}

void ov7720_init(void)
{
	//Reset the camera
	camera_WriteReg(COM7, SCCB_RESET);
	delay_ms(100);

	wrSensorRegs8_8(ov7720_default_regs);

	// more drive capability
	camera_WriteReg(COM2, 3);
}

void ov7720_colorbarpattern(void)
{
	// test pattern
	camera_WriteReg(0x0C, 0x01);
	camera_WriteReg(0x66, 0x20);
}

void ov7720_set_clk_prescaler(uint8_t prescaler)
{
	camera_WriteReg(0x11, prescaler);	// CLKRC
}
