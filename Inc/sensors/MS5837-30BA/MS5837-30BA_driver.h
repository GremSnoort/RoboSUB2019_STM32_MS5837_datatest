/*
 * Copyright (C) 2018-2019 RC-AUV.
 * All rights reserved.
 */

#ifndef _MS5837_30BA_H_
#define _MS5837_30BA_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include "sensors/Sensors_priv.h"

#define MS5837_ADDR_READ          0xED
#define MS5837_ADDR_WRITE         0xEC
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

static const uint8_t ms5837_30ba = 0;
static const uint8_t ms5837_02ba = 1;

static const float Pa   = 100.0f;
static const float bar  = 0.001f;
static const float mbar = 1.0f;

typedef struct {
	I2C_HandleTypeDef* i2c;

	uint16_t calibData[8];
	
	uint32_t D1;
	uint32_t D2;
	int32_t  TEMP;
	int32_t  P;
	
	uint8_t  model;
	
	float fluidDensity;
}MS5837_Handle;

static void InitParams( MS5837_Handle *dev );

/* Set model of MS5837 sensor. Valid options are ms5837_30ba (default)
 * and ms5837_02ba.
 */
void MS5837SetModel( MS5837_Handle *dev, uint8_t model );

/* Provide the density of the working fluid in kg/m^3. Default is for 
 * seawater. Should be 997 for freshwater.
 */
void MS5837SetFluidDensity( MS5837_Handle *dev, float density );

DrvStatus MS5837Init( MS5837_Handle *dev );

float MS5837Pressure( MS5837_Handle *dev, float conversion );
float MS5837Temperature( MS5837_Handle *dev );
float MS5837Depth( MS5837_Handle *dev );
float MS5837Altitude( MS5837_Handle *dev );

static void Calculate( MS5837_Handle *dev );

DrvStatus MS5837Read( MS5837_Handle *dev );

#ifdef __cplusplus
}
#endif

#endif /* _MS5837_30BA_H_ */
