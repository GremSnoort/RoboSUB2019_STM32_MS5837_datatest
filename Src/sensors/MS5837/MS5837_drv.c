/*
 * Copyright (C) 2018-2019 RC-AUV.
 * All rights reserved.
 */

#include "sensors/MS5837/MS5837_drv.h"

#include <math.h>

void MS5837SetModel( struct MS5837Device* dev, MS5837Model model )
{
	dev->model = model;
}

void MS5837SetFluidDensity( struct MS5837Device* dev, float density )
{
	dev->fluidDensity = density;
}

DrvStatus MS5837Init( struct MS5837Device* dev )
{
	uint8_t cmd;
	uint8_t buffer[2];
	
	// Reset the MS5837 according to datasheet
	cmd = MS5837_RESET;
	if ( HAL_I2C_Master_Transmit( dev->i2c, MS5837_ADDR_WRITE, &cmd, 1, 10000 ) != HAL_OK )
	{
		return DRV_RESET_FAILURE;
	}
	
	// Wait for reset to complete
	HAL_Delay(10);

	// Read calibration values and CRC
	for ( uint8_t i = 0; i < 7; i++ )
	{
		cmd = MS5837_PROM_READ + i*2;
		if( HAL_I2C_Master_Transmit( dev->i2c, MS5837_ADDR_WRITE, &cmd, 1, 10000 ) != HAL_OK )
		{
			return DRV_TRANSMIT_FAILURE;
		}
		
		if ( HAL_I2C_Master_Receive( dev->i2c, MS5837_ADDR_READ, buffer, 2, 10000 ) != HAL_OK )
		{
			return DRV_RECIEVE_FAILURE;
		}
		
		dev->calibData[i] = to_uint16( buffer );
	}

	// Verify data with CRC
	uint8_t crcRead = dev->calibData[0] >> 12;
	uint8_t crcCalculated = crc4( dev->calibData );

	if ( crcCalculated == crcRead )
	{
		return DRV_SUCCESS; // Initialization success
	}

	return DRV_CRC_ERROR; // CRC fail
}

float MS5837Pressure( struct MS5837Device* dev, float conversion )
{
	return dev->P * conversion;
}

float MS5837Temperature( struct MS5837Device* dev )
{
	return dev->TEMP / 100.0f;
}

float MS5837Depth( struct MS5837Device* dev )
{
	return ( MS5837Pressure( dev, Pa ) - .101300 ) / ( dev->fluidDensity * 9.80665 );
}

float MS5837Altitude( struct MS5837Device* dev )
{
	return ( 1 - pow( ( MS5837Pressure( dev, mbar ) / 1013.25), .190284 ) ) * 145366.45 * .3048;
}

static void MS5837Calculate( struct MS5837Device* dev )
{
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation	
	int32_t dT 		= 0;
	int64_t SENS 	= 0;
	int64_t OFF 	= 0;
	int32_t SENSi = 0;
	int32_t OFFi 	= 0;  
	int32_t Ti 		= 0;    
	int64_t OFF2 	= 0;
	int64_t SENS2 = 0;
	
	// Terms called
	dT = dev->D2 - (uint32_t)(dev->calibData[5]) * 256l;
	if ( dev->model == MS5837_02BA )
	{
		SENS = (int64_t)(dev->calibData[1]) * 65536l + ( (int64_t)(dev->calibData[3]) * dT ) / 128l;
		OFF = (int64_t)(dev->calibData[2]) * 131072l + ( (int64_t)(dev->calibData[4]) * dT ) / 64l;
		dev->P = ( dev->D1 * SENS/(2097152l) - OFF ) / (32768l);
	}
	else
	{
		SENS = (int64_t)(dev->calibData[1]) * 32768l + ( (int64_t)(dev->calibData[3]) * dT ) / 256l;
		OFF = (int64_t)(dev->calibData[2]) * 65536l + ( (int64_t)(dev->calibData[4]) * dT ) / 128l;
		dev->P = ( dev->D1 * SENS / (2097152l)-OFF ) / (8192l);
	}
	
	// Temp conversion
	dev->TEMP = 2000l + (int64_t)(dT) * dev->calibData[6] / 8388608LL;
	
	//Second order compensation
	if ( dev->model == MS5837_02BA )
	{
		if ( (dev->TEMP / 100) < 20 )
		{
			//Low temp
			Ti = ( 11*(int64_t)(dT) * (int64_t)(dT) ) / (34359738368LL);
			OFFi = ( 31 * ( dev->TEMP-2000 ) * ( dev->TEMP-2000 ) ) / 8;
			SENSi = ( 63 * ( dev->TEMP-2000 ) * ( dev->TEMP-2000 ) ) / 32;
		}
	}
	else
	{
		if ( (dev->TEMP / 100) < 20 )
		{
			//Low temp
			Ti = ( 3 * (int64_t)(dT) * (int64_t)(dT) ) / (8589934592LL);
			OFFi = ( 3 * ( dev->TEMP - 2000 ) * ( dev->TEMP - 2000 ) ) / 2;
			SENSi = ( 5 * ( dev->TEMP - 2000 ) * ( dev->TEMP - 2000 ) ) / 8;
			
			if ( (dev->TEMP / 100) < -15 )
			{
				//Very low temp
				OFFi = OFFi + 7 * ( dev->TEMP + 1500l ) * ( dev->TEMP + 1500l );
				SENSi = SENSi + 4 * ( dev->TEMP + 1500l ) * ( dev->TEMP + 1500l );
			}
		}
		else if ( ( dev->TEMP / 100 ) >= 20 )
		{
			//High temp
			Ti = 2 * ( dT * dT ) / (137438953472LL);
			OFFi = ( 1 * ( dev->TEMP - 2000) * ( dev->TEMP - 2000 ) ) / 16;
			SENSi = 0;
		}
	}
	
	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
	SENS2 = SENS-SENSi;
	
	if ( dev->model == MS5837_02BA )
	{
		dev->TEMP = (dev->TEMP-Ti);
		dev->P = ( ( ( dev->D1 * SENS2 ) / 2097152l - OFF2 ) / 32768l ) / 100;
	}
	else
	{
		dev->TEMP = (dev->TEMP-Ti);
		dev->P = ( ( ( dev->D1 * SENS2 ) / 2097152l - OFF2 ) / 8192l ) / 10;
	}
}

DrvStatus MS5837Read( struct MS5837Device* dev )
{
	uint8_t cmd;
	uint8_t buffer[3];
	
	// Request D1 conversion
	cmd = MS5837_CONVERT_D1_8192;
	if ( HAL_I2C_Master_Transmit( dev->i2c, MS5837_ADDR_WRITE, &cmd, 1, 10000 ) != HAL_OK )
	{
		return DRV_TRANSMIT_FAILURE;
	}

	HAL_Delay(20); // Max conversion time per datasheet
	
	cmd = MS5837_ADC_READ;
	if ( HAL_I2C_Master_Transmit( dev->i2c, MS5837_ADDR_WRITE, &cmd, 1, 10000 ) != HAL_OK )
	{
		return DRV_TRANSMIT_FAILURE;
	}
	
	if ( HAL_I2C_Master_Receive( dev->i2c, MS5837_ADDR_READ, buffer, 3, 10000 ) != HAL_OK )
	{
		return DRV_RECIEVE_FAILURE;
	}
	
	dev->D1 = to_uint32( buffer );
	
	// Request D2 conversion
	cmd = MS5837_CONVERT_D2_8192;
	if ( HAL_I2C_Master_Transmit( dev->i2c, MS5837_ADDR_WRITE, &cmd, 1, 10000 ) != HAL_OK )
	{
		return DRV_TRANSMIT_FAILURE;
	}	
	
	HAL_Delay(20); // Max conversion time per datasheet
	
	cmd = MS5837_ADC_READ;
	if ( HAL_I2C_Master_Transmit( dev->i2c, MS5837_ADDR_WRITE, &cmd, 1, 10000 ) != HAL_OK )
	{
		return DRV_TRANSMIT_FAILURE;
	}
	
	if ( HAL_I2C_Master_Receive( dev->i2c, MS5837_ADDR_READ, buffer, 3, 10000 ) != HAL_OK )
	{
		return DRV_RECIEVE_FAILURE;
	}

	dev->D2 = to_uint32( buffer );	

	MS5837Calculate( dev );
	
	return DRV_SUCCESS;
}

MS5837Device MS5837GetNewDevice( MS5837Model model, float density, I2C_HandleTypeDef* i2c )
{
	MS5837Device dev;
	
	dev.model = model;
	dev.fluidDensity = density;
	dev.i2c = i2c;
	
	return dev;
}
