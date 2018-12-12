/*
 * Copyright (C) 2018-2019 RC-AUV.
 * All rights reserved.
 */
 
#ifndef _SENSORS_PRIV_H_
#define _SENSORS_PRIV_H_
 
#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
 
typedef enum EDrvStatus
{
	DRV_SUCCESS =  0,
	DRV_FAILURE  = -1,
	
	DRV_RESET_FAILURE,
	DRV_TRANSMIT_FAILURE,
	DRV_RECIEVE_FAILURE,
	DRV_CRC_ERROR,
 
}DrvStatus;
 
static uint16_t to_uint16( uint8_t bytes[] )
{
  return (uint16_t)( (bytes[0] << 8) | bytes[1] );
}

static uint32_t to_uint32( uint8_t bytes[] )
{
  return (uint32_t)( (bytes[0] << 16) | (bytes[1] << 8) | bytes[2] );
}

static uint8_t crc4( uint16_t n_prom[] )
{
	uint16_t n_rem = 0;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);
	n_prom[7] = 0;

	for ( uint8_t i = 0; i < 16; i++ )
	{
		if ( i%2 == 1 )
		{
			n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
		}
		else
		{
			n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
		}
		for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- )
		{
			if ( n_rem & 0x8000 )
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	
	n_rem = ((n_rem >> 12) & 0x000F);

	return n_rem ^ 0x00;
}
 
 #ifdef __cplusplus
}
#endif

#endif /* _SENSORS_PRIV_H_ */
