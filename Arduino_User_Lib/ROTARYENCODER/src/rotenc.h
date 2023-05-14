/************************************************************************
	ROTENC
Author: Sergio Santos
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: Rotary encoder Potentiometer
Date: 25102020
Comment:
	Stable
************************************************************************/
#ifndef _ROTENC_H_
	#define _ROTENC_H_

/*** Global Library ***/
#include <inttypes.h>

/*** Global Constant & Macro ***/

/*** Global Variable ***/
typedef struct{
	uint8_t PinChnA;
	uint8_t PinChnB;
	uint8_t pchn;
	uint8_t chn;
	uint16_t num;
}rotaryencoderparameter;
	
struct rotenc{
	// VARIABLIES
	rotaryencoderparameter par;
	// PROTOTYPES VTABLE
	rotaryencoderparameter (*rte)(rotaryencoderparameter* par, uint8_t data);
};
typedef struct rotenc ROTENC;

/*** Global Header ***/
ROTENC ROTENCenable(uint8_t ChnApin, uint8_t ChnBpin);

#endif

/***EOF***/

