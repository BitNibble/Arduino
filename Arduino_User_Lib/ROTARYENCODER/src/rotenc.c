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
/*** File Library ***/
#include "rotenc.h"

/*** File Constant & Macro ***/

/*** File Variable ***/

/*** File Header ***/
rotaryencoderparameter RotEnc_rte(rotaryencoderparameter* par, uint8_t data);

/*** Procedure & Function ***/
ROTENC ROTENCenable( uint8_t ChnApin, uint8_t ChnBpin )
{
	// struct object
	ROTENC rtnc;
	// Initialize variables
	rtnc.par.PinChnA = ChnApin;
	rtnc.par.PinChnB = ChnBpin;
	rtnc.par.pchn = rtnc.par.chn = (1 << ChnBpin) | (1 << ChnApin);
	rtnc.par.num = 0;
	// function pointers
	rtnc.rte = RotEnc_rte;
	
	return rtnc;
}

rotaryencoderparameter RotEnc_rte(rotaryencoderparameter* par, uint8_t data)
{
	uint8_t hl;
	par->chn = data & ((1 << par->PinChnB) | (1 << par->PinChnA));
	hl = par->chn ^ par->pchn;
	hl &= par->pchn;
	if(par->pchn != par->chn){
		if((par->pchn == ((1 << par->PinChnB) | (1 << par->PinChnA))) && (hl & (1 << par->PinChnA)))
		par->num++;
		if((par->pchn == ((1 << par->PinChnB) | (1 << par->PinChnA))) && (hl & (1 << par->PinChnB)))
		par->num--;
	}
	par->pchn = par->chn;
	return *par;
}

/*** File Interrupt ***/

/***EOF***/

