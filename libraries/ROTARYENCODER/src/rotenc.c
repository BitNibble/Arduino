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
uint8_t ROTENClh(uint8_t xp, uint8_t xi);
uint8_t ROTENChl(uint8_t xp, uint8_t xi);

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
	hl = ROTENChl(par->pchn, par->chn);
	if(hl){
		if((par->pchn == ((1 << par->PinChnB) | (1 << par->PinChnA))) && (hl & (1 << par->PinChnA)))
		par->num++;
		if((par->pchn == ((1 << par->PinChnB) | (1 << par->PinChnA))) && (hl & (1 << par->PinChnB)))
		par->num--;
	}
	par->pchn = par->chn;
	return *par;
}

// auxiliar
uint8_t ROTENClh(uint8_t xp, uint8_t xi)
{
	uint8_t i;
	i = xp ^ xi;
	i &= xi;
	return i;
}
uint8_t ROTENChl(uint8_t xp, uint8_t xi)
{
	uint8_t i;
	i = xi ^ xp;
	i &= xp;
	return i;
}

/*** File Interrupt ***/

/***EOF***/

