/********************************************************************
	EXPLODE
Author: Sergio Santos
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: all
Date: 07052023
Comment:
	Pin Analysis
********************************************************************/
/*** File Library ***/
#include "explode.h"

/*** File Constant & Macro ***/

/*** File Variable ***/

/*** File Header ***/
void EXPLODEupdate(explodeparameter* par, uint8_t x);
uint8_t EXPLODEhh(explodeparameter* par);
uint8_t EXPLODEll(explodeparameter* par);
uint8_t EXPLODElh(explodeparameter* par);
uint8_t EXPLODEhl(explodeparameter* par);

/*** Procedure & Function ***/
EXPLODE EXPLODEenable( void )
{
	// struct object
	struct expld explode;
	// initialize VAR
	explode.par.XI = 0;
	explode.par.XF = 0;
	// function pointers
	explode.update = EXPLODEupdate;
	return explode; // return copy
}
// boot (preamble in while loop)
void EXPLODEupdate(explodeparameter* par, uint8_t x)
{
	par->XI = par->XF;
	par->XF = x;
	par->HL = EXPLODEhl(par);
	par->LH = EXPLODElh(par);
	par->HH = EXPLODEhh(par);
	par->LL = EXPLODEll(par);
}
// hh
uint8_t EXPLODEhh(explodeparameter* par)
{
	return (par->XI & par->XF);
}
// ll
uint8_t EXPLODEll(explodeparameter* par)
{
	return ~(par->XI | par->XF);
}
// lh
uint8_t EXPLODElh(explodeparameter* par)
{
	uint8_t i;
	i = par->XI ^ par->XF;
	i &= par->XF;
	return i;
}
// hl
uint8_t EXPLODEhl(explodeparameter* par)
{
	uint8_t i;
	i = par->XF ^ par->XI;
	i &= par->XI;
	return i;
}

/*** File Interrupt ***/

/***EOF***/

