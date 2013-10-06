/***************************************************************************

    file                 : BASE.cpp
    created              : jue oct 3 13:28:27 CEST 2013
    copyright            : (C) 2002 CarlosRodriguez

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

float getAllowedSpeed(tTrackSeg *segment);
float getAccel(tCarElt* car);
float getDistToSegEnd(tCarElt* car);
float getBrake(tCarElt* car);
float setCorrectDir(tCarElt* car);
int getGear(tCarElt *car);

void setDistanciaAlMedio(tTrackSeg *segment);

void actualizaFreno(tCarElt* car);

/* Parámetros ajustables */
const float cteDeAjusteLateral   = 0.5;		// Constante para el error lateral

const float cteGravedad = 9.81;			/* [m/(s*s)] */
const float FULL_ACCEL_MARGIN = 1.0;	/* [m/s] */

const float SHIFT = 0.9;         /* [-] (% of rpmredline) */
const float SHIFT_MARGIN = 8.0;  /* [m/s] */

const float cteFreno = 0.075;
const float cteFrenoMotor = 85;

/* Variables globales */
float freno = 0.0;
float actualSD = 0.0; // SD = SPeed Difference. Diferencia de velocidad entre la actual y la deseada.
float auxSD = 0.0; // SD = SPeed Difference.
float distanciaAlMedio = 0.0;

/* 
 * Module entry point  
 */ 
extern "C" int 
BASE(tModInfo *modInfo) 
{
	memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = const_cast<char *>("BASE");		/* name of the module (short) */
    modInfo->desc    = const_cast<char *>("");			/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;						/* init function */
    modInfo->gfId    = ROB_IDENT;						/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
}

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
	setDistanciaAlMedio(car->_trkPos.seg);
	actualizaFreno(car);
	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	// set up the values to return
	car->ctrl.steer = setCorrectDir(car) / car->_steerLock; // car->_steerLock es el ángulo máximo de giro de la rueda
	car->ctrl.gear = getGear(car);
	car->ctrl.brakeCmd = freno;
	if (car->ctrl.brakeCmd == 0.0) {
		car->ctrl.accelCmd = getAccel(car);
	} else {
		car->ctrl.accelCmd = 0.0;
	}

    /*  
     * add the driving code here to modify the 
     * car->_steerCmd 
     * car->_accelCmd 
     * car->_brakeCmd 
     * car->_gearCmd 
     * car->_clutchCmd 
     */ 
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

/***************************************************************************
 *
 * Funciones complementarias
 *
***************************************************************************/

/* Actualiza la distancia al centro de la pista deseada con cada golpe de reloj */
void setDistanciaAlMedio(tTrackSeg *segment)
{
	if(segment->type == TR_LFT){
		if(distanciaAlMedio > -(segment->width/2)){
			distanciaAlMedio -= 0.2;
		}
	}
	else if(segment->type == TR_RGT){
		if(distanciaAlMedio < (segment->width/2)){
			distanciaAlMedio += 0.2;
		}
	}
	else if(segment->type == TR_STR){
		if(distanciaAlMedio<0){
			distanciaAlMedio += 0.2;
		}
		else if (distanciaAlMedio>0){
			distanciaAlMedio -= 0.2;
		}
		else distanciaAlMedio = 0; // Overwriting, I know. It's just for a better comprehension
	}
}

/* Actualiza la dirección correcta con cada golpe de reloj */
float setCorrectDir(tCarElt* car)
{
	float direccion, eAngular, eLateral;

	eAngular = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	eLateral = cteDeAjusteLateral * (car->_trkPos.toMiddle + distanciaAlMedio) / car->_trkPos.seg->width;
	direccion = eAngular - eLateral;
	NORM_PI_PI(direccion); 			// Put the angle back in the range from -PI to PI
	return direccion;
}

/* Compute the allowed speed on a segment */
float getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->next->type == TR_STR or (segment->next->next->type == TR_STR and segment->next->next->next->type == TR_STR)){
        return FLT_MAX;
    } else {
        float mu = segment->surface->kFriction;
        return sqrt(mu*cteGravedad*segment->radius);
    }
}

/* Compute the length to the end of the segment */
float getDistToSegEnd(tCarElt* car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}

/* Compute fitting acceleration */
float getAccel(tCarElt* car)
{
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
    }
}

void actualizaFreno(tCarElt* car)
{
	tTrackSeg *segptr = car->_trkPos.seg; // segptr: puntero al segmento actual
	float currentspeedsqr = car->_speed_x*car->_speed_x;
	float mu = segptr->surface->kFriction;	// Coeficiente de rozamiento del segmento
	float maxlookaheaddist = currentspeedsqr/(2.0*mu*G); // Distancia de frenado para el peor de los casos
	float lookaheaddist = getDistToSegEnd(car);	// Distancia para la que la frenada ya ha sido calculada (en principio el final del segmento)

	float allowedspeed = getAllowedSpeed(segptr);
	if (allowedspeed < car->_speed_x){
		float actualSD = car->_speed_x - allowedspeed;
		if (actualSD >= auxSD){
			auxSD = actualSD;
			if(freno<1.0){
				freno = freno + cteFreno;
			}
		}
	}
	else if (allowedspeed > car->_speed_x + 4){
		freno = 0.0;
		auxSD = 0.0;
	}

	segptr = segptr->next;	// segptr ahora apunta al siguiente segmento del circuito
	while (lookaheaddist < maxlookaheaddist) {
		allowedspeed = getAllowedSpeed(segptr);
		if (allowedspeed < car->_speed_x) {
			float allowedspeedsqr = allowedspeed*allowedspeed;
			float brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*mu*G);
			if (brakedist > lookaheaddist) {
				float actualSD = car->_speed_x - allowedspeed;
				if(actualSD>=auxSD){
					auxSD = actualSD;
					if (freno<1.0){
						freno = freno + cteFreno;
					}
				}
			}
		}
		else if (allowedspeed > car->_speed_x + 4){
			freno = 0.0;
			auxSD = 0.0;
		}
		lookaheaddist += segptr->length;
		segptr = segptr->next;
	}
	printf("\n %f - ", freno);
}

/* Compute gear */
int getGear(tCarElt *car)
{
	if (car->_gear <= 0) return 1;
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && (omega*wr*SHIFT) + (freno+0.01)*cteFrenoMotor > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}