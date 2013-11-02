/***************************************************************************

    file                 : bh0152_bh0137_Basico.cpp
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

static tTrack   *curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

float velocidadMaxima(tTrackSeg *segment);
float finalSegmento(tCarElt* car);
float Direccion(tCarElt* car);
int cajaDeCambios(tCarElt *car);

void actualizaFreno(tCarElt* car);
void actualizaAcelerador(tCarElt* car);

/* Parámetros ajustables */
const float cteDeAjusteLateral   = 35.0;    // Constante para el error lateral
const float cteAcelerador = 0.025;
const float margenAcelerador = 1.0; /* [m/s] */

/* Variables globales */
float freno = 0.0;
int exponenteFreno = 0;
float acelerador = 0.0;
float distanciaAlMedio = 0.0;

/* 
 * Module entry point  
 */ 
extern "C" int 
bh0152_bh0137_Basico(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = const_cast<char *>("bh0152_bh0137_Basico");      /* name of the module (short) */
    modInfo->desc    = const_cast<char *>("");          /* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;                      /* init function */
    modInfo->gfId    = ROB_IDENT;                       /* supported framework version */
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
    itf->rbNewRace  = newrace;   /* Start a new race */
    itf->rbDrive    = drive;     /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;   /* End of the current race */
    itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    itf->index      = index;     /* Index used if multiple interfaces */
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
    actualizaFreno(car);
    actualizaAcelerador(car);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    // set up the values to return
    car->ctrl.steer = Direccion(car) / car->_steerLock; // car->_steerLock es el ángulo máximo de giro de la rueda
    car->ctrl.gear = cajaDeCambios(car);
    car->ctrl.brakeCmd = freno;
    car->ctrl.accelCmd = acelerador;

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


/* Actualiza la dirección correcta con cada golpe de reloj */
float Direccion(tCarElt* car)
{
    float direccion, eAngular, eLateral;

    eAngular = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    eLateral = atan((car->_trkPos.toMiddle + distanciaAlMedio) / cteDeAjusteLateral);
    direccion = eAngular - eLateral;
    NORM_PI_PI(direccion);
    return direccion;
}

/* Calcula la marcha adecuada */
int cajaDeCambios(tCarElt *car)
{
    if (car->_gear <= 0) return 1;

    if (car->_enginerpm > 700){
        return car->_gear + 1;
    } 

    else if(car->_enginerpm < 200 and car->_gear > 1){
        return car->_gear - 1;
    }
    else if ((velocidadMaxima(car->_trkPos.seg) - car->_speed_x) < 10){ // Freno motor
        if(car->_enginerpm < 500 and car->_gear > 3){
            if (freno > 0.2 and car->_gear > 1){
                return car->_gear - 1;
            }
        }
    }

    return car->_gear;
}

/* Calcula la velocidad máxima para un segmento dado */
float velocidadMaxima(tTrackSeg *segment)
{
    if (segment->next->type == TR_STR){
        return 41.7;    // 150 km/h
    }
    else return 13.9;   // 50 km/h
}

/* Calcula el grado del acelerador a aplicar con cada golpe de reloj */
void actualizaAcelerador(tCarElt* car){
    if (freno == 0.0) {
        if(velocidadMaxima(car->_trkPos.seg) > car->_speed_x + margenAcelerador){
            if(acelerador < 1.0){
                acelerador = acelerador + cteAcelerador;
            }
        }
        else if (velocidadMaxima(car->_trkPos.seg) < car->_speed_x){
            acelerador = 0.0;
        }
    }
    else acelerador = 0.0;
}

/* Calcula el grado de frenada a aplicar con cada golpe de reloj */
void actualizaFreno(tCarElt* car){

    tTrackSeg *segmento = car->_trkPos.seg;
    float vCuadrado = car->_speed_x*car->_speed_x;
    float vMaxima = velocidadMaxima(segmento);

    if (vMaxima <= car->_speed_x){
            if(freno<1.0 and exponenteFreno < 4){
                freno = 0.01 * pow(3,exponenteFreno); 
                exponenteFreno++;
            }
    }
    else if (vMaxima > car->_speed_x){
        freno = 0.0;
        exponenteFreno = 0;
    }

    segmento = segmento->next;  // segmento ahora apunta al siguiente segmento del circuito
    float mu = segmento->surface->kFriction;    // Coeficiente de rozamiento del segmento
    float distCalculada = finalSegmento(car);   // Distancia para la que la frenada ya ha sido calculada
    float distMaxima = vCuadrado/(2.0*mu*G);    // Distancia de frenado para el peor de los casos
    while (distCalculada < distMaxima) {
        vMaxima = velocidadMaxima(segmento);
        if (vMaxima < car->_speed_x) {
            float vMaximaCuadrado = vMaxima*vMaxima;
            float distFrenado = (vCuadrado - vMaximaCuadrado) / (2.0*mu*G);
            if (distFrenado > distCalculada) {
                if (freno < 1.0 and exponenteFreno < 4){
                    freno = 0.01 * pow(3,exponenteFreno);
                    exponenteFreno++;
                }
            }
        }
        else if (vMaxima > car->_speed_x){
            freno = 0.0;
            exponenteFreno = 0;
        }
        distCalculada += segmento->length;
        segmento = segmento->next;
    }
}

/* Calcula la distancia por recorrer hasta llegar al final del segmento */
float finalSegmento(tCarElt* car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }
}