/*
 *    Copyright (C) 2017 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author Cristina Mendoza Gutiérrez y Cristian Vázquez Cordero
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <math.h>       /* sqrt */

using namespace std;
enum state { IDLE, GOTO, TURN, SKIRT, END};

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    SpecificWorker(MapPrx& mprx);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);

    //CONTROL DEL ROBOT Y LASER
    float gauss(float Vrot, float Vx, float h);
    float sinusoide(float x);
    void idle();
    float gotoTarget(TBaseState bState, TLaserData laserData);
    void end();
    void skirt(TBaseState bState, TLaserData &laserData);
    void turn(float linealSpeed,TLaserData laserData);
    void onTarget(float dist);
    void isOnLine(TBaseState bState);
    void reachableTarget(TBaseState bState, float dist, TLaserData &laserData);


    //LLAMADAS INTERFAZ
    void go(const float x, const float z);
    void turn(const float speed);
    bool getState();
    void stop();
    void setPick(const Pick &myPick);




public slots:
    void compute();

private:
    InnerModel *innermodel;
    state estado;
    float const VLIN_MAX = 700;
    float const VROT_MAX = 0.6;
    bool lado; //TRUE = DERECHA, FALSE = IZQUIERDA.
    bool preState = true;
    bool pick=false; /* Flag diferencia entre pick y searchTags */

    struct Target
    {
        QMutex mutex; //Para hacer las operaciones sobre el target atómicas
        float xt, zt, xr, zr;
        bool empty;


        //Constructor
        Target() {
            xt = 0;
            zt = 0;
            xr = 0;
            zr = 0;
            empty = true;
        };

        //Inserta las coord x y z en el target
        bool insert(float _xt, float _zt, float _xr, float _zr ) {
            QMutexLocker ml(&mutex); //Controla el mutex
            xt = _xt;
            zt = _zt;
            xr = _xr;
            zr = _zr;
            empty = false;
            return true;
        };

        //Extrae las coord x y z del target y del robot
        std::pair <std::pair <float,float>,std::pair <float,float>> extract() {
            QMutexLocker ml(&mutex);
            std::pair <std::pair <float,float>,std::pair <float,float>> coors;
            std::pair <float,float> coorsTarget;
            std::pair <float,float> coorsRobot;
            coorsTarget.first=xt;
            coorsTarget.second=zt;
            coorsRobot.first=xr;
            coorsRobot.second=zr;
            coors.first=coorsTarget;
            coors.second=coorsRobot;
            return coors;
        };

        //Devuelve si el target esta vacio
        bool isEmpty() {
            QMutexLocker ml(&mutex);
            return empty;
        };

        //Pone a vacio el target
        void setEmpty() {
            QMutexLocker ml(&mutex);
            empty =true;
        };

        //Devuelve las coordenadas del target
        pair <float,float> getPoseTarget() {
            std::pair <float,float> coorsTarget;
            coorsTarget.first=xt;
            coorsTarget.second=zt;
            return coorsTarget;
        };

        //Devuelve las coordenadas del target
        pair <float,float> getPoseRobot() {
            std::pair <float,float> coorsRobot;
            coorsRobot.first=xr;
            coorsRobot.second=zr;
            return coorsRobot;
        };

    };

    Target target;


};

#endif
