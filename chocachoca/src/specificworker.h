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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);      
	float gauss(float Vrot, float Vx, float h);
	float sinusoide(float x);

public slots:
	void compute(); 	

private:
    InnerModel *innermodel;

    float const V_MAX = 400;
    
    struct Target
    {
        QMutex mutex; //Para hacer las operaciones sobre el target atómicas
        float x,z;
        bool empty;
        
        //Constructor
        Target(){
            x = 0;
            z = 0;
            empty = true;
        };
        
        //Inserta las coord x y z en el target
        bool insert(float _x, float _z){
            QMutexLocker ml(&mutex); //Controla el mutex
            x = _x;
            z = _z;
            empty = false;
            return true;
        };
        
        //Extrae las coord x y z del target
        std::pair <float,float> extract() {
            QMutexLocker ml(&mutex);
            std::pair <float,float> coor;
            coor.first=x;
            coor.second=z;
            return coor;
        };
        
        //Devuelve si el target esta vacio
        bool isEmpty() {
            QMutexLocker ml(&mutex);
            return empty;
        };
        
        //Pone a vacio el target
        void setEmpty(){
            QMutexLocker ml(&mutex);
            empty =true;
        }
        
        //Devuelve true si el robot ha alcanzado el objetivo
        bool enObjetivo(float _x, float _z){
            if(abs(_x - x) < 100 && abs(_z - z) < 100)
                return true;
            else
                return false;
        }
            
    };
	
    Target target;

  
};

#endif

