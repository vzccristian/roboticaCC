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
       @author authorname
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
	bool noTarget(float x, float z); //True si hemos llegado al objetivo
	

public slots:
	void compute(); 	

private:
    InnerModel *innermodel;

    struct Target
    {
	QMutex mutex;
        float x,z;
        bool empty;
        Target(){
            x=0;
            z=0;
            empty=true;
        };
        bool insert(float _x, float _z){
	    QMutexLocker ml(&mutex);
            x=_x;
            z=_z;
            empty=false;
            return true;
        };
        bool extract(float &_x, float &_z) {
	    QMutexLocker ml(&mutex);
            _x=x;
            _z=z;
            empty=true;
            return true;
        };
        bool isEmpty() {
	    QMutexLocker ml(&mutex);
            return empty;
        };
    };
	
    Target target;

  
};

#endif

