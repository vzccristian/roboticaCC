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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
    mutex = new QMutex(QMutex::Recursive);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


	innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");

	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
    
/* 
  float distumbral = 300;
  float angleumbral = 0.79;
  float rotacion=0.6;
  bool valido=false;
  
  //Tomar los datos del laser
  TLaserData data = laser_proxy->getLaserData(); 
  //Ordenamos el array de menor a mayor
  std::sort( data.begin()+25, data.end()-25, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ; 
  
  for(auto h: data)
    qDebug() << h.angle << h.dist;
  
    if (data[25].dist < distumbral) {
      qDebug()<< "Valido: "<<data[25].dist<<data[25].angle;
      if ( (rand()%(10)) % 2 != 0)
        rotacion=rotacion*(-1);
        differentialrobot_proxy->setSpeedBase(5, rotacion);
      usleep(rand()%(1500000-100000 + 1) + 100000);
    } else  {
    differentialrobot_proxy->setSpeedBase(1500, 0);
 }*/
  
// LEER DATOS
   if(!target.isEmpty()) {
    float x;
    float z;
    mutex->lock();
    target.extract(x,z); 
  //  qDebug() <<x << z ;
    mutex->unlock();
    
// MOVER ROBOT
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    
        differentialrobot_proxy->setSpeedBase(100, 0);
        usleep(rand()%(1500000-100000 + 1) + 100000);
            qDebug() << "No vacio: "<< bState.z << bState.correctedZ;
    }
    
    

}



void SpecificWorker::setPick(const Pick &myPick)
{
  mutex->lock();
  qDebug() << myPick.x << myPick.z ;
  target.insert(myPick.x,myPick.z); 
  mutex->unlock();

}





