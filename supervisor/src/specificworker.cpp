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

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
    
    mtx.lock();
            
    
    mtx.unlock();
    
    //MAQUINA DE ESTADOS
    switch (estado)  
      {  
        case SEARCH: 
            search();
            break; 
        case WAIT:
            wait();
            break;  
        default:  
	    break;
    } 
      
      
      //while (!noObjetivo)
  //Robot GIRAR
  //Comprobar si ve QR
    //Si QRvisto=estado
      //switch estado
      //estado0
	//GOTO x0 y0
	//estado=QR1
      //estado1
	//GOTO x1 y1
	//estado=QR2
	//etc..
    
    
}

void SpecificWorker::search() {
    
    
}

void SpecificWorker::wait() {

    while (chocachoca_proxy->getState()) {
        /* No hacer nada */ }
        
    
}





void SpecificWorker::newAprilTag(const tagsList &tags)
{
    mtx.lock();
        for (auto w:watchingtags)  //Inicialización
            w=0; 
        for (auto h:tags) { //Escritura
            qDebug() << h.id;
            watchingtags[h.id]=1;
        }
    mtx.unlock();
}






