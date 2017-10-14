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
  estado = IDLE;
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
    //TOMAR DATOS DEL MUNDO
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    
    //ACTUALIZAR ARBOL
    innermodel->updateTransformValues("base", bState.x,0, bState.z,0,bState.alpha, 0 ); 
    
    //Tomar los datos del laser
    TLaserData laserData = laser_proxy->getLaserData(); 
//     int i=0;
//     while (i<100) {
//         cout <<"["<<i<<"] D:"<< laserData[i].dist<<" A:"<<laserData[i].angle<< "  ";
//         i++;
//         if (i%10==0)
//             cout << endl;
//         
//     }
    //Ordenamos el array de menor a mayor
    std::sort( laserData.begin()+25, laserData.end()-25, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return  a.dist < b.dist; }) ; 
    float linealSpeed=0;
    switch (estado)  
      {  
        case IDLE: 
            idle();
            break; 
        case GOTO:
            linealSpeed=goToPick(bState,laserData);
            break;  
        case END:
            end();
            break;
        case TURN:
            turn(linealSpeed,laserData);
            break;
        case SKIRT:
            skirt();
            break;
         default:  
	   
	   break;
      }  
    
    
    

}

// --------- MAQUINA DE ESTADOS --------------
// IDLE
 void SpecificWorker::idle(){
   qDebug() << "Estado IDLE";
   if(!target.isEmpty())
     estado=GOTO;
 }
// ----------------------


// GO TO PICK
float SpecificWorker::goToPick(TBaseState bState,TLaserData laserData) {
    qDebug() << "Estado PICK";
    if(!target.isEmpty()) { //EXISTE OBJETIVO
         //VARIABLES
        float dist,linealSpeed;
        std::pair <std::pair <float,float>,std::pair <float,float>> coord = target.extract(); //Tomamos las coord del pick (target y robot)
        QVec Trobot = innermodel->transform("base",QVec::vec3(coord.first.first,0,coord.first.second),"world"); //Desplaza el eje de coord del mundo al robot
        dist = Trobot.norm2(); //Calculamos la distancia entre los puntos
        if(dist > 25 ) { //NO HEMOS LLEGADO AL OBJETIVO
            float rot,angleSpeed;
            rot=atan2(Trobot.x(),Trobot.z()); //Calculamos la rotacion con el arcotangente
            qDebug() << "Distancia:"<<dist<< "Rot:"<<rot<<"Obstaculo:"<<laserData[50].dist;
            //Calcular velocidad
            linealSpeed = VLIN_MAX * gauss(rot,0.3, 0.5) * sinusoide(dist); //CUANTA MENOS DISTANCIA MAS RECTA ES LA LINEA
            if (laserData[25].dist<220) {  //400 tiene de ancho - 270 para no tocar nunca.
                estado=TURN;
                return linealSpeed;
            } else
                differentialrobot_proxy->setSpeedBase(linealSpeed, rot); //Movimiento
            //HAY QUE TENER EN CUENTA DOS "DISTANCIAS". La distancia con el frontal para cuando hay que detectar obstaculos
                // y la distancia con los laterales a la hora de girar solamente.
                
                
//             if (rot<0.2) {
//                 //COMPROBAMOS SI HAY OBSTACULO
//                
//                 else
//                     differentialrobot_proxy->setSpeedBase(linealSpeed, rot); //Movimiento
//             } else
//                 differentialrobot_proxy->setSpeedBase(linealSpeed, rot); //Movimiento
        } else 
            estado=END;
    } // FIN isEmpty
return 0;
}

// END
 void SpecificWorker::end(){
    qDebug() << "Estado END";
    target.setEmpty();
    estado=IDLE;
    differentialrobot_proxy->setSpeedBase(0, 0); //Parar

    
 }
// ----------------------

// TURN
 void SpecificWorker::turn(float linealSpeed, TLaserData laserData){
   std::sort( laserData.begin()+10, laserData.end()-10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return  a.dist < b.dist; }) ; 
   qDebug() << "Estado TURN"<< laserData[10].dist<<laserData[10].angle;
   if (laserData[10].angle<0)
       differentialrobot_proxy->setSpeedBase(0,0.2); 
   else 
    differentialrobot_proxy->setSpeedBase(0,-0.2); 
   //GIRAR HASTA X.
   if (abs(laserData[10].angle)>1.55 && abs(laserData[10].angle)<1.60) {
       estado=SKIRT;
   }
 }
// ----------------------

// SKIRT
 void SpecificWorker::skirt(){
   qDebug() << "Estado SKIRT";
   differentialrobot_proxy->setSpeedBase(0, 0); //Parar
 }
// ----------------------

// ----------------------
// GAUSSIANA
// VROT = VELOCIDAD ROTACION
// VX = ANGULO DE ROTACION
// H = PARAMETRO DE CORTE EN FUNCION GAUSSIANA
 float SpecificWorker::gauss(float Vrot,float Vx, float h){
    qDebug() << "vRot:" <<Vrot<< "Vx: "<< Vx<<"H:"<<h;
    float lambda = (-pow(Vx,2.0)/log(h));
    return exp(-pow(Vrot,2.0)/lambda);
 }
 
 
// SINUSOIDE
 float SpecificWorker::sinusoide(float x){
    return 1/(1+exp(-x))-0.5;
 }
 
void SpecificWorker::setPick(const Pick &myPick)
{
  qDebug() << myPick.x << myPick.z ;
  
  //TOMAR DATOS DEL MUNDO (MUAHAHA)
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);
  target.insert(myPick.x,myPick.z,bState.x,bState.z); 

}

//   float distumbral = 300;
//   float angleumbral = 0.79;
//   float rotacion=0.6;
//   bool valido=false;
//   
//   //Tomar los datos del laser
//   TLaserData data = laser_proxy->getLaserData(); 
//   //Ordenamos el array de menor a mayor
//   std::sort( data.begin()+25, data.end()-25, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ; 
//   
//   for(auto h: data)
//     qDebug() << h.angle << h.dist;
//   
//     if (data[25].dist < distumbral) {
//       qDebug()<< "Valido: "<<data[25].dist<<data[25].angle;
//       if ( (rand()%(10)) % 2 != 0)
//         rotacion=rotacion*(-1);
//         differentialrobot_proxy->setSpeedBase(5, rotacion);
//       usleep(rand()%(1500000-100000 + 1) + 100000);
//     } else  {
//     differentialrobot_proxy->setSpeedBase(1500, 0);
//  }
