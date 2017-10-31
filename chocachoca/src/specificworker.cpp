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
 
    float linealSpeed=0;
    RoboCompDifferentialRobot::TBaseState bState; 
    differentialrobot_proxy->getBaseState(bState);//TOMAR DATOS DEL MUNDO
    
    innermodel->updateTransformValues("base", bState.x,0, bState.z,0,bState.alpha, 0 );  //ACTUALIZAR ARBOL

    TLaserData laserData = laser_proxy->getLaserData(); //Tomar los datos del laser
    
    //MAQUINA DE ESTADOS
    switch (estado)  
      {  
        case IDLE: 
            idle();
            break; 
        case GOTO:
            linealSpeed=gotoTarget(bState,laserData);
            break;  
        case END:
            end();
            break;
        case TURN:
            turn(linealSpeed,laserData);
            break;
        case SKIRT:
            skirt(bState,laserData);
            break;
        default:  
	    break;
      }  
}

// --------- ESTADOS --------------
// IDLE
 void SpecificWorker::idle(){
   qDebug() << "IDLE";
   if(!target.isEmpty())
     estado=GOTO;
 }

// GOTOPICK
float SpecificWorker::gotoTarget(TBaseState bState, TLaserData laserData) {
  float dist,linealSpeed;
  qDebug() << "GOTO";
   
  std::sort( laserData.begin()+20, laserData.end()-20, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return  a.dist < b.dist; }) ;     //Ordenamos el array de menor a mayor
  std::pair <std::pair <float,float>,std::pair <float,float>> coord = target.extract(); //Tomamos las coord del pick (target y robot)
  QVec Trobot = innermodel->transform("base",QVec::vec3(coord.first.first,0,coord.first.second),"world"); //Desplaza el eje de coord del mundo al robot
  
  dist = Trobot.norm2(); //Calculamos la distancia entre los puntos
  
  if(dist > 50 ) { //NO HEMOS LLEGADO AL OBJETIVO
      float rot=atan2(Trobot.x(),Trobot.z()); //Calculamos la rotacion con el arcotangente
      qDebug() << "Distancia:"<<dist<< "Rot:"<<rot<<"Obstaculo:"<<laserData[50].dist;
      
      linealSpeed = VLIN_MAX * gauss(rot,0.3, 0.5) * sinusoide(dist); ////Calcular velocidad
      
      if (laserData[20].dist<220) {  //400 tiene de ancho - 270 para no tocar nunca.
	  estado=TURN; return linealSpeed;
      } else
	  differentialrobot_proxy->setSpeedBase(linealSpeed, rot); //Movimiento
  } else 
      estado=END;    
    
return 0;
}

// END
 void SpecificWorker::end(){
    qDebug() << "END";
    target.setEmpty();
    estado=IDLE;
    differentialrobot_proxy->setSpeedBase(0, 0); //Parar
 }

 
// TURN
 void SpecificWorker::turn(float linealSpeed, TLaserData laserData){
    std::sort( laserData.begin()+10, laserData.end()-10, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return  a.dist < b.dist; }) ; 
    qDebug() << "TURN"<< laserData[10].dist<<laserData[10].angle;
    
    if (laserData[10].angle<0) {
      differentialrobot_proxy->setSpeedBase(0,0.2); lado=false;
    } else {
      differentialrobot_proxy->setSpeedBase(0,-0.2); lado=true; 
    }
    
    if (abs(laserData[10].angle)>1.55 && abs(laserData[10].angle)<1.60) {  //GIRAR HASTA X.
       estado=SKIRT; preState=true;
    }
}


// SKIRT
 void SpecificWorker::skirt(TBaseState bState, TLaserData &laserData){
    qDebug() << "SKIRT";
    int umbralVision = 1000;
    TLaserData laserDataUnLado = laserData;
    QPolygonF polygon;
    QVec laserToWorld;
    float dist;
    
    std::pair <std::pair <float,float>,std::pair <float,float>> coord = target.extract(); //Tomamos las coord del pick (target y robot)
    QVec Trobot = innermodel->transform("base",QVec::vec3(coord.first.first,0,coord.first.second),"world"); //Desplaza el eje de coord del mundo al robot
   
    dist = Trobot.norm2(); //Calculamos la distancia entre los puntos
       
    // COMPRUEBO SI ESTOY EN TARGET
    if(dist < 330 ) { 
       estado=END; return;
    }
     
    // COMPRUEBO LA LINEA
    if (isOnLine(bState.x, bState.z)) {
        if (!preState) {
        estado=GOTO; return; }
    } else 
      preState=false;
   
    // COMPRUEBO SI VEO TARGET
    polygon << QPointF(bState.x, bState.z);  //Punto inicio poligono.
    
    int i=20; //CERCA
    if (dist > umbralVision) //LEJOS
      i=35;
    
    while (i<(100-i)) { //CREA POLIGONO
      laserToWorld = innermodel->laserTo("world", "laser", laserData[i].dist, laserData[i].angle);
      polygon << QPointF(laserToWorld.x(), laserToWorld.z());  i++;
    }
   
    pair <float,float> t =  target.getPoseTarget(); //Coor target
    
    if (polygon.containsPoint( QPointF(t.first, t.second),Qt::WindingFill )
      && polygon.containsPoint( QPointF(t.first, t.second+270),Qt::WindingFill )
      && polygon.containsPoint( QPointF(t.first, t.second-270),Qt::WindingFill )
      && polygon.containsPoint( QPointF(t.first-270, t.second),Qt::WindingFill )
      && polygon.containsPoint( QPointF(t.first+270, t.second),Qt::WindingFill )
    ) { //COMPROBACION COORS EN POLIGONO
      estado=GOTO; return;
    }     

    //SI LLEGO AQUI --> BORDEAR
    int vInicio=51;
    int vFinal=10;

    if (lado) {
      int aux=vInicio;
      vInicio=vFinal;
      vFinal=aux;
    }
    
    //ORDENACION SOLO DEL LADO A BORDEAR
    std::sort( laserDataUnLado.begin()+vInicio, laserDataUnLado.end()-vFinal, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return  a.dist < b.dist; }) ;  
    
    if (laserDataUnLado[vInicio].dist<340) 
      differentialrobot_proxy->setSpeedBase(20,0.3); 
    else if  (laserDataUnLado[vInicio].dist>450) 
      differentialrobot_proxy->setSpeedBase(20,-0.3); 
    else
      differentialrobot_proxy->setSpeedBase(100,0);
   

}
 
 
 bool SpecificWorker::isOnLine(float x, float z){
    pair <float,float> coorsT =  target.getPoseTarget();
    pair <float,float> coorsI =  target.getPoseRobot();
    float resul=abs( ((coorsT.second-coorsI.second)*(x-coorsI.first)) - ((coorsT.first-coorsI.first)*(z-coorsI.second)) );
    if ( resul < 20000.0 )
      return true;
    else
      return false; 
    
}
 
// ----------------------

// ----------------------
// GAUSSIANA
// VROT = VELOCIDAD ROTACION
// VX = ANGULO DE ROTACION
// H = PARAMETRO DE CORTE EN FUNCION GAUSSIANA
float SpecificWorker::gauss(float Vrot,float Vx, float h){
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
  
  //TOMAR DATOS DEL MUNDO
  RoboCompDifferentialRobot::TBaseState bState;
  differentialrobot_proxy->getBaseState(bState);
  target.insert(myPick.x,myPick.z,bState.x,bState.z); 
  estado=GOTO;
}

