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
    innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/betaWorldArm.xml");
    timer.start(Period);
    return true;
}

void SpecificWorker::compute() {
    float linealSpeed=0;
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);     //Tomar datos base
    innermodel->updateTransformValues("robot", bState.x,0, bState.z,0,bState.alpha, 0 );     //Actualizar arbol

    TLaserData laserData = laser_proxy->getLaserData();     //Tomar los datos del laser

    //MAQUINA DE ESTADOS
    switch (estado) {
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
    case ARM:
	arm();
	break;
    default:
        break;
    }
}

// --------- ESTADOS --------------
// IDLE
void SpecificWorker::idle() {
    qDebug() << "IDLE";
    if(!target.isEmpty())
        estado=GOTO;
}

// GOTO
float SpecificWorker::gotoTarget(TBaseState bState, TLaserData laserData) {
    qDebug() << "GOTO";

    float dist,linealSpeed;
    std::sort( laserData.begin()+12, laserData.end()-12, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    });                                                                                                                                   //Ordenamos el array de menor a mayor
    std::pair <std::pair <float,float>,std::pair <float,float> > coord = target.extract();     //Tomamos las coord del pick (target y robot)
    QVec Trobot = innermodel->transform("robot",QVec::vec3(coord.first.first,0,coord.first.second),"world");     //Desplaza el eje de coord del mundo al robot

    dist = Trobot.norm2();     //Calcular la distancia entre los puntos

    if(dist > thresholdValues.first ) {     //No se ha alzancado objetivo
        float rot=atan2(Trobot.x(),Trobot.z());         //Calculamos la rotacion con el arcotangente
        linealSpeed = VLIN_MAX * gauss(rot,0.3, 0.4) * sinusoide(dist);         ////Calcular velocidad

        if (laserData[12].dist<thresholdValues.first) {         //400 tiene de ancho - 270 para no tocar nunca.
            estado=TURN;
            return linealSpeed;
        } else
            differentialrobot_proxy->setSpeedBase(linealSpeed, rot);             //Movimiento
    } else 
        estado=END; //Objetivo alcanzado
    
    return 0;
}

// END
void SpecificWorker::end() {
    qDebug() << "END";
    target.setEmpty();
    estado=IDLE;
    differentialrobot_proxy->setSpeedBase(0, 0);     //Parar
    pick = false;     /* Condicion pick=false permite continuar aprilTagsMaster */
}


// TURN
void SpecificWorker::turn(float linealSpeed, TLaserData laserData) {
     qDebug() << "TURN";
    std::sort( laserData.begin()+10, laserData.end()-10, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    });

    if (laserData[10].angle<0) {
        differentialrobot_proxy->setSpeedBase(0,0.2);
        lado=false;
    } else {
        differentialrobot_proxy->setSpeedBase(0,-0.2);
        lado=true;
    }

    if (abs(laserData[10].angle)>1.49 && abs(laserData[10].angle)<1.61)
        estado=SKIRT;
    preState=true;
}


// SKIRT
void SpecificWorker::skirt(TBaseState bState, TLaserData &laserData) {
    qDebug() << "SKIRT";
    TLaserData laserDataUnLado = laserData;
    float dist=0.0;

    std::pair <std::pair <float,float>,std::pair <float,float> > coord = target.extract();     //Tomamos las coord del pick (target y robot)
    QVec Trobot = innermodel->transform("robot",QVec::vec3(coord.first.first,0,coord.first.second),"world");     //Desplaza el eje de coord del mundo al robot
    dist = Trobot.norm2();
    

    // COMPRUEBO SI ESTOY EN TARGET
    if (onTarget(dist)) return;

    // COMPRUEBO LA LINEA
    if (isOnLine(bState)) return;
    
    // COMPRUEBO SI VEO TARGET
    if (reachableTarget(bState,dist,laserData)) return;
    
    // BORDEAR
    int vInicio=51;
    int vFinal=10;
    float ladoRot=0.2;
    int speed=20;
    int maxSpeed=100;

    if (lado) {
        int aux=vInicio;
        vInicio=vFinal;
        vFinal=aux;
        ladoRot=-(ladoRot);
    }
    
    //ORDENACION SOLO DEL LADO A BORDEAR
    std::sort( laserDataUnLado.begin()+vInicio, laserDataUnLado.end()-vFinal, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    });


    if (laserDataUnLado[vInicio].dist<thresholdValues.first) 
        differentialrobot_proxy->setSpeedBase(speed,ladoRot); 
    else if  (laserDataUnLado[vInicio].dist>thresholdValues.second) 
        differentialrobot_proxy->setSpeedBase(speed,-(ladoRot));
    else 
        differentialrobot_proxy->setSpeedBase(maxSpeed,0);
}


void SpecificWorker::arm(){
  
}
// ----------------------

/*
/ Auxiliary methods
*/


/* Check if robot is on target */
bool SpecificWorker::onTarget(float dist) {
    if(dist < thresholdValues.first ) {
        estado=END;
        return true;
    }
    return false;
}

/* Check if robot is on line */
bool SpecificWorker::isOnLine(TBaseState bState) {
    pair <float,float> coorsT =  target.getPoseTarget();
    pair <float,float> coorsI =  target.getPoseRobot();
    float resul=abs( ((coorsT.second-coorsI.second)*(bState.x-coorsI.first)) - ((coorsT.first-coorsI.first)*(bState.z-coorsI.second)) );
    resul = (resul / sqrt(pow(coorsT.second - coorsI.second, 2.0) + pow(coorsT.first - coorsI.first, 2.0)));
    //qDebug()<<"ISONLINE RESULT:"<<resul;

    if ( resul <= 20 ) {
        if (!preState) {
            estado=GOTO; return true;
        }
    } else
        preState=false;
    return false;
    
}


/* Check if target is reachable  */
bool SpecificWorker::reachableTarget(TBaseState bState, float dist, TLaserData &laserData) {
    QVec laserToWorld;
    QPolygonF polygon;
    int umbralVision = 1000;
    int anchoPunto=100;

    polygon << QPointF(bState.x, bState.z);     //Punto inicio poligono.

    int i=30;     //CERCA
    if (dist > umbralVision)     //LEJOS
        i=40;
    int finLaser=100-i;
    qDebug() <<"Dist"<<dist<<"ANGULO:"<< i << finLaser;
    while (i<finLaser) {     //CREA POLIGONO
        laserToWorld = innermodel->laserTo("world", "laser", laserData[i].dist, laserData[i].angle);
        polygon << QPointF(laserToWorld.x(), laserToWorld.z());
        i++;
    }

    pair <float,float> t =  target.getPoseTarget();    //Coor target
    if (	polygon.containsPoint( QPointF(t.first, t.second),Qt::WindingFill )
            && polygon.containsPoint( QPointF(t.first+anchoPunto, t.second+anchoPunto),Qt::WindingFill )
            && polygon.containsPoint( QPointF(t.first+anchoPunto, t.second-anchoPunto),Qt::WindingFill )
            && polygon.containsPoint( QPointF(t.first-anchoPunto, t.second+anchoPunto),Qt::WindingFill )
            && polygon.containsPoint( QPointF(t.first-anchoPunto, t.second-anchoPunto),Qt::WindingFill )
       ) {     //COMPROBACION COORS EN POLIGONO
	 qDebug() << "POLIGONO JODER";
        estado=GOTO;
        return true;
    }
    return false;
    
//     if (	polygon.containsPoint( QPointF(t.first, t.second),Qt::WindingFill )
//             && polygon.containsPoint( QPointF(t.first, t.second+anchoPunto),Qt::WindingFill )
//             && polygon.containsPoint( QPointF(t.first, t.second-anchoPunto),Qt::WindingFill )
//             && polygon.containsPoint( QPointF(t.first-anchoPunto, t.second),Qt::WindingFill )
//             && polygon.containsPoint( QPointF(t.first+anchoPunto, t.second),Qt::WindingFill )
//        ) {     //COMPROBACION COORS EN POLIGONO
// 	 qDebug() << "POLIGONO JODER";
//         estado=GOTO;
//         return true;
//     }
//     return false;
}


/* Gaussian
/ VROT = Rotation speed
/ VX = Angle Speed
/ H = Parameter of cut in Gaussian function
*/
float SpecificWorker::gauss(float Vrot,float Vx, float h) {
    float lambda = (-pow(Vx,2.0)/log(h));
    return exp(-pow(Vrot,2.0)/lambda);
}


/* Sinusoid */
float SpecificWorker::sinusoide(float x) {
    return 1/(1+exp(-x))-0.5;
}


/*
/ Calls to the interface
*/

/* Go to target if there isnt pick */
void SpecificWorker::go(const float x, const float z) {
    while (pick) {
        /* No hace nada */
    }
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    target.insert(x,z,bState.x,bState.z);
    estado=GOTO;
}

/* Just Turn on itself */
void SpecificWorker::turn(const float speed) {
    estado=IDLE;
    differentialrobot_proxy->setSpeedBase(0,speed);
}

/* Return true if robot is working, otherwise false. */
bool SpecificWorker::getState() {
    if (estado==IDLE)
        return false;
    else
        return true;
}

/* Stop the robot */
void SpecificWorker::stop() {
    differentialrobot_proxy->setSpeedBase(0,0);
}

/* New pick */
void SpecificWorker::setPick(const Pick &myPick)
{
    qDebug() << myPick.x << myPick.z;
    pick = true;

    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    target.insert(myPick.x,myPick.z,bState.x,bState.z);
    estado=GOTO;
}
