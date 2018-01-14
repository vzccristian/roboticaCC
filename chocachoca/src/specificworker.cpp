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
    // Innermodel load file
    innermodel = new InnerModel("/home/robocomp/robocomp/components/roboticaCC/misc/betaWorldArm3.xml");
    
    // Motors
    joints << "shoulder_right_1"<<"shoulder_right_2"<<"shoulder_right_3"<<"elbow_right"<<"wrist_right_1"<<"wrist_right_2";
    motores = QVec::zeros(joints.size());
    
    //Flags
    picked = nearToBox = false;
    
    setDefaultArmPosition(true);
    
    timer.start(Period);
    return true;
}


// Compute 
void SpecificWorker::compute() {
    float linealSpeed=0;
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);     //Tomar datos base
    innermodel->updateTransformValues("robot", bState.x,0, bState.z,0,bState.alpha, 0 );     //Actualizar arbol

    TLaserData laserData = laser_proxy->getLaserData();     //Tomar los datos del laser
    updateJoints();
    try{
      newAprilTag(getapriltags_proxy->checkMarcas());
    } catch(const Ice::Exception &e) {
      std::cout << e <<endl;
    }
    
    //MAQUINA DE ESTADOS
    switch (estado) {
    case IDLE:
        idle();
        break;
    case GOTO:
        linealSpeed=gotoTarget(bState,laserData);
        break;
    case TURN:
        turn(linealSpeed,laserData);
        break;
    case SKIRT:
        skirt(bState,laserData);
        break;
    case PICK:
        pickingBox();
        break;
    case RELEASE:
        releasingBox();
        break;
    case HAND_WATCHING_BOX:
        handWatchingBox();
        break;
    case END:
        end();
        break;
    default:
        break;
    }
    qDebug() << "fuera";
}


/*
// --------- STATES -------------- //
*/

// IDLE - Main method 
void SpecificWorker::idle() {
    qDebug() << "IDLE";
    if(!target.isEmpty())
        estado=GOTO;
}

// GOTO - Main method 
float SpecificWorker::gotoTarget(TBaseState bState, TLaserData laserData) {
    qDebug() << "GOTO";

    float dist,linealSpeed;
    std::sort( laserData.begin()+12, laserData.end()-12, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;
    });                                                                                                                                   
    
    std::pair <std::pair <float,float>,std::pair <float,float> > coord = target.extract();     //Tomamos las coord del pick (target y robot)
    QVec Trobot = innermodel->transform("robot",QVec::vec3(coord.first.first,0,coord.first.second),"world");     //Desplaza el eje de coord del mundo al robot

    dist = Trobot.norm2();     //Calcular la distancia entre los puntos

    if(dist > thresholdValues.first ) {     //No se ha alzancado objetivo
        float rot=atan2(Trobot.x(),Trobot.z());         //Calculamos la rotacion con el arcotangente
        linealSpeed = VLIN_MAX * gauss(rot,0.3, 0.4) * sinusoide(dist);         ////Calcular velocidad

        //         if (laserData[12].dist<thresholdValues.first) {         //400 tiene de ancho - 270 para no tocar nunca.
        //             
        //             
        /////////////////////////////////////////////////////////////////
        //             OJOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
        //             estado=TURN;
        //////////////////////////////////////////////////////////////////
        //             return linealSpeed;
        //         } else
            differentialrobot_proxy->setSpeedBase(linealSpeed, rot);             //Movimiento
    } else 
        estado=END; //Objetivo alcanzado
    
    return 0;
}


// GOTO - Auxiliary methods

/* 
    Gaussian
    @vrot Rotation speed
    @vx Angle Speed
    @h Parameter of cut in Gaussian function
    @return float gauss operation
*/
float SpecificWorker::gauss(float vrot,float vx, float h) {
    float lambda = (-pow(vx,2.0)/log(h));
    return exp(-pow(vrot,2.0)/lambda);
}


/* /
    Sinusoid 
*/
float SpecificWorker::sinusoide(float x) {
    return 1/(1+exp(-x))-0.5;
}

void SpecificWorker::handWatchingBox() {
    sleep(1);
}


// TURN - Main method

void SpecificWorker::turn(float linealSpeed, TLaserData laserData) {
     qDebug() << "TURN";
    std::sort( laserData.begin()+10, laserData.end()-10, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist; });

    if (laserData[10].angle<0) {
        differentialrobot_proxy->setSpeedBase(0,0.2);
        side=false;
    } else {
        differentialrobot_proxy->setSpeedBase(0,-0.2);
        side=true;
    }

    if (abs(laserData[10].angle)>1.49 && abs(laserData[10].angle)<1.61)
        estado=SKIRT;
    preState=true;
}


// SKIRT - Main method

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
    int vInicio=51, vFinal=10, speed=20, maxSpeed=100;
    float ladoRot=0.2;

    if (side) {
        int aux=vInicio;
        vInicio=vFinal;
        vFinal=aux;
        ladoRot=-(ladoRot);
    }
    
    //ORDENACION SOLO DEL LADO A BORDEAR
    std::sort( laserDataUnLado.begin()+vInicio, laserDataUnLado.end()-vFinal, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
        return a.dist < b.dist;});

    if (laserDataUnLado[vInicio].dist<thresholdValues.first) 
        differentialrobot_proxy->setSpeedBase(speed,ladoRot); 
    else if  (laserDataUnLado[vInicio].dist>thresholdValues.second) 
        differentialrobot_proxy->setSpeedBase(speed,-(ladoRot));
    else 
        differentialrobot_proxy->setSpeedBase(maxSpeed,0);
}



// SKIRT - Auxiliary methods


/* 
    Check if robot is on target 
*/
bool SpecificWorker::onTarget(float dist) {
    if(dist < thresholdValues.first ) {
        estado=END;
        return true;
    }
    return false;
}

/* 
    Check if robot is on line
*/
bool SpecificWorker::isOnLine(TBaseState bState) {
    pair <float,float> coorsT =  target.getPoseTarget();
    pair <float,float> coorsI =  target.getPoseRobot();
    float resul=abs( ((coorsT.second-coorsI.second)*(bState.x-coorsI.first)) - ((coorsT.first-coorsI.first)*(bState.z-coorsI.second)) );
    resul = (resul / sqrt(pow(coorsT.second - coorsI.second, 2.0) + pow(coorsT.first - coorsI.first, 2.0)));

    if ( resul <= 20 ) {
        if (!preState) {
            estado=GOTO; return true;
        }
    } else
        preState=false;
    
    return false;
}

/* 
    Check if target is reachable 
*/
bool SpecificWorker::reachableTarget(TBaseState bState, float dist, TLaserData &laserData) {
    QVec laserToWorld;
    QPolygonF polygon;
    int umbralVision = 1000, anchoPunto=100;

    polygon << QPointF(bState.x, bState.z);     //Punto inicio poligono.

    int i=30;     //CERCA
    if (dist > umbralVision)     //LEJOS
        i=40;
    int finLaser=100-i;
    
    while (i<finLaser) {  
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
       ) {    
        estado=GOTO;
        return true;
    }
    
    return false;
}


// PICK - Main method

void SpecificWorker::pickingBox() {
    estado = PICK;
    if (!picked) { 
        qDebug() << "TX: "<<targetBox.tx<<" -- TY: "<<targetBox.ty<<" -- TZ: "<<targetBox.tz<< " ....... RX: "
            <<targetBox.rx<< " -- RY: "<<targetBox.ry<< " -- RZ: "<<targetBox.rz;
        
        error = QVec::vec6(0,0,0,0,0,0);
        
        if (targetBox.tz < 105) {
            nearToBox = true;
            error += QVec::vec6(0,0,-LINEAL_INCREMENT*6,0,0);
        } else { 
            fixPosition();
            fixRotation();
        }
        
        //error.print("MOVIMIENTO DE LOS MOTORES");
        adjustMotors();
    } else {
        nearToBox=false;
        stopMotors();
        prepareToMove();
        estado=IDLE;
        qDebug() << "mandoIDLE";
    }
    
    if (nearToBox) {
        picked = true;
        sleep(1); //Sleep para esperar a bajar
        qDebug() << "picked -------->" << picked;

    }
    
    targetBox={targetBox.id,0,0,0,0,0,0};
    
}

// PICK - Auxiliary methods

void SpecificWorker::fixPosition() {
    //MOVIMIENTO RESPECTO A EJE X
    if ( targetBox.tx > 1)
        error += QVec::vec6(-LINEAL_INCREMENT,0,0,0,0,0);
    else if  (targetBox.tx < -1)
        error += QVec::vec6(LINEAL_INCREMENT,0,0,0,0,0);
    
    //MOVIMIENTO RESPECTO A EJE Y
    if ( targetBox.ty > 1)
        error += QVec::vec6(0,-LINEAL_INCREMENT,0,0,0,0);
    else if  (targetBox.ty < -1)
        error += QVec::vec6(0,LINEAL_INCREMENT,0,0,0,0);
    
    //ALTURA
    if ( targetBox.tz > 105)
        error += QVec::vec6(0,0,-LINEAL_INCREMENT*1.5,0,0,0);
    
}
void SpecificWorker::fixRotation() {
    if (targetBox.tz > 150) {
        if (( targetBox.rz > -PI/2) && (targetBox.rz < -PI/4)) {    //A. GIRA ANTI-HORARIO
            qDebug() << "RZ-A [>-PI/2 y -PI/4]: "<< targetBox.rz;
            error += QVec::vec6(0,0,0,0,0,targetBox.rz/ANGULAR_PROP); 
        } else if (( targetBox.rz > PI/4) && (targetBox.rz < PI/2)) { //B. GIRA HORARIO
            qDebug() << "RZ-B [PI/4 y <PI/2]: "<< targetBox.rz;
            error += QVec::vec6(0,0,0,0,0,targetBox.rz/ANGULAR_PROP); 
        } else if (targetBox.rz > 0 && targetBox.rz < PI/4) { //C. GIRA ANTI-HORARIO
            qDebug() << "RZ-C [>0 y <PI/4]: "<< targetBox.rz;
            error += QVec::vec6(0,0,0,0,0,-targetBox.rz/ANGULAR_PROP);
        } else if (( targetBox.rz > -PI/4) && (targetBox.rz < 0)) { //D. GIRA HORARIO
            qDebug() << "RZ-D [>-PI/4 Y <0]: "<< targetBox.rz;
            error += QVec::vec6(0,0,0,0,0,-targetBox.rz/ANGULAR_PROP);
        } else {
            qDebug() << "RZ [------------------------->]: "<< targetBox.rz;
        }
    }
}

void SpecificWorker::adjustMotors() {
    bool jacobianWorks=true;
    QMat jacobian = innermodel->jacobian(joints, motores, "rgbdHand");
    RoboCompJointMotor::MotorGoalVelocityList vl;
    
    try {	
        QVec incs = jacobian.invert() * error; //Vector de incrementos de velocidad
        //incs.print("INCREMENTOS");
        
        int i=0;
        for(auto m: joints) {
            RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
            vl.push_back(vg);
            i++;
        }
    } catch(const QString &e){ 
        jacobianWorks=false;
        qDebug() << e << "Error inverting matrix";
    }
    
    if (jacobianWorks) {
        try {
            jointmotor_proxy->setSyncVelocity(vl);
        } catch(const Ice::Exception &e) {
            std::cout << e.what() << std::endl; }
    }
}

void SpecificWorker::stopMotors() {
    RoboCompJointMotor::MotorGoalVelocityList vl;
    for(auto m: joints){
        RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
        vl.push_back(vg);
    }
    try {
        jointmotor_proxy->setSyncVelocity(vl);
    } catch(const Ice::Exception &e) {
        std::cout << e.what() << std::endl; }
    
}

bool SpecificWorker::prepareToMove() {
    //CERRAR DEDOS
    RoboCompJointMotor::MotorGoalPositionList pFingers;
    pFingers.push_back(RoboCompJointMotor::MotorGoalPosition{-FACTOR_FINGERS, 1.0,"finger_right_1"});
    pFingers.push_back(RoboCompJointMotor::MotorGoalPosition{FACTOR_FINGERS, 1.0,"finger_right_2"});
    jointmotor_proxy->setSyncPosition(pFingers);
    
    //PASAR A LA MANO //TODO
    //         string caja = "C"+std::to_string(targetBox.id);
    //         for (int i=1; i<7; i++) {
    //             try {	
    //                 if(innermodelmanager_proxy->collide(string("finger_right_1_mesh4"),caja+"_"+std::to_string(i)))
    //                     qDebug() << "Collide" << i;
    //                 else
    //                     qDebug() << "NO Collide" << i;
    //             }catch(const Ice::Exception &e){
    //                 std::cout << e << std::endl;
    //             }
    //         }
    
    
    //LEVANTAR BRAZO
    sleep(1); //Esperar a cerrar dedos
    setDefaultArmPosition(false);
    sleep(1);
}

void SpecificWorker::setDefaultArmPosition(bool init) {
    // Set positions
    
    RoboCompJointMotor::MotorGoalPosition wrist_right, elbow_right, shoulder_right_2, finger_right_1, finger_right_2;
    
    wrist_right.name = "wrist_right_2";
	wrist_right.position = 1.2;
	wrist_right.maxSpeed = 1;
	
	elbow_right.name = "elbow_right";
	elbow_right.position = 1;
	elbow_right.maxSpeed = 1;
	
	shoulder_right_2.name = "shoulder_right_2";
	shoulder_right_2.position = -0.785398;
	shoulder_right_2.maxSpeed = 1;
    
    jointmotor_proxy->setPosition(wrist_right);
	jointmotor_proxy->setPosition(elbow_right);
	jointmotor_proxy->setPosition(shoulder_right_2);
    
    if (init) {
        finger_right_1.name = "finger_right_1";
        finger_right_1.position = 0.0;
        finger_right_1.maxSpeed = 1;
        
        finger_right_2.name = "finger_right_2";
        finger_right_2.position = 0.0;
        finger_right_2.maxSpeed = 1;
        
        jointmotor_proxy->setPosition(finger_right_1);
        jointmotor_proxy->setPosition(finger_right_2);
    }
}

// RELEASE - Main method
void SpecificWorker::releasingBox()
{
  qDebug() << "RELEASING BOX";
  estado=RELEASE;
  
}


// END - Main method
void SpecificWorker::end() {
    qDebug() << "END";
    target.setEmpty();
    estado=IDLE;
    differentialrobot_proxy->setSpeedBase(0, 0);     //Parar
    pick = false;     /* Condicion pick=false permite continuar aprilTagsMaster */
}




/*
// --------- Calls to our own interface -------------- //
*/

/* 
    Go to target if there isnt pick
*/
void SpecificWorker::go(const float x, const float z) {
    while (pick) {
        qDebug() << "Waiting for userpick";
        sleep(1);
    }
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    target.insert(x,z,bState.x,bState.z);
    estado=GOTO;
}

/* 
    Just Turn on itself
*/
void SpecificWorker::turn(const float speed) {
    estado=IDLE;
    differentialrobot_proxy->setSpeedBase(0,speed);
}

/* 
    Return true if robot is working, otherwise false. 
*/
string SpecificWorker::getState() {
    switch (estado) {
        case IDLE:
            return "IDLE";
        case GOTO:
            return "GOTO";
        case HAND_WATCHING_BOX:
            return "HAND_WATCHING_BOX";
        case TURN:
            return "TURN";
        case SKIRT:
            return "SKIRT";
        case PICK:
            return "PICK";
        case RELEASE:
            return "RELEASE";
        case END:
            return "END";
        default:
            return "ERROR";
    }
}

/* 
    Stop the robot 
*/
void SpecificWorker::stop() {
    estado=END;
}

/*
    New pick
*/
void SpecificWorker::setPick(const Pick &myPick)
{
    qDebug() << myPick.x << myPick.z;
    pick = true;

    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    target.insert(myPick.x,myPick.z,bState.x,bState.z);
    estado=GOTO;
}


void SpecificWorker::updateJoints()
{
 	try
	{
		RoboCompJointMotor::MotorStateMap mMap;
		jointmotor_proxy->getAllMotorState(mMap);
		for(auto m: mMap){
			innermodel->updateJointValue(QString::fromStdString(m.first),m.second.pos);
		}
	}
	catch(const Ice::Exception &e)
	{	std::cout << e.what() << std::endl;}
	
} 

//HAND 
void SpecificWorker::newAprilTag(const RoboCompGetAprilTags::listaMarcas &tags)
{
    int i;
    if (estado != PICK) { //NO OBJETIVO SELECCIONADO
        for (i=0; i<(signed)tags.size(); i++) {
            if (tags[i].id > 9 && estado != PICK) {
            differentialrobot_proxy->setSpeedBase(0, 0);   
            targetBox=tags[i];
            estado=HAND_WATCHING_BOX;
            }
        }
    } else { //OBJETIVO SELECCIONADO, SOLO ACTUALIZO ESE
        for (i=0; i<(signed)tags.size(); i++) {
            if (tags[i].id == targetBox.id) 
                targetBox=tags[i]; //Actualizo
            }
    }
  
  
}


