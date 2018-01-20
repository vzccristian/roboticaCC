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
        state = IDLE;
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
        engines = QVec::zeros(joints.size());

        //Flags
        picked = nearToBox = pick = false;
        handCamera = preState = true;

        thresholdValues=make_pair(270,470);
        setDefaultArmPosition(true);

        timer.start(Period);
        return true;
}


// Compute
void SpecificWorker::compute() {
        float linealSpeed=0;
        RoboCompDifferentialRobot::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        innermodel->updateTransformValues("robot", bState.x,0, bState.z,0,bState.alpha, 0 ); // Update tree

        TLaserData laserData = laser_proxy->getLaserData(); // Get laser data

        updateJoints(); // Update joints

        try {
                newAprilTag(getapriltags_proxy->checkMarcas());
        } catch(const Ice::Exception &e) {
                std::cout << e <<endl;
        }

        qDebug() << QString::fromStdString(getState()); // Show current state

        //States machine
        switch (state) {
            case IDLE: //Idle
                    idle();
                    break;
            case GOTO: //Go to target
                    linealSpeed=gotoTarget(bState,laserData);
                    break;
            case TURN: //Rotation to avoid obstacles
                    turn(linealSpeed,laserData);
                    break;
            case SKIRT: //Avoid obstacles
                    skirt(bState,laserData);
                    break;
            case PICK: //Pick box
                    pickingBox();
                    break;
            case HAND_WATCHING_BOX: //Arm is watching box
                    handWatchingBox();
                    break;
            case END: //Target reached
                    end();
                    break;
            default:
                    break;
        }

}


/*
   // --------- STATES -------------- //
 */

// IDLE - Main method
void SpecificWorker::idle() {
        if(!target.isEmpty())
            state = GOTO;
}

// GOTO - Main method
// @Warning: Avoid obstacles is commented
float SpecificWorker::gotoTarget(TBaseState bState, TLaserData laserData) {
        float dist,linealSpeed;
        /* std::sort( laserData.begin()+12, laserData.end()-12, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
                return a.dist < b.dist;
        });
        */
        std::pair <std::pair <float,float>,std::pair <float,float> > coord = target.extract(); //Take origin and destination coordinates
        QVec Trobot = innermodel->transform("robot",QVec::vec3(coord.first.first,0,coord.first.second),"world"); //Move the coordinate axis

        dist = Trobot.norm2(); //Calculate the distance between the points

        if(dist > thresholdValues.first ) { //Objective not reached. (Distance greater than the threshold)
                float rot=atan2(Trobot.x(),Trobot.z()); //Calculate rotation
                linealSpeed = VLIN_MAX * gauss(rot,0.3, 0.4) * sinusoide(dist); //Calculate speed
                /*      if (laserData[12].dist<thresholdValues.first) { //Minimum values to avoid touching obstacles
                           state=TURN;
                          return linealSpeed;
                        } else
                */
                differentialrobot_proxy->setSpeedBase(linealSpeed, rot); //Movement
        } else
                state=END; //Objetivo alcanzado
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


// TURN - Main method

void SpecificWorker::turn(float linealSpeed, TLaserData laserData) {
        std::sort( laserData.begin()+10, laserData.end()-10, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
                return a.dist < b.dist;
        });

        if (laserData[10].angle<0) {
                differentialrobot_proxy->setSpeedBase(0,0.2);
                side=false;
        } else {
                differentialrobot_proxy->setSpeedBase(0,-0.2);
                side=true;
        }

        if (abs(laserData[10].angle)>1.49 && abs(laserData[10].angle)<1.61)
                state=SKIRT;
        preState=true;
}


// SKIRT - Main method

void SpecificWorker::skirt(TBaseState bState, TLaserData &laserData) {
        TLaserData oneSideLaserData = laserData;
        float dist=0.0;

        std::pair <std::pair <float,float>,std::pair <float,float> > coord = target.extract(); //Tomamos las coord del pick (target y robot)
        QVec Trobot = innermodel->transform("robot",QVec::vec3(coord.first.first,0,coord.first.second),"world"); //Desplaza el eje de coord del mundo al robot
        dist = Trobot.norm2();

        if (onTarget(dist)) return; // Check if objective has been achieved

        if (isOnLine(bState)) return; //Check if I'm on the imaginary line that links origin and objective

        if (reachableTarget(bState,dist,laserData)) return; //Check if target is reachable from my current position

        // At this point, continue skirting the obstacle.
        int startPosition=51, endPosition=10, speed=20, maxSpeed=100;
        float sideRot=0.2;

        if (side) {
                int aux=startPosition;
                startPosition=endPosition;
                endPosition=aux;
                sideRot=-(sideRot);
        }

        // Sort only the side to border
        std::sort( oneSideLaserData.begin()+startPosition, oneSideLaserData.end()-endPosition, [] (RoboCompLaser::TData a, RoboCompLaser::TData b) {
                return a.dist < b.dist;
        });

        if (oneSideLaserData[startPosition].dist<thresholdValues.first)
                differentialrobot_proxy->setSpeedBase(speed,sideRot);
        else if  (oneSideLaserData[startPosition].dist>thresholdValues.second)
                differentialrobot_proxy->setSpeedBase(speed,-(sideRot));
        else
                differentialrobot_proxy->setSpeedBase(maxSpeed,0);
}



// SKIRT - Auxiliary methods

/*
    Check if robot is on target
 */
bool SpecificWorker::onTarget(float dist) {
        if(dist < thresholdValues.first ) {
                state=END;
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
                        state=GOTO; return true;
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
        int visionThreshold = 1000, widthThreshold=100;

        polygon << QPointF(bState.x, bState.z); // Start point of the polygon

        int i=30; // Near
        if (dist > visionThreshold) // Far
                i=40;
        int maxLaser=100-i;

        while (i<maxLaser) {
                laserToWorld = innermodel->laserTo("world", "laser", laserData[i].dist, laserData[i].angle);
                polygon << QPointF(laserToWorld.x(), laserToWorld.z());
                i++;
        }

        pair <float,float> t =  target.getPoseTarget(); // Coors target
        if (  polygon.containsPoint( QPointF(t.first, t.second),Qt::WindingFill )
              && polygon.containsPoint( QPointF(t.first+widthThreshold, t.second+widthThreshold),Qt::WindingFill )
              && polygon.containsPoint( QPointF(t.first+widthThreshold, t.second-widthThreshold),Qt::WindingFill )
              && polygon.containsPoint( QPointF(t.first-widthThreshold, t.second+widthThreshold),Qt::WindingFill )
              && polygon.containsPoint( QPointF(t.first-widthThreshold, t.second-widthThreshold),Qt::WindingFill )
              ) {
                state=GOTO;
                return true;
        }

        return false;
}

// HAND_WATCHING_BOX - Main method

void SpecificWorker::handWatchingBox() {
        sleep(0.1);
}


// PICK - Main method

void SpecificWorker::pickingBox() {
        state = PICK;
        if (!picked) {
                error = QVec::vec6(0,0,0,0,0,0);

                if (targetBox.tz < 105) {
                        nearToBox = true;
                        error += QVec::vec6(0,0,-LINEAL_INCREMENT*6,0,0);
                } else {
                        fixPosition();
                        fixRotation();
                }

                //TODO
                if (!nearToBox && targetBox.tz==0 && targetBox.ty == 0 && targetBox.tx == 0)  //No la veo
                  setDefaultArmPosition(true);

                adjustMotors();
        } else {
                stopMotors();
                prepareToMove();
                state=END;
                nearToBox=handCamera=picked=false;
        }

        if (nearToBox) {
                picked = true;
                sleep(1); //Sleep to wait for the arm to come down
        }

        targetBox={targetBox.id,0,0,0,0,0,0};
}

// PICK - Auxiliary methods

void SpecificWorker::fixPosition() {
        // Adjust X axis
        if ( targetBox.tx > 1)
                error += QVec::vec6(-LINEAL_INCREMENT,0,0,0,0,0);
        else if  (targetBox.tx < -1)
                error += QVec::vec6(LINEAL_INCREMENT,0,0,0,0,0);

        // Adjust Y axis
        if ( targetBox.ty > 1)
                error += QVec::vec6(0,-LINEAL_INCREMENT,0,0,0,0);
        else if  (targetBox.ty < -1)
                error += QVec::vec6(0,LINEAL_INCREMENT,0,0,0,0);

        // Adjust height
        if ( targetBox.tz > 105)
                error += QVec::vec6(0,0,-LINEAL_INCREMENT*1.5,0,0,0);

}

void SpecificWorker::fixRotation() {
        if (targetBox.tz > 150) {
                if ((targetBox.rz > -PI/2 && targetBox.rz < -PI/4) ||
                    (targetBox.rz > PI/4 && targetBox.rz < PI/2))
                        error += QVec::vec6(0,0,0,0,0,targetBox.rz/ANGULAR_PROP);
                else if ((targetBox.rz > 0 && targetBox.rz < PI/4) ||
                         (targetBox.rz > -PI/4 && targetBox.rz < 0))
                              error += QVec::vec6(0,0,0,0,0,-targetBox.rz/ANGULAR_PROP);
        }
}

void SpecificWorker::adjustMotors() {
        bool jacobianWorks=true;
        QMat jacobian = innermodel->jacobian(joints, engines, "rgbdHand");
        RoboCompJointMotor::MotorGoalVelocityList vl;

        try {
                QVec incs = jacobian.invert() * error; // Speed increment array
                int i=0;
                for(auto m: joints) {
                        RoboCompJointMotor::MotorGoalVelocity vg{FACTOR*incs[i], 1.0, m.toStdString()};
                        vl.push_back(vg);
                        i++;
                }
        } catch(const QString &e) {
                jacobianWorks=false;
                qDebug() << "Error inverting matrix";
        }

        if (jacobianWorks) {
                try {
                        jointmotor_proxy->setSyncVelocity(vl);
                } catch(const Ice::Exception &e) {
                        std::cout << e.what() << std::endl;
                }
        }
}

void SpecificWorker::stopMotors() {
        RoboCompJointMotor::MotorGoalVelocityList vl;
        for(auto m: joints) {
                RoboCompJointMotor::MotorGoalVelocity vg{0.0, 1.0, m.toStdString()};
                vl.push_back(vg);
        }
        try {
                jointmotor_proxy->setSyncVelocity(vl);
        } catch(const Ice::Exception &e) {
                std::cout << e.what() << std::endl;
        }

}

void SpecificWorker::prepareToMove() {
        // Close fingers
        RoboCompJointMotor::MotorGoalPosition finger_right_1, finger_right_2;

        finger_right_1.name = "finger_right_1";
        finger_right_1.position = -0.7;
        finger_right_1.maxSpeed = 2;

        finger_right_2.name = "finger_right_2";
        finger_right_2.position = 0.7;
        finger_right_2.maxSpeed = 2;

        jointmotor_proxy->setPosition(finger_right_1);
        jointmotor_proxy->setPosition(finger_right_2);
        sleep(1);  //Wait to close fingers

        // Lift arm
        InnerModelNode *box = innermodel->getNode("C"+QString::number(targetBox.id));
        InnerModelNode *dst = innermodel->getNode("cameraHand");
        innermodel->moveSubTree(box,dst); // Update innermodel
        innermodel->update();

        setDefaultArmPosition(false); // Set default arm position
        sleep(1); //Wait for arm position
}

void SpecificWorker::setDefaultArmPosition(bool init) {
        // Set positions
        RoboCompJointMotor::MotorGoalPosition wrist_right_1,wrist_right_2, elbow_right, shoulder_right_1,shoulder_right_2,shoulder_right_3, finger_right_1, finger_right_2;


        wrist_right_1.name = "wrist_right_1";
        wrist_right_1.position = 0;
        wrist_right_1.maxSpeed = 1;

        wrist_right_2.name = "wrist_right_2";
        wrist_right_2.position = 1.2;
        wrist_right_2.maxSpeed = 1;

        elbow_right.name = "elbow_right";
        elbow_right.position = 1.5;
        elbow_right.maxSpeed = 1;

        shoulder_right_1.name = "shoulder_right_1";
        shoulder_right_1.position = 0;
        shoulder_right_1.maxSpeed = 1;

        shoulder_right_2.name = "shoulder_right_2";
        shoulder_right_2.position = -1.2;
        shoulder_right_2.maxSpeed = 1;

        shoulder_right_3.name = "shoulder_right_3";
        shoulder_right_3.position = 0;
        shoulder_right_3.maxSpeed = 1;

        jointmotor_proxy->setPosition(wrist_right_1);
        jointmotor_proxy->setPosition(wrist_right_2);
        jointmotor_proxy->setPosition(elbow_right);
        jointmotor_proxy->setPosition(shoulder_right_1);
        jointmotor_proxy->setPosition(shoulder_right_2);
        jointmotor_proxy->setPosition(shoulder_right_3);

        if (init) { // True if the fingers are needed open
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
        setArmReleasingPosition();
        state = END;
}

// RELEASE - Auxiliary method


void SpecificWorker::setArmReleasingPosition() {
        RoboCompJointMotor::MotorGoalPosition wrist_right_1, wrist_right_2, elbow_right,shoulder_right_1, shoulder_right_2, shoulder_right_3, finger_right_1, finger_right_2;

        wrist_right_1.name = "wrist_right_1";
        wrist_right_1.position = 0;
        wrist_right_1.maxSpeed = 1;

        wrist_right_2.name = "wrist_right_2";
        wrist_right_2.position = 0.5;
        wrist_right_2.maxSpeed = 1;

        elbow_right.name = "elbow_right";
        elbow_right.position = 1.5;
        elbow_right.maxSpeed = 1;

        shoulder_right_1.name = "shoulder_right_1";
        shoulder_right_1.position = 0;
        shoulder_right_1.maxSpeed = 1;

        shoulder_right_2.name = "shoulder_right_2";
        shoulder_right_2.position = -0.56;
        shoulder_right_2.maxSpeed = 1;

        shoulder_right_3.name = "shoulder_right_3";
        shoulder_right_3.position = 0;
        shoulder_right_3.maxSpeed = 1;

        jointmotor_proxy->setPosition(wrist_right_1);
        jointmotor_proxy->setPosition(wrist_right_2);
        jointmotor_proxy->setPosition(elbow_right);
        jointmotor_proxy->setPosition(shoulder_right_1);
        jointmotor_proxy->setPosition(shoulder_right_2);
        jointmotor_proxy->setPosition(shoulder_right_3);

        sleep(2); // Wait for position

        finger_right_1.name = "finger_right_1";
        finger_right_1.position = 0.0;
        finger_right_1.maxSpeed = 1;

        finger_right_2.name = "finger_right_2";
        finger_right_2.position = 0.0;
        finger_right_2.maxSpeed = 1;

        jointmotor_proxy->setPosition(finger_right_1);
        jointmotor_proxy->setPosition(finger_right_2);


        InnerModelNode *box = innermodel->getNode("C"+QString::number(targetBox.id));
        InnerModelNode *dst = innermodel->getNode("world");
        innermodel->moveSubTree(box,dst);
        innermodel->update(); // Update innermodel
        sleep(1); // Wait for update
        setDefaultArmPosition(false);
        sleep(2); // Wait for position
        differentialrobot_proxy->setSpeedBase(-250,0); //Move backwards when leaving the box
        sleep(2); // Wait while moving
        handCamera=true; // Activate hand camera
}

// END - Main method
void SpecificWorker::end() {
        target.setEmpty(); // Clear target
        state=IDLE;
        differentialrobot_proxy->setSpeedBase(0, 0); // Stop robot
        sleep(0.1);
        pick = false; // If you have entered a pick using RCISMousePicker, you can continue to other components.
}


/*
   // --------- Calls to our own interface -------------- //
 */

/*
    Go to target if there isnt pick
 */
void SpecificWorker::go(const float x, const float z) {
        while (pick) {
                qDebug() << "Waiting for end userpick";
                sleep(1);
        }
        RoboCompDifferentialRobot::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        target.insert(x,z,bState.x,bState.z);
        state=GOTO;
}

/*
    Just Turn on itself
 */
void SpecificWorker::turn(const float speed) {
        state=IDLE;
        differentialrobot_proxy->setSpeedBase(0,speed);
}

/*
    Return true if robot is working, otherwise false.
 */
string SpecificWorker::getState() {
        switch (state) {
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
        state=END;
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
        state=GOTO;
}


void SpecificWorker::updateJoints()
{
        try
        {
                RoboCompJointMotor::MotorStateMap mMap;
                jointmotor_proxy->getAllMotorState(mMap);
                for(auto m: mMap) {
                        innermodel->updateJointValue(QString::fromStdString(m.first),m.second.pos);
                }
        }
        catch(const Ice::Exception &e)
        { std::cout << e.what() << std::endl;}

}

//HAND
void SpecificWorker::newAprilTag(const RoboCompGetAprilTags::listaMarcas &tags)
{
        int i;
        if (handCamera) { // Condition to check camera
                if (state == GOTO || state == END) { // No box selected
                        for (i=0; i<(signed)tags.size(); i++) {
                                if (tags[i].id > 9) {
                                        differentialrobot_proxy->setSpeedBase(0, 0);
                                        targetBox=tags[i];
                                        state=HAND_WATCHING_BOX;
                                }
                        }
                } else { // Box previously selected. Only update that box.
                        for (i=0; i<(signed)tags.size(); i++) {
                                if (tags[i].id == targetBox.id)
                                        targetBox=tags[i]; // Update
                        }
                }

        } else
                sleep(0.5);


}
