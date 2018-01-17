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


//innermodel->jacobian

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <math.h>       /* sqrt */
#include <unistd.h>
#include <time.h>

#define LINEAL_INCREMENT 10
#define ANGULAR_PROP 12.0
#define FACTOR 1
#define FACTOR_FINGERS 0.6
#define PI 3.14159265358979323846

using namespace std;
enum states { IDLE, GOTO, HAND_WATCHING_BOX, TURN, SKIRT, END, PICK };
class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(MapPrx& mprx);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        // STATE MACHINE
        void idle();
        void end();
        float gotoTarget(TBaseState bState, TLaserData laserData);
        void handWatchingBox();
        void skirt(TBaseState bState, TLaserData &laserData);
        void turn(float linealSpeed,TLaserData laserData);
        void arm();

        // AUXILIARY METHODS
        float gauss(float Vrot, float Vx, float h);
        float sinusoide(float x);

        bool onTarget(float dist);
        bool isOnLine(TBaseState bState);
        bool reachableTarget(TBaseState bState, float dist, TLaserData &laserData);

        void fixPosition();
        void fixRotation();
        void adjustMotors();
        void stopMotors();
        void prepareToMove();
        void setDefaultArmPosition(bool init);
        void setArmReleasingPosition();


        // OWN INTERFACE CALLS
        void go(const float x, const float z);
        void turn(const float speed);
        string getState();
        void stop();
        void setPick(const Pick &myPick);
        void releasingBox();
        void pickingBox();

        // OTHER INTERFACE CALLS
        void newAprilTag(const RoboCompGetAprilTags::listaMarcas &tags);


  public slots:
        void compute();

  private:
        InnerModel *innermodel;

        float const VLIN_MAX = 400;
        float const VROT_MAX = 0.6;

        states state; /* Current state */

        bool side; /* True = right, false = left. */
        bool preState;
        bool pick; /* Flag for differences between pick and searchTags */
        pair <int, int> thresholdValues; /* Min - max thresholdValues */

        //ARM
        QStringList joints;
        QVec engines;
        QVec error;

        void updateJoints();

        //HAND
        RoboCompGetAprilTags::marca targetBox;

        bool picked; /* True = box is in hand, False = box is not in hand */
        bool handCamera; /* True = handCamera ON, False = handCamera OFF */
        bool nearToBox; /* True = Too close to see the QR code */


        struct Target { /* Target struct */
            QMutex mutex;
            float xt, zt, xr, zr;
            bool empty;

            //Constructor
            Target() {
                xt = 0; zt = 0;
                xr = 0; zr = 0;
                empty = true;
            };

            bool insert(float _xt, float _zt, float _xr, float _zr ) {
                QMutexLocker ml(&mutex); //Controla el mutex
                xt = _xt;
                zt = _zt;
                xr = _xr;
                zr = _zr;
                empty = false;
                return true;
            };

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

            bool isEmpty() {
                QMutexLocker ml(&mutex);
                return empty;
            };

            void setEmpty() {
                QMutexLocker ml(&mutex);
                empty = true;
            };

            pair <float,float> getPoseTarget() {
                std::pair <float,float> coorsTarget;
                coorsTarget.first=xt;
                coorsTarget.second=zt;
                return coorsTarget;
            };

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
