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

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    innermodel = new InnerModel("/home/robocomp/robocomp/components/roboticaCC/misc/betaWorldArm3.xml");

    state = SEARCH;

    dump = -1;
    coorsDump.first = coorsDump.second = waitingFor = 0;
    for (auto &x:movedBoxes)
        x=-1;
    for (auto &x:coorsBox)
        x=-2;

    begin_time = clock();
    timer.start(Period);
    return true;
}

void SpecificWorker::compute() {

    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    try{
      newAprilTag(getapriltags_proxy->checkMarcas());
    } catch(const Ice::Exception &e) {
      qDebug() << "Can not connect to aprilTagsMASTER";
    }

    innermodel->updateTransformValues("robot", bState.x,0, bState.z,0,bState.alpha, 0 ); //ACTUALIZAR ARBOL

    //State machine
    switch (state) {
          case SEARCH:
              qDebug() << "SEARCH";
              search();
              break;
          case SENDGOTO:
              qDebug() << "SENDGOTO";
              sendGoto();
              break;
          case WAIT:
              qDebug() << "WAIT";
              wait();
              break;
          case SENDPICKBOX:
              qDebug() << "SENDPICKBOX";
              sendPickBox();
              break;
          case SENDRELEASEBOX:
              qDebug() << "SENDRELEASEBOX";
              sendReleaseBox();
              break;
          default:
              break;
    }
}

/* Search next tag */
void SpecificWorker::search() {
    int x,z;
    waitingFor=0;
    if (float(clock() - begin_time) /10000.0 > 10.0 ) {
        srand( time( NULL ) );  //  using the time seed from srand explanation
        x=rand()%1000-500;
        z=rand()%1000-500;
        chocachoca_proxy->go(x,z);
        begin_time=float(clock());
    } else {
        if (chocachoca_proxy->getState().compare("IDLE")==0)
            chocachoca_proxy->turn(-0.9);
    }

}

/* Go to a specific target */
void SpecificWorker::sendGoto() {
    chocachoca_proxy->go(coorsBox[1],coorsBox[2]);
    waitingFor=1;
    state=WAIT;

}

/* Wait until robot arrive to target */
void SpecificWorker::wait() {
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    switch (waitingFor){
        case 1: //Arrive to box
            if (chocachoca_proxy->getState().compare("HAND_WATCHING_BOX")==0)  // Box ready to be picked
                state=SENDPICKBOX;
            break;
        case 2: //Picking box
            if (chocachoca_proxy->getState().compare("IDLE")==0) {  // Box ready to be moved to the dumpster
                chocachoca_proxy->go(coorsDump.first,coorsDump.second);
                waitingFor=3;
            }
            break;
        case 3: //Arrive to dump
            if (chocachoca_proxy->getState().compare("IDLE")==0) // Box ready to be released
                state=SENDRELEASEBOX;
            break;
        case 4: //Releasing box
            if (chocachoca_proxy->getState().compare("IDLE")==0) {  // Robot ready to keep searching
                addToMovedBoxes(coorsBox[0]);
                for (auto &x:coorsBox)
                    x=-2;
                begin_time=float(clock());
                state=SEARCH;
            }
            break;
        default:
            break;
    }
}


void SpecificWorker::sendPickBox() {
    chocachoca_proxy->pickingBox();
    waitingFor=2;
    state=WAIT;
}

void SpecificWorker::sendReleaseBox() {
    chocachoca_proxy->releasingBox();
    waitingFor=4;
    state=WAIT;
}


void SpecificWorker::addToMovedBoxes(int id) {
    int i=0;
    bool added=false;
    innermodel->updateTransformValues("C"+QString::number(coorsBox[0]), coorsBox[1], 5, coorsBox[2],0,0,0); // Update tree
    while (i<MAXBOXES && !added) {
        if (movedBoxes[i] == -1) {
            movedBoxes[i] = id;
            added=true;
        }
        i++;
    }
}

/* aprilTagsMaster */

void SpecificWorker::newAprilTag(const RoboCompGetAprilTags::listaMarcas &tags) {
    searchDump(tags);
    if (waitingFor < 3)
        searchBoxes(tags);
}

void SpecificWorker::searchDump(const RoboCompGetAprilTags::listaMarcas &tags) {
    int threshold=600,i;
    QVec targetCoors;
    for (i=0; i<(signed)tags.size(); i++) {
        if (tags[i].id < 10 && (dump == -1 || dump == tags[i].id)) {
            targetCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"rgbd");
            float x = targetCoors.x();
            float z = targetCoors.z();
            if (abs(x)>abs(z)) {
                if (x>0) x=x-threshold;
                else x=x+threshold;
            }  else {
                if (z>0) z=z-threshold;
                else z=z+threshold;
            }
            coorsDump=make_pair(x,z);
            dump = tags[i].id;

        }
    }


}

void SpecificWorker::searchBoxes(const RoboCompGetAprilTags::listaMarcas &tags) {
    int i;
    float dist=MAXSEARCHBOX, currentDist=0.0;
    QVec targetCoors,Trobot;

    for (i=0; i<(signed)tags.size(); i++) {
         if (tags[i].id > 9 && !boxIsMoved(tags[i].id) && (coorsBox[0]<0 || coorsBox[0]==tags[i].id) ) {
            targetCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"robot");
            Trobot = innermodel->transform("robot",QVec::vec3(targetCoors.x(),0,targetCoors.z()),"world");
            currentDist = Trobot.norm2();
            if (currentDist < dist) { //MEJOR
                dist=currentDist;
                coorsBox[0]=tags[i].id;
                coorsBox[1]=targetCoors.x();
                coorsBox[2]=targetCoors.z();
            }

         }
    }
    if (dist!=MAXSEARCHBOX && waitingFor < 2)
        state=SENDGOTO;
}

bool SpecificWorker::boxIsMoved(int id) {
    for (auto x:movedBoxes) {
        if (x == id)
            return true;
    }
    return false;

}
