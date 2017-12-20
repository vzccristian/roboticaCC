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

    dump=-1;
    coorsDump.first=0;
    coorsDump.second=0;
    boxInArm=false;
    
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
      std::cout << e <<endl;
    }
    
    innermodel->updateTransformValues("robot", bState.x,0, bState.z,0,bState.alpha, 0 ); //ACTUALIZAR ARBOL
            
    //MAQUINA DE ESTADOS
    switch (estado) {
    case SEARCH:
        search();
        break;
    case GOTO:
        gotoTarget();
        break;
    case WAIT:
        wait();
        break;
    case PICKBOX:
        pickBox();
        break;
    case RELEASEBOX:
        releaseBox();
        break;
    default:
        break;
    }
}

/* Search next tag */
void SpecificWorker::search() {
    qDebug() << "SEARCH ";
    int x,z;
    if (float(clock() - begin_time) /10000.0 > 10.0 ) {
        srand( time( NULL ) );  //  using the time seed from srand explanation
        x=rand()%1000-500;
        z=rand()%1000-500;
        qDebug() << x<<z;
        chocachoca_proxy->go(x,z);
        begin_time=float(clock());
    } else {
        if (!chocachoca_proxy->getState())
            chocachoca_proxy->turn(-0.9);

    }

}

/* Go to a specific target */
void SpecificWorker::gotoTarget() {
    qDebug() << "GOTO " << coorsBox[1] << coorsBox[2];
//     if (!chocachoca_proxy->getState()) {
    chocachoca_proxy->go(coorsBox[1],coorsBox[2]);
    estado=WAIT;
//     }
}

/* Wait until robot arrive to target */
void SpecificWorker::wait() {
//     qDebug() << "WAIT";
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
//     qDebug() << "ROBOT. x-->"<<bState.x<<"  z-->"<<bState.z;
//     qDebug() << "BOX "<<coorsBox[0]<<" x-->"<<coorsBox[1]<<"  z-->"<<coorsBox[2];
    if (boxInArm) {
        if ((abs(bState.x - (coorsDump.first)) <= MAXDIST*1.5) && (abs(bState.z - (coorsDump.second)) <= MAXDIST*1.5)) { //ENCONTRADO
            chocachoca_proxy->stop();
            estado=RELEASEBOX;
        } 
    }  else {
        if ((abs(bState.x - (coorsBox[1])) <= MAXDIST) && (abs(bState.z - (coorsBox[2])) <= MAXDIST)) { //ENCONTRADO
            chocachoca_proxy->stop();
            estado=PICKBOX;
        } 
    }

}


void SpecificWorker::pickBox() {
    qDebug() << "pickBox supervisor";
    if (chocachoca_proxy->pickingBox()) {
      qDebug() << "chocachoca dice que ya ha cogido la caja";
      boxInArm=true;
      chocachoca_proxy->go(coorsDump.first,coorsDump.second);
      estado=WAIT;
    }
}

void SpecificWorker::releaseBox() {
    qDebug() << "releaseBox";
    boxInArm=false;
    addToMovedBoxes(coorsBox[0]);
    begin_time=float(clock());
    estado=SEARCH;
    
}


void SpecificWorker::addToMovedBoxes(int id) {
    qDebug() << "addToMovedBoxes";
    int i=0;
    bool added=false;
    while (i<MAXBOXES && !added) {
        if (movedBoxes[i] == -1) {
            movedBoxes[i] = id;
            qDebug() << "Aniadido ID: "<<id<<" posicion "<<i;
            added=true;
        }
        i++;
    }
}

/* aprilTagsMaster */
void SpecificWorker::newAprilTag(const RoboCompGetAprilTags::listaMarcas &tags) {
    if (dump == -1)
        searchDump(tags);
//     if (estado==SEARCH)
        searchBoxes(tags);
}

void SpecificWorker::searchDump(const RoboCompGetAprilTags::listaMarcas &tags) {
    int umbral=300,i;
    QVec targetCoors;
    
    for (i=0; i<(signed)tags.size(); i++) {
        if (tags[i].id < 10) {
            targetCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"rgbd");
            float x = targetCoors.x(); float z = targetCoors.z();
            if (abs(x)>abs(z)) {
                if (x>0) x=x-umbral;
                else x=x+umbral;
            }  else {
                if (z>0) z=z-umbral;
                else z=z+umbral;
            }
            coorsDump=make_pair(x,z); //Coordenadas
            dump = tags[i].id;
            qDebug() << "Dump:"<<dump;
            return;
        }
    }

    
}

void SpecificWorker::searchBoxes(const RoboCompGetAprilTags::listaMarcas &tags) {
    int i;
    float dist=MAXSEARCHBOX, currentDist=0.0;
    QVec targetCoors,Trobot;
    
    for (i=0; i<(signed)tags.size(); i++) {
         if (tags[i].id > 9 && !boxIsMoved(tags[i].id)) {
            targetCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"robot");
            Trobot = innermodel->transform("robot",QVec::vec3(targetCoors.x(),0,targetCoors.z()),"world");    
            currentDist = Trobot.norm2();     
            if (currentDist < dist) { //MEJOR
                dist=currentDist;
                coorsBox[0]=tags[i].id; 
                coorsBox[1]=targetCoors.x();
                coorsBox[2]=targetCoors.z();
                qDebug() << "UPDATE -> BOX "<<coorsBox[0]<<" x-->"<<coorsBox[1]<<"  z-->"<<coorsBox[2]<< ". distancia"<<currentDist;
            }
           
         }
    }
    if (dist!=MAXSEARCHBOX)
        estado=GOTO;
}

bool SpecificWorker::boxIsMoved(int id) {
    for (auto x:movedBoxes) {
        if (x == id) {
            return true;
            qDebug() << "Encontrado "<<id;
        }
    }
    return false;
    
}
