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
    innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/betaWorldArm2.xml");

    nextTag=0;
    tagLocated=-1;
    
    for (auto x:movedBox)
	x=-1;
    for (auto x:watchingtags)
	x=0;
    
    int i;
    for  (int i=0;i<MAXTAGS;i++) {
      coorsBox[i][0]=-2;
      coorsBox[i][1]=-2;
      coorsBox[i][2]=-2;
    }
	

	
    timer.start(Period);
    return true;
}

void SpecificWorker::compute() {

    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
            
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
        pickbox();
    default:
        break;
    }
}

/* Search next tag */
void SpecificWorker::search() {
    qDebug() << "SEARCH";

    // SEARCHING DUMPS AND BOXES
    if (tagLocated != -1) {
        chocachoca_proxy->turn(0);
        searchTheNearestBox();
    }
    else if (!chocachoca_proxy->getState() )
        chocachoca_proxy->turn(0.5);

}

/* Go to a specific target */
void SpecificWorker::gotoTarget() {
    qDebug() << "GOTOTARGET";
    if (!chocachoca_proxy->getState()) {
        chocachoca_proxy->go(coorsTag[nextTag%4].first,coorsTag[nextTag%4].second);
        estado=WAIT;
    }
}

/* Wait until robot arrive to target */
void SpecificWorker::wait() {
    qDebug() << "WAIT";
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    if (!chocachoca_proxy->getState()) {
        if ((abs(bState.x - (coorsTag[nextTag%4].first)) <= MAXDIST) && (abs(bState.z - (coorsTag[nextTag%4].second)) <= MAXDIST)) {
            nextTag++;

        }
        estado = SEARCH;
    }
}


void SpecificWorker::pickbox() {
    qDebug() << "pickBox";

}


int SpecificWorker::searchTheNearestBox() {
//     qDebug() << "searchTheNearestBox";
    int boxI,i;
    float dist=0.0, auxDist=5000.0;
    QVec Trobot;

    for (i=0; i<MAXBOXES; i++) {
        if (!boxIsMoved(i)) {
            Trobot = innermodel->transform("robot",QVec::vec3(coorsBox[i][1],0,coorsBox[i][2]),"world");
            dist = Trobot.norm2();     //Calcular la distancia entre los puntos
            if (dist < auxDist) {
                auxDist=dist;
                boxI=i;
            }
        }
    }
    return boxI;

}


bool SpecificWorker::boxIsMoved(int box) {
//     qDebug() << "boxIsMoved "<<box;
    int i;
    for (i=0; i<MAXBOXES; i++) {
        if (movedBox[i] == coorsBox[box][0]) {
            qDebug()<< "Caja"<<watchingBox[box]<<" ya movida";
            return true;
        }
    }
    //qDebug()<< "Caja"<<coorsBox[box][0]<<" NO movida";
    return false;

}



/* aprilTagsMaster */
void SpecificWorker::newAprilTag(const tagsList &tags) {
    if (tagLocated==-1) { //TAGS BASURERO
        searchDump(tags);
    }
    addBoxes(tags);
}


void SpecificWorker::searchDump(const tagsList &tags) {
    int i,umbral=450;
    QVec targetCoors;
    for (i=0; i<(signed)tags.size(); i++) {
        if (tags[i].id < 10 ) {
            watchingtags[tags[i].id]=1;
            targetCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"rgbd");
            float x = targetCoors.x(); float z = targetCoors.z();
            if (abs(x)>abs(z)) {
                if (x>0) x=x-umbral;
                else x=x+umbral;
            }  else {
                if (z>0) z=z-umbral;
                else z=z+umbral;
            }
            coorsTag[tags[i].id]=make_pair(x,z);
        }
    }
    //COMPROBACION VUELTA COMPLETA
    int saw=0;
    for (i=0; i<4; i++) {
        if (watchingtags[i]==1)
            saw++;
    }

    //SELECCION DE UN BASURERO
    if (saw==MAXTAGS) {
        srand( time( NULL ) );  //  using the time seed from srand explanation
        tagLocated=rand()%MAXTAGS;
    }
}

void SpecificWorker::addBoxes(const tagsList &tags) {
    int i,j,k;
    bool stopWB=false, stopCB=false;
    QVec targetCoors;
    
    for (i=0; i<MAXBOXES; i++)
        watchingBox[i]=-1;

    for (i=0; i<(signed)tags.size(); i++) { // FULL LIST
        qDebug() << tags[i].id ;
        if (tags[i].id > 9 ) { //BOXES
            stopWB=false;
            for (j=0; j<MAXBOXES && !stopWB; j++) {
                if (watchingBox[j]==-1) {
                    watchingBox[j] = tags[i].id;
                    
                    targetCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"robot");
                    stopWB=true;
                    stopCB=false;
                    qDebug() << "Coors antes de transformar"<<tags[i].tx<<tags[i].tz;
                    qDebug() << "Coors despues de transformar"<<targetCoors.x()<<targetCoors.z();
		    qDebug() << "Coors despues de transformar / 2 "<<targetCoors.x()/2<<targetCoors.z()/2;
                    for (k=0; k<MAXBOXES && !stopCB; k++) {
                        coorsBox[k][0]=tags[i].id; 
                        coorsBox[k][1]=targetCoors.x();
                        coorsBox[k][2]=targetCoors.z();
                        qDebug() << "NUEVO ID ------------------> "<<coorsBox[k][0]<<coorsBox[k][1]<<coorsBox[k][2];
                        stopCB=true;
                    }
                    
                }
            }
        }
    }
}
