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

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params){
	innermodel = new InnerModel("/home/robocomp/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	return true;
}

void SpecificWorker::compute(){
	RoboCompDifferentialRobot::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	innermodel->updateTransformValues("base", bState.x,0, bState.z,0,bState.alpha, 0 ); //ACTUALIZAR ARBOL

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
    	default:
    		break;
	}
}

/* Search next tag */
void SpecificWorker::search() {
	if (!chocachoca_proxy->getState()) {
		qDebug() << "SEARCH activo"<<nextTag%4;
		chocachoca_proxy->turn(0.6);
		if (watchingtags[nextTag%4]==1)
			estado=GOTO;
	}
}

/* Go to a specific target */
void SpecificWorker::gotoTarget() {
	if (!chocachoca_proxy->getState()) {
		chocachoca_proxy->go(coorsTag[nextTag%4].first,coorsTag[nextTag%4].second);
		estado=WAIT;
	}
}

/* Wait until robot arrive to target */
void SpecificWorker::wait() {
	RoboCompDifferentialRobot::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	if (!chocachoca_proxy->getState()) {
		if ((abs(bState.x - (coorsTag[nextTag%4].first)) <= 400) && (abs(bState.z - (coorsTag[nextTag%4].second)) <= 400))
			nextTag++;
		estado = SEARCH;
	}
}

/* aprilTagsMaster */
void SpecificWorker::newAprilTag(const tagsList &tags){
	int i,umbral=300;
	for (i=0; i<4; i++)  //Inicialización
		watchingtags[i]=0;
	for (i=0; i<(signed)tags.size(); i++) {
		watchingtags[tags[i].id]=1;
		QVec tagsCoors = innermodel->transform("world",QVec::vec3(tags[i].tx,0,tags[i].tz),"rgbd");
		float x = tagsCoors.x();
		float z = tagsCoors.z();
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
