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
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <mutex>          // std::mutex
#include "Chocachoca.h"

using namespace std;
enum state { SEARCH, WAIT, GOTO};

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
public:
    SpecificWorker(MapPrx& mprx);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);

    void newAprilTag(const tagsList &tags);
    void search();
    void gotoTarget();
    void wait();

public slots:
    void compute();

private:
    InnerModel *innermodel;
    std::mutex mtx;
    int nextTag=0;
//  std::pair<int,int> coorsTag[4]= {make_pair(0,2100),make_pair(2100,0),make_pair(0,-2100),make_pair(-2100,0)};
    std::pair<int,int> coorsTag[4];
    std::pair<int,int> coorsCurrent;
    int watchingtags[4]= {0,0,0,0};
    state estado=SEARCH;
    int maxDist=400;
};

#endif

