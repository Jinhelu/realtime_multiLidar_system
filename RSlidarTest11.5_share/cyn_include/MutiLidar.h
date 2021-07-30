//
// Created by cyn on 20-11-4.
//

#ifndef MUTLIDAR_H
#define MUTLIDAR_H

#include <bits/stdc++.h>
#include "Auxiliary.h"
#include "LidarMap_t.h"
using namespace std;

#define PCLShow
#define OpenCVShow

int MutLidar(IniParam &Param,LidarMap_t &lidarMap_th,volatile bool &RunFlag,string pcapAdress);

void LidarMapThreadFun(volatile bool &RunFlag,LidarMap_t& lidarMap_t,string pcapAdress);

#endif //MUTILIDARTEST_LIDARMAP_H
