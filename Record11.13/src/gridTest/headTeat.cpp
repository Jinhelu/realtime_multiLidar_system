//
// Created by cyn on 2020/6/11.
//
#include <bits/stdc++.h>
#include "../grid.h"

int main(){
    grid testgrid;
    GridMap testMap(4,4);
    testMap.ModifyGridState(1,1,OCCU);
    testMap.ModifyGridState(2,2,FREE);
    testMap.ModifyGridState(2,2,OCCU);

    testMap.ModifyGridCorLaser(-2,2,OCCU);
    //testMap.ModifyGridState(2,2,OCCU);

    testMap.SaveGrid();

    GridMap testMap2("../saved/grid.txt");
    return 0;
}

