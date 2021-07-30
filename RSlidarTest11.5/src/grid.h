#ifndef GRID_H
#define GRID_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
using namespace std;

enum gridState{FREE,OCCU,UNKNOW};// 分别对应  0 1 2

class grid{
public:
    grid(){
        _state = UNKNOW;
        _visitNum = 0;//只有被判断为Free或者Occu才算访问
        _freeNum = 0;
        _occuNum = 0;
    };

    grid(gridState state,unsigned int visitNum,unsigned int freeNum,unsigned int occuNum){
        _state = state;
        _visitNum = visitNum;
        _freeNum = freeNum;
        _occuNum = occuNum;
    };

    void addVisit(gridState visitState){
        if(visitState!=UNKNOW) {
            ++_visitNum;
            if(visitState==FREE) ++_freeNum;
            else if(visitState==OCCU) ++_occuNum;
            else cerr << "grid.h addVisit: error visitState!" << endl;
        }
        else return;

        if(_visitNum == 1) _state=visitState;

        if(_visitNum > 2){
            if(_state == FREE && ((float)_occuNum) / _visitNum > 0.666) _state=OCCU;
            if(_state == OCCU && ((float)_freeNum) / _visitNum > 0.666) _state=FREE;
        }
    }

    gridState state(){return _state; }
    unsigned int visitNum() const{return _visitNum; }
    unsigned int freeNum() const{return _freeNum; }
    unsigned int occuNum() const{return _occuNum; }


private:
    gridState _state;
    unsigned int _visitNum;//只有被判断为Free或者Occu才算访问
    unsigned int _freeNum;
    unsigned int _occuNum;
};

class GridMap{
public:
    //初始化删格地图  输入：图像宽高 删格宽高   初始化各个删格状态为2（未知）
    //构造函数不能返回特定类型的参数
    GridMap(int WidthGridNum = 120,int HeightGridNum = 120){
      if (((WidthGridNum/2)*2)!=WidthGridNum || ((HeightGridNum/2)*2)!=HeightGridNum)
        cerr<<"输入删格必须为偶数！重新定义删格！"<<endl;

      else{
        grid temp;
        deque<grid> XCoordState (WidthGridNum, temp);
        deque<deque<grid> > YCoord(HeightGridNum, XCoordState);
        _YCoord = YCoord;
        _laser_x = WidthGridNum/2-1;//实际上不属于任何一个栅格，在两格中间
        _laser_y = HeightGridNum/2-1;//实际上不属于任何一个栅格，在两格中间
      }
    }

    //构析函数

    //~GridMap(){delete &_YCoord;delete &_XCoordState;
    //  delete &_WidthGridNum; delete &_HeightGridNum;
    //}


    //读取某个特定删格的状态 从第0个删格开始
    gridState GetGridState(int x,int y){ return _YCoord[y][x].state();}
    unsigned int GetGridVisitNum(int x,int y){ return _YCoord[y][x].visitNum();}
    unsigned int GetGridFreeNum(int x,int y){ return _YCoord[y][x].freeNum();}
    unsigned int GetGridOccuNum(int x,int y){ return _YCoord[y][x].occuNum();}

    //从激光雷达坐标转化为原点坐标(从0开始)
    void LaserCoord2GridCoord (int& x_laser,int& y_laser,int& x_grid,int& y_grid) const {
        if(x_laser>=0) x_grid= _laser_x + x_laser;
        else x_grid= _laser_x + x_laser+1;

        if(y_laser>0) y_grid=_laser_y - y_laser+1;
        else y_grid=_laser_y - y_laser;
    }

    //某删格在删格原点下的坐标(从0开始)转化为在激光雷达下的坐标
    void GridCoord2LaserCoord(int &GridCordX,int &GridCordY,int &LaserCordX,int &LaserCordY) const{
        if(GridCordX>=_laser_x) LaserCordX=GridCordX-_laser_x;
        else LaserCordX = GridCordX - _laser_x -1;

        if(GridCordY > _laser_y) LaserCordY=_laser_y-GridCordY+1;
        else LaserCordY = _laser_y-GridCordY;
    }

    //以激光原点为参照系读取删格状态
    gridState GetGridCorLaser(int x,int y){
      //cout << "_YCoord[_laser_y - y -1][_laser_x + x -1];  " << _laser_y - y -1
      //     <<"  "<< _laser_x + x -1 << endl;
      int x_grid,y_grid;
      LaserCoord2GridCoord(x,y,x_grid,y_grid);
      return _YCoord[y_grid][x_grid].state();
    }


    //读取删格的长和宽
    int WidthNum(){ return _YCoord[0].size(); }
    int HeightNum(){ return _YCoord.size(); }
    int LaserNumX(){ return _laser_x; }
    int LaserNumY(){ return _laser_y; }


    //删格规模扩容  在原有删格坐标系下的 极限坐标(可以为负数) 珊格数从0开始
    bool EnlargeGrid(int x,int y){
      grid temp;
      //判断X是否超出范围，超出范围就扩容
      if(x>=(int)_YCoord[0].size()){
        int deltaX = x - _YCoord[0].size()+1;
        //x方向补齐
        while(deltaX){
          //每行都增加一个删格
          int col_flag = _YCoord.size();
          while(col_flag){_YCoord[col_flag-1].push_back(temp);  col_flag--;}
          deltaX--;
        }
        //cout << "x正方向扩容" << endl;
      }
      else if(x<0){
        int deltaX = (-x);
        while(deltaX){
          //cout << "x负方向扩容" << deltaX <<endl;
          int col_flag = _YCoord.size();
          while(col_flag){_YCoord[col_flag-1].push_front(temp);  col_flag--; }
          deltaX--;
        }
        _laser_x += (-x);
        //cout << "x负方向扩容" << endl;
      }


      //判断Y是否超出范围，超出范围就扩容
      if(y>=(int)_YCoord.size()){
        deque<grid> LineTemp(_YCoord[0].size(),temp);
        int deltaY = y - _YCoord.size() +1;
        //Y方向补齐
        while(deltaY){_YCoord.push_back(LineTemp);  deltaY--;}
        //cout << "y正方向扩容" << endl;
      }
      else if(y<0){
        deque<grid> LineTemp(_YCoord[0].size(),temp);
        int deltaY = (-y);
        while(deltaY){_YCoord.push_front(LineTemp);  deltaY--;}
        _laser_y = _laser_y + (-y);
        //cout << "y负方向扩容" << endl;
      }
      return true;
    }


    //修改某个特定删格的状态 从第0个删格开始  x坐标 y坐标 x,y可以为负数
    bool ModifyGridState(int x,int y,gridState state){
      if (state != FREE && state != OCCU && state != UNKNOW){
        cerr << "input _state is illegal!" << endl;
        return false;
      }
      else{
        EnlargeGrid(x,y);
        if(x>=0  && y>=0 )  _YCoord[y][x].addVisit(state);
        else{
            if(x<0) x=0;
            if(y<0) y=0;
            _YCoord[y][x].addVisit(state);
        }
      }
      return true;
    }


    //以雷达为原点的参考系修改特定删格状态
    bool ModifyGridCorLaser(int x,int y,gridState state){
        int x_grid,y_grid;
        LaserCoord2GridCoord(x,y,x_grid,y_grid);

        EnlargeGrid(x_grid, y_grid);//此时的_laser_x和_laser_y已经改变

        LaserCoord2GridCoord(x,y,x_grid,y_grid);//此时一定是合法的 x_grid, y_grid
        ModifyGridState(x_grid, y_grid,state);

        return true;
    }

    //以向量形式返回删格
    vector<vector<int> > GetGridVector(){
        vector<int> a;
        vector<vector<int> > YCoord_Vector(this->HeightNum(),a);
        for(int j=0;j<this->HeightNum();j++){
            for(int i=0;i<this->WidthNum();i++){
                int oldState;
                if(this->_YCoord[j][i].state() == FREE) oldState=0;
                else if(this->_YCoord[j][i].state() == OCCU) oldState=1;
                else oldState = 2;
                YCoord_Vector[j].push_back(oldState);
            }
        }
        return YCoord_Vector;
    }

    //  ../saved/grid.txt
    bool SaveGrid(string filename = "grid.txt"){
      //创建文件对象
      ofstream outfile("../saved/"+filename,ofstream::out);

      if(!outfile){cerr << " 文件打开失败 "<<endl;  return false;}
      //写删格中的激光位置
      outfile << _laser_x << " " << _laser_y << endl;

      for(int j=0;j<_YCoord.size();j++){
        //对每一行y
        for(int i=0;i<_YCoord[0].size();i++){
          //对每行的每个删格x

          outfile << GetGridState(i,j) << " "<< GetGridVisitNum(i,j) << " "
                  << GetGridFreeNum(i,j) << " " << GetGridOccuNum(i,j) << "   ";
        }
        outfile << endl;//换行即新的栅格行
      }
      //写入文件
      return true;
    }

    //以读文件的方式创建对象
    GridMap(string adress){
      ifstream infile(adress);
      if (!infile)  { cerr<<"读取文件失败!"<<endl; }
      else{
        string temp_str;
        //读取激光雷达数据
        infile >> _laser_x >> _laser_y;
        getline(infile,temp_str);

        //读取文件到二维deque
        while(getline(infile,temp_str)){
          //行内的操作
          deque<grid> test_line;
          stringstream temp_ss(temp_str);

          gridState state;
          unsigned int stateNum;
          unsigned int visitNum;
          unsigned int freeNum;
          unsigned int occuNum;

          while(temp_ss >> stateNum >> visitNum >> freeNum >> occuNum) {
              if(stateNum==0) state=FREE;
              else if(stateNum==1) state = OCCU;
              else state = UNKNOW;

              grid temp(state,visitNum,freeNum,occuNum);
              test_line.push_back(temp);
          }

          _YCoord.push_back(test_line);

        }//整个文件读取完成

        //初始化private成员
        cout<<"文件读取完成!"<<" 行数： "<<this->HeightNum()<<"列数： "<<this->WidthNum()
            << endl<< "激光雷达原点： x："<< _laser_x  << "   y:" << _laser_y <<endl;
      }
    }



private:
    int _laser_x;//名义上偏左
    int _laser_y;//名义上偏上

    deque<deque<grid> >  _YCoord;    //标号是Y序列 ,纵坐标，行数

};

#endif
