#ifndef TMATRIX_H
#define TMATRIX_H

#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include<math.h>


class RMatrix
{
protected:
public:
    RMatrix();
    RMatrix(float r00, float r01, float r02, float r10, float r11, float r12,float r20, float r21, float r22);
    RMatrix transpose();
    void eye();
    void print();
    ~RMatrix();
    float mat[3][3];
};



class TMatrix
{
protected:
    /* data */

    float mat[4][4];

public:
    TMatrix(/* args */);
    TMatrix(geometry_msgs::Pose _pose);
    TMatrix(float, float, float, float, float, float, float, float,float, float, float, float,float, float, float, float);
    TMatrix(RMatrix r);
    ~TMatrix();
    TMatrix operator* (TMatrix B);
    
    
    void eye();
    void zero();
    void print();
    void setTranslation(float x, float y, float z);

    float getElem(int i, int j);


    geometry_msgs::Pose getPose();
    TMatrix inverse();
};



#endif