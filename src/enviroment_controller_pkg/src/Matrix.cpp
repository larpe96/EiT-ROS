#include "Matrix.h"


RMatrix::RMatrix()
{
    eye();
}

RMatrix::RMatrix(float r00, float r01, float r02, float r10, float r11, float r12,float r20, float r21, float r22)
{
    mat[0][0] = r00;
    mat[0][1] = r01;
    mat[0][2] = r02;

    mat[1][0] = r10;
    mat[1][1] = r11;
    mat[1][2] = r12;
    
    mat[2][0] = r20;
    mat[2][1] = r21;
    mat[2][2] = r22;
}


RMatrix RMatrix::transpose()
{
    RMatrix res;
    int len_i = sizeof(mat)/sizeof(mat[0]);
    int len_j = sizeof(mat[0])/sizeof(mat[0][0]);

    for(int i=0; i<len_i;i++)
    {
        for(int j=0; j<len_j;j++)
        {
            res.mat[j][i] = mat[i][j];
        }
    }
    return res;
}


void RMatrix::eye()
{
    int len_i = sizeof(mat)/sizeof(mat[0]);
    int len_j = sizeof(mat[0])/sizeof(mat[0][0]);
    for(int i=0; i<len_i;i++)
    {
        for(int j=0; j<len_j;j++)
        {
            if (i==j)
            {
                mat[i][j]=1;
            }
            else{
                mat[i][j]=0;
            }
        }
    }
}

void RMatrix::print()
{
    int len_i = sizeof(mat)/sizeof(mat[0]);
    int len_j = sizeof(mat[0])/sizeof(mat[0][0]);
    for(int i=0; i<len_i;i++)
    {
        for(int j=0; j<len_j;j++)
        {
            std::cout<< mat[i][j]<<"\t";
        }
        std::cout<<"\n";
    }
}


RMatrix::~RMatrix()
{
}



TMatrix::TMatrix(/* args */)
{
    eye();
}

TMatrix::TMatrix(RMatrix r)
{
    eye();
    mat[0][0] = r.mat[0][0];
    mat[0][1] = r.mat[0][1];
    mat[0][2] = r.mat[0][2];

    mat[1][0] = r.mat[1][0];
    mat[1][1] = r.mat[1][1];
    mat[1][2] = r.mat[1][2];
    
    mat[2][0] = r.mat[2][0];
    mat[2][1] = r.mat[2][1];
    mat[2][2] = r.mat[2][2];
}

TMatrix::TMatrix(geometry_msgs::Pose _pose)
{
    eye();
    // Translation
    mat[0][3] = _pose.position.x;
    mat[1][3] = _pose.position.y;
    mat[2][3] = _pose.position.z;

    // Rotation
    float x = _pose.orientation.x;
    float y = _pose.orientation.y;
    float z = _pose.orientation.z;
    float w = _pose.orientation.w;
    // quat 2 rotm

    // row 0
    mat[0][0] = 2*(pow(w,2)+pow(x,2))-1;
    mat[0][1] = 2*(x*y-w*z);
    mat[0][2] = 2*(x*z+w*y);
    // row 1
    mat[1][0] = 2*(x*y+w*z); 
    mat[1][1] = 2*(pow(w,2)+pow(y,2))-1;
    mat[1][2] = 2*(y*z-w*x);
    // row 2
    mat[2][0] = 2*(x*z-w*y); 
    mat[2][1] = 2*(y*z+w*x);
    mat[2][2] = 2*(pow(w,2)+pow(z,2))-1;


    //mat[3][2] = 20;
}


TMatrix::TMatrix(float a0, float a1, float a2, float a3, float b0, float b1, float b2, float b3, float c0, float c1, float c2, float c3, float d0, float d1, float d2, float d3)
{
    mat[0][0] = a0;
    mat[0][1] = a1;
    mat[0][2] = a2;
    mat[0][3] = a3;
    mat[1][0] = b0;
    mat[1][1] = b1;
    mat[1][2] = b2;
    mat[1][3] = b3;
    mat[2][0] = c0;
    mat[2][1] = c1;
    mat[2][2] = c2;
    mat[2][3] = c3;
    mat[3][0] = d0;
    mat[3][1] = d1;
    mat[3][2] = d2;
    mat[3][3] = d3;

}

TMatrix::~TMatrix()
{
}


geometry_msgs::Pose TMatrix::getPose()
{
    // see https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf

    
    geometry_msgs::Pose pose;
    pose.position.x = mat[0][3]/mat[3][3];
    pose.position.y = mat[1][3]/mat[3][3];
    pose.position.z  = mat[2][3]/mat[3][3];

    
    float _x,_y,_z,_w,temp; // not normalised
    // _x = 1+mat[0][0]-mat[1][1]-mat[2][2];// = 4x^2
    // _y = mat[1][0]+mat[0][1]; // 4xy
    // _z = mat[2][0]+mat[0][2]; //4xz
    // _w = mat[1][2]-mat[2][1]; //4xw

    if (mat[2][2]<0)
    {
        if(mat[0][0] >mat[1][1] )
        {
            _x = 1+mat[0][0]-mat[1][1]-mat[2][2];
            _y = mat[0][1] + mat[1][0];
            _z = mat[2][0] + mat[0][2];
            _w = mat[1][2] - mat[2][1];
            temp = _x;
        }
        else{
            _x = mat[0][1] + mat[1][0];
            _y = 1-mat[0][0] + mat[1][1] - mat[2][2];
            _z = mat[1][2] + mat[2][1];
            _w = mat[2][0] - mat[0][2];
            temp=_y;
        }
    }
    else{
        if (mat[0][0]< -mat[1][1])
        {
            _x=mat[2][0] + mat[0][2];
            _y=mat[1][2] + mat[2][1];
            _z=1-mat[0][0] - mat[1][1]+mat[2][2];
            _w=mat[0][1] - mat[1][0];
            temp = _z;

        }
        else{
            _x=mat[1][2] - mat[2][1];
            _y=mat[2][0] - mat[0][2];
            _z=mat[0][1] - mat[1][0];
            _w=1+mat[0][0] + mat[1][1] + mat[2][2];

            temp = _w;
        }
    }

    // scale factor
    float scale_fac = 0.5/(sqrt(temp));

    // norm quat
    pose.orientation.x = _x * scale_fac;
    pose.orientation.y = _y * scale_fac;
    pose.orientation.z = _z * scale_fac;
    pose.orientation.w = _w * scale_fac;


    return pose;
}


void TMatrix::eye()
{
    int len_i = sizeof(mat)/sizeof(mat[0]);
    int len_j = sizeof(mat[0])/sizeof(mat[0][0]);
    for(int i=0; i<len_i;i++)
    {
        for(int j=0; j<len_j;j++)
        {
            if (i==j)
            {
                mat[i][j]=1;
            }
            else{
                mat[i][j]=0;
            }
        }
    }
}

void TMatrix::zero()
{
    int len_i = sizeof(mat)/sizeof(mat[0]);
    int len_j = sizeof(mat[0])/sizeof(mat[0][0]);
    for(int i=0; i<len_i;i++)
    {
        for(int j=0; j<len_j;j++)
        {
            mat[i][j]=0;
        }
    }
}

void TMatrix::print()
{

    int len_i = sizeof(mat)/sizeof(mat[0]);
    int len_j = sizeof(mat[0])/sizeof(mat[0][0]);
    
    for(int i=0; i<len_i;i++)
    {
        for(int j=0; j<len_j;j++)
        {
            
            std::cout<<mat[i][j]<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<"\n";
}

TMatrix TMatrix::operator*(TMatrix B)
{
    TMatrix res;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res.mat[i][j] = 0;
 
            for (int k = 0; k < 4; k++) {
                res.mat[i][j] += mat[i][k] * B.mat[k][j];
            }
        }
    }
    return res;
}

float TMatrix::getElem(int i, int j)
{
    return mat[i][j];
}


void TMatrix::setTranslation(float x, float y, float z)
{
    mat[0][3]=x;
    mat[1][3]=y;
    mat[2][3]=z;  
    mat[3][3]=1;  
}

TMatrix TMatrix::inverse()
{
    TMatrix tmat; // translation
    tmat.setTranslation((-1)*mat[0][3]/mat[3][3],(-1)*mat[1][3]/mat[3][3],(-1)*mat[2][3]/mat[3][3]);

    RMatrix rmat(mat[0][0],mat[0][1],mat[0][2],mat[1][0],mat[1][1],mat[1][2],mat[2][0],mat[2][1],mat[2][2]);
    rmat = rmat.transpose();

    TMatrix T_r_transpose(rmat);
    return T_r_transpose*tmat;
}


