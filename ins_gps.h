#ifndef __INS_GPS__H__
#define __INS_GPS__H__


#include "matrix.h"
#pragma once
#define PI 3.1415926535897932384626433832795
#define D2R  PI/180
#define R2D  180/pi
#define G2MSS  9.81
#define MSS2G  (1/G2MSS);     
#define KT2MS  0.514444;   % knot to m/s
#define MS2KMH  3.6

class IMU
{
public:
    Matrix arw;
    Matrix arrw;
    Matrix vrw;
    Matrix vrrw;
    Matrix gb_fix;
    Matrix ab_fix;
    Matrix gb_drift;
    Matrix ab_drift;
    Matrix gb_corr;
    Matrix ab_corr;
    Matrix freq;
    Matrix ab_std;
    Matrix gb_std;
    Matrix ab_psd;
    Matrix gb_psd;
    Matrix align_err;
    Matrix align;
public:
    IMU():arw(1,3,0.3),
            arrw(1,3,0),
            vrw(1,3,0.029),
            vrrw(1,3,0),
            gb_fix(3,1,0.01),
            ab_fix(3,1,16),
            gb_drift(3,1,6.5/3600),
            ab_drift(3,1,0.1),    
            gb_corr(1,3,100),
            ab_corr(1,3,100),    
            freq(1,3,100),
            align(1,3,0),
            align_err(1,3,1*D2R)

    {
/*               arw.mat_set(0,0,1.240492296339036e-03);
               arw.mat_set(0,1,1.140727875285529e-03);
               arw.mat_set(0,2,9.375873096278934e-04);
               gb_fix.mat_set(0,0,-5.512800728745165e-06);
               gb_fix.mat_set(0,1,1.109262639036322e-05);
               gb_fix.mat_set(0,2,1.269351691201790e-04);
               gb_drift.mat_set(0,0,7.112094640463134e-05);
               gb_drift.mat_set(0,1,7.112094640463134e-05);
               gb_drift.mat_set(0,2,1.663320036005468e-04);*/
               align_err.mat_set(0,2,180*D2R);
    }

// Angle random walks [X Y Z] (deg/root-hour)
// Angle rate random walks [X Y Z] (deg/root-hour/s)
// Velocity random walks [X Y Z] (m/s/root-hour)
// Velocity rate random walks [X Y Z] (deg/root-hour/s)
// Gyro static biases [X Y Z] (deg/s)
// Acc static biases [X Y Z] (mg)
// Gyro dynamic biases [X Y Z] (deg/s)
// Acc dynamic biases [X Y Z] (mg)
// Gyro correlation times [X Y Z] (seconds)
// Acc correlation times [X Y Z] (seconds)
// IMU operation frequency [X Y Z] (Hz)
// Magnetometer noise density [X Y Z] (mgauss/root-Hz)

    ~IMU();
    void imu_si_errors( IMU& imu,const double& dt);
    
};

class GPS
{
public:
    Matrix std;
    Matrix stdm;
    Matrix stdv;
    Matrix larm;
    double freq;

public:
    GPS():std(1,3,1e-7),
            stdm(1,3,1),
            //stdv(1,3,0.0514),
            stdv(1,3,10),
            larm(3,1,0),
            freq(5)

    {
        stdm.mat_set(0,2,5);
        std.mat_set(0,0,7.86796293899852e-07);
        std.mat_set(0,1,9.31921056438468e-07);
        std.mat_set(0,2,10);
    }
    ~GPS();
       
};

class GPS_DATA
{
public:
    Matrix pos;
    Matrix vel;
    double t;

public:
    GPS_DATA():pos(3,1,0),
            vel(3,1,0),
            t(0)
    {}
    ~GPS_DATA();
       
};

class IMU_DATA
{
public:
    Matrix fb;
    Matrix wb;
    double t;

public:
    IMU_DATA():fb(3,1,0),
            wb(3,1,0),
            t(0)
    {}
    ~IMU_DATA();
       
};
#endif
