#ifndef __INS_NAV__H__
#define __INS_NAV__H__
#include <iostream>
#include "matrix.h"
#include "ins_gps.h"
#pragma once
#define EARTH_A 6378137.0
#define EARTH_E 0.0818191908426

using namespace std;
class NAV
{
private:
    Matrix R;
    Matrix g_n;
    Matrix vel;
    double lt;
    double lg;
    double h;
    double yaw;
    double pitch;
    double roll;
    double cal_idx;
    Euler DCM_bn;
    Matrix EulerRates;
    Matrix yaw_drift;
    Quaternion qua;
    Matrix omega_ie_n;
    Matrix omega_en_n;
    Matrix Tpr;
    Matrix fn;
    Matrix F;
    Matrix G;
    Matrix H;
    Matrix xp;
    Matrix xi;
    Matrix Q;
    Matrix Pi;
    Matrix Pp;
    Matrix RR;
    Matrix Qd;


    


public:
    NAV():
        R(2,1,0),
        g_n(3,1,0),
        vel(3,1,0),
        lt(0),
        lg(0),
        h(0),
        yaw(0),
        pitch(0),
        roll(0),
        cal_idx(0),
        DCM_bn(3,3,0),
        EulerRates(3,1,0),
        yaw_drift(3,1,0),
        omega_ie_n(3,1,0),
        omega_en_n(3,1,0),
        Tpr(3,3,0),
        fn(3,1,0),
        F(21,21,0),
        G(21,12,0),
        H(6,21,0),
        xp(21,1,0),
        xi(21,1,0),
        Q(12,12,0),
        Pi(21,21,0),
        Pp(21,21,0),
        RR(6,6,0),
        Qd(21,6,0)

        
    {};
    ~NAV();
    void earthrate(Matrix & omega_ie_n);
    void radius();
    void transportate(Matrix & omega_en_n);
    void update_att(Euler & DCMbn_n,Matrix& wb,const double& dt);
    void gravity();
    void vel_update(Matrix & vel_n,Matrix& fn,const double& dt);
    void pos_update(double& lt,double & lg , double& h,const double& dt);
    void init(IMU_DATA& imu_data,GPS_DATA& gps_data,GPS& gps, IMU& imu,double& roll,double& pitch,double& yaw);
    void get_Tpr(Matrix& Tpr);
    void get_F(IMU& imu);
    void kalman(Matrix& xp,Matrix &z,double& dtg);
    void update(Matrix& gb_fix,Matrix& ab_fix,Matrix& gb_grift,Matrix& ab_drift);
    void update_yaw(Euler& DCM,double &yaw_measure, double dt);
    void calibrate(Matrix &cal,Matrix &wb,Matrix &fb);

};
#endif
