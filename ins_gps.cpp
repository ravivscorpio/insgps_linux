#include "ins_gps.h"


using namespace std;


IMU::~IMU()
{

}
void IMU::imu_si_errors(IMU& imu,const double& dt)
{
    


    this->arw.scalermultiply(imu.arw,1.0/60.0*D2R);//deg/root-hour -> rad/s/root-Hz
    this->vrw.scalermultiply(imu.vrw,1.0/60.0);// m/s/root-hour -> m/s^2/root-Hz
    this->vrw.scalermultiply(imu.ab_std,1/sqrt(dt));//m/s^2/root-Hz  ->  m/s^2
    this->arw.scalermultiply(imu.gb_std,1/sqrt(dt));//rad/s/root-Hz  ->  rad/s
    this->ab_fix.scalermultiply(imu.ab_fix,0.001*G2MSS);//mg -> m/s^2
    this->gb_fix.scalermultiply(imu.gb_fix,D2R);//deg/s -> rad/s;
    this->ab_drift.scalermultiply(imu.ab_drift,0.001*G2MSS);// mg -> m/s^2
    this->gb_drift.scalermultiply(imu.gb_drift,D2R);//deg/s -> rad/s;
    imu.ab_drift.scalermultiply(imu.ab_psd,sqrt(this->gb_corr.mat_get(0,0)));
    imu.gb_drift.scalermultiply(imu.gb_psd,sqrt(this->gb_corr.mat_get(0,0)));
}



GPS::~GPS()
{

}



IMU_DATA::~IMU_DATA()
{

}


GPS_DATA::~GPS_DATA()
{

}
   

    
