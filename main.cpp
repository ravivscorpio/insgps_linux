

#include <stdlib.h>
#include "matrix.h"
#include "ins_gps.h"
#include "nav.h"
#include "serial.h"
#include <string>
#include <iostream>
#include <fstream>
#include "MidjDrv.h"
#include <sstream>



using namespace std;


unsigned char midg_data[4096];
unsigned int len=0;
MIDJ_InsMsg* InsMsg;
Matrix cal(8,1,0);
int main()
{
    int kkk=0;
    string fname="INS_GPS_16072015_123908.kf2";
    string line,word;
    fstream file (fname,ios::in);
    fstream file_out ("pitch.txt",ios::out);

    IMU imu1,imu2;
    
    //mat.showmat();
    imu1.imu_si_errors(imu2,0.01);
    GPS gps;  

    GPS_DATA gps_data;
    IMU_DATA imu_data;

    NAV nav;

    double yaw,pitch,roll,yaw_measure;
    double lt,lg,h;
    Matrix vel(3,1,0);
    Matrix fn(3,1,0);
    Euler DCMnb;
    Euler DCMbn;

    Matrix xp(21,1,9);



    Matrix wb_corrected(3,1,0);
    Matrix fb_corrected(3,1,0);

    Matrix omega_ie_n(3,1,0);
    Matrix omega_en_n(3,1,0);

    Matrix Tpr;
    Matrix z(6,1,0),zp(3,1,0),zv(3,1,0),zp_angle(3,1,0);
    double data[15];
    double dtg=0.2,dti=0.01,dty=10,tg_old=0,ti_old=0,t=0,ty_old=0;
    bool gps_valid=false,yaw_valid=false,first_init_gps=false;
    int fff=0;
    double a[50],b[50];
    int M=5,N=15;
    MidjDrv_Init ();
    for (int i=0;i<N;i++)
    {
        a[i]=0;
        b[i]=1.0   /N;
    }
    Filter p1(M),q1(M),r1(M),ax1(M),ay1(M),az1(M),t1(M);
    Filter p2(N,b,a),q2(N,b,a),r2(N,b,a),ax2(N,b,a),ay2(N,b,a),az2(N,b,a);
    double p,q,r,ax,ay,az;
    double p3,q3,r3,ax3,ay3,az3;
    int t_old;
    int idx=0;
    if (file.is_open())
    {
        for (int iii=0;iii<20;iii++)
            getline(file,line);
        
        while(true)
        {

            if  ( MidjDrv_Rx (midg_data))
            {
            
                InsMsg=(MIDJ_InsMsg*)midg_data;
               
            }
            else
                continue;
            
            t=InsMsg->TimeStamp/1000.0;
            /*t1.median_filter(t,InsMsg->TimeStamp/1000.0);
            p1.median_filter(p,InsMsg->xRate/100.0/180*3.14159265358);
            q1.median_filter(q,InsMsg->yRate/100.0/180*3.14159265358);
            r1.median_filter(r,InsMsg->zRate/100.0/180*3.14159265358);
            ax1.median_filter(ax,InsMsg->xAcc/1000.0*9.81);
            ay1.median_filter(ay,InsMsg->yAcc/1000.0*9.81);
            az1.median_filter(az,InsMsg->zAcc/1000.0*9.81);*/
            p=InsMsg->xRate/100.0/180.0*3.14159265358;
            q=InsMsg->yRate/100.0/180.0*3.14159265358;
            r=InsMsg->zRate/100.0/180.0*3.14159265358;
            ax=InsMsg->xAcc/1000.0*9.81;
            ay=InsMsg->yAcc/1000.0*9.81;
            az=InsMsg->zAcc/1000.0*9.81;
	        for (int i=0;i<50;i++)
	            midg_data[i]=0;
            //p2.filter(p,p3);
            //q2.filter(q,q3);
            //r2.filter(r,r3);
            //ax2.filter(ax,ax3);
            //ay2.filter(ay,ay3);
            //az2.filter(az,az3);
           //cout<<" "<<t<<" "<<p<<" "<<" "<<q<<" "<<r<<" "<<ax<<" "<<ay<<" "<<az<<endl;
           //continue;


            imu_data.wb.mat_set(0,0,p);
            imu_data.wb.mat_set(1,0,q);
            imu_data.wb.mat_set(2,0,r);
            imu_data.fb.mat_set(0,0,ax);
            imu_data.fb.mat_set(1,0,ay);
            imu_data.fb.mat_set(2,0,az);
            /*imu_data.wb.mat_set(0,0,InsMsg->xRate/100.0/180*3.14159265358);
            imu_data.wb.mat_set(1,0,InsMsg->yRate/100.0/180*3.14159265358);
            imu_data.wb.mat_set(2,0,InsMsg->zRate/100.0/180*3.14159265358);
            imu_data.fb.mat_set(0,0,InsMsg->xAcc/1000.0*9.81);
            imu_data.fb.mat_set(1,0,InsMsg->yAcc/1000.0*9.81);
            imu_data.fb.mat_set(2,0,InsMsg->zAcc/1000.0*9.81);*/

            //bool gps_valid= (data[1]==1);


        
            /*imu_data.wb.mat_set(0,0,data[11]);
            imu_data.wb.mat_set(1,0,data[12]);
            imu_data.wb.mat_set(2,0,data[13]);
            imu_data.fb.mat_set(0,0,data[8]);
            imu_data.fb.mat_set(1,0,data[9]);
            imu_data.fb.mat_set(2,0,data[10]);
            gps_data.pos.mat_set(0,0,data[2]);
            gps_data.pos.mat_set(1,0,data[3]);
            gps_data.pos.mat_set(2,0,data[4]);

            gps_data.vel.mat_set(0,0,data[5]);
            gps_data.vel.mat_set(1,0,data[6]);
            gps_data.vel.mat_set(2,0,data[7]);
            bool gps_valid= (data[1]==1);
            t=data[0];*/

        
        
            if (!first_init_gps)
            {
                tg_old=t;
                ti_old=t;

                if (idx<200) 
                {
                
                    nav.calibrate(cal,imu_data.wb,imu_data.fb);
                    cout<<cal.mat_get(1,0)/3.14*180<<" "<<cal.mat_get(0,0)/3.14*180<<endl;
                    idx++;
                    continue;
                }
                else
                {
                    
                    first_init_gps=true;
                }
                
                nav.vel_update(vel,fn,0);
                nav.pos_update(lt,lg,h,0);
                /*imu_data.wb.mat_set(0,0,0.0793e-3);
                imu_data.wb.mat_set(1,0,-0.0288e-3);
                imu_data.wb.mat_set(2,0,-0.1781e-3);
                imu_data.fb.mat_set(0,0,0.0141);
                imu_data.fb.mat_set(1,0,0.001);
                imu_data.fb.mat_set(2,0,-0.9565);*/
                gps_data.pos.mat_set(0,0,0.56364058);
                gps_data.pos.mat_set(1,0,0.60852916);
                gps_data.pos.mat_set(2,0,44.360001);

                gps_data.vel.mat_set(0,0,0.0);
                gps_data.vel.mat_set(1,0,0.0);
                gps_data.vel.mat_set(2,0,0.0);

                imu2.gb_fix.mat_set(0,0,+cal.mat_get(2,0));
                imu2.gb_fix.mat_set(1,0,+cal.mat_get(3,0));
                imu2.gb_fix.mat_set(2,0,+cal.mat_get(4,0));
                imu2.ab_fix.mat_set(0,0,+cal.mat_get(5,0));
                imu2.ab_fix.mat_set(1,0,+cal.mat_get(6,0));
                imu2.ab_fix.mat_set(2,0,+cal.mat_get(7,0));
                nav.init(imu_data,gps_data,gps,imu2,cal.mat_get(0,0),cal.mat_get(1,0),yaw);
                
                

                   

                


                    

                
            }
            

        
            
                dti=t-ti_old;
                
                if ((dti>0.1)||(dti<0))
                    continue;
                
                ti_old=t;
                imu2.gb_fix.mat_add(wb_corrected,imu_data.wb);
                wb_corrected.mat_add(wb_corrected,imu2.gb_drift);
                imu2.ab_fix.mat_add(fb_corrected,imu_data.fb);
                fb_corrected.mat_add(fb_corrected,imu2.ab_drift);

                nav.earthrate(omega_ie_n);
                nav.transportate(omega_en_n);
                nav.update_att(DCMbn,wb_corrected,dti);
                DCMbn.dcm2euler(yaw,pitch,roll);
                cout<<t<<" "<<roll/3.14*180<<" "<<pitch/3.14*180<<" "<<yaw/3.14*180<<endl;
                nav.gravity();
                DCMbn.mat_mul(fn,fb_corrected);
                 
                nav.vel_update(vel,fn,dti);
                
                nav.pos_update(lt,lg,h,dti);

            if (gps_valid)
            {
                dtg=t-tg_old;
                if ((dtg>1)||(dtg<-1))
                    continue;
            
                tg_old=t;
                
                nav.get_Tpr(Tpr);
                zp_angle.mat_set(0,0,lt-gps_data.pos.mat_get(0,0));
                zp_angle.mat_set(1,0,lg-gps_data.pos.mat_get(1,0));
                zp_angle.mat_set(2,0,h-gps_data.pos.mat_get(2,0));
                vel.mat_sub(zv,gps_data.vel);
                Tpr.mat_mul(zp,zp_angle);
                zp.mat_add(zp,gps.larm);
                zv.mat_sub_set(z,0,0);
                zp.mat_sub_set(z,3,0);
                nav.get_F(imu2);
    
                nav.kalman(xp,z,dtg);
                
		        nav.update(imu2.gb_fix,imu2.ab_fix,imu2.gb_drift,imu2.ab_drift);

		gps_valid=false;
	   }
       if (kkk%10==0)
	   {
               gps_valid=true;
                
           }     
              
           
            

                
            kkk++;
            
        }
        if (kkk%505==0)
           {
              dty=t-ty_old;
              ty_old=t;
              yaw_valid=true;
              yaw_measure=0;
              nav.update_yaw(DCMbn,yaw_measure,dty);
           }

    }
    printf("end");






    return 0;
}
