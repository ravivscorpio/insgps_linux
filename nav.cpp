#include "nav.h"

    NAV::~NAV()
    {

    }
    void NAV::init(IMU_DATA& imu_data,GPS_DATA& gps_data,GPS& gps, IMU& imu)
    {
        this->DCM_bn.I();
        this->lt=gps_data.pos.mat_get(0,0);
        this->lg=gps_data.pos.mat_get(1,0);
        this->h=gps_data.pos.mat_get(2,0);
        this->vel.mat_set(0,0,gps_data.vel.mat_get(0,0));
        this->vel.mat_set(1,0,gps_data.vel.mat_get(1,0));
        this->vel.mat_set(2,0,gps_data.vel.mat_get(2,0));
        this->RR.mat_set(0,0,gps.stdv.mat_get(0,0));
        this->RR.mat_set(1,1,gps.stdv.mat_get(0,1));
        this->RR.mat_set(2,2,gps.stdv.mat_get(0,2));
        this->RR.mat_set(3,3,gps.stdm.mat_get(0,0));
        this->RR.mat_set(4,4,gps.stdm.mat_get(0,1));
        this->RR.mat_set(5,5,gps.stdm.mat_get(0,2));
        
        this->Q.mat_set(0,0,    imu.arw.mat_get(0,0));
        this->Q.mat_set(1,1,    imu.arw.mat_get(0,1));
        this->Q.mat_set(2,2,    imu.arw.mat_get(0,2));
        this->Q.mat_set(3,3,    imu.vrw.mat_get(0,0));
        this->Q.mat_set(4,4,    imu.vrw.mat_get(0,1));
        this->Q.mat_set(5,5,    imu.vrw.mat_get(0,2));
        this->Q.mat_set(6,6, imu.gb_psd.mat_get(0,0));
        this->Q.mat_set(7,7 ,imu.gb_psd.mat_get(0,1));
        this->Q.mat_set(8,8  ,imu.gb_psd.mat_get(0,2));
        this->Q.mat_set(9,9  ,imu.ab_psd.mat_get(0,0));
        this->Q.mat_set(10,10,imu.ab_psd.mat_get(0,1));
        this->Q.mat_set(11,11,imu.ab_psd.mat_get(0,2));

        this->Pp.mat_set(0,0,imu.align_err.mat_get(0,0));
        this->Pp.mat_set(1,1,imu.align_err.mat_get(0,1));
        this->Pp.mat_set(2,2,imu.align_err.mat_get(0,2));
        this->Pp.mat_set(3,3,gps.stdv.mat_get(0,0));
        this->Pp.mat_set(4,4,gps.stdv.mat_get(0,1));
        this->Pp.mat_set(5,5,gps.stdv.mat_get(0,2));
        this->Pp.mat_set(6,6,gps.std.mat_get(0,0));
        this->Pp.mat_set(7,7, gps.std.mat_get(0,1));
        this->Pp.mat_set(8,8, gps.std.mat_get(0,2));
          this->Pp.mat_set(9,9,imu.gb_fix.mat_get(0,0));
        this->Pp.mat_set(10,10,imu.gb_fix.mat_get(1,0));
        this->Pp.mat_set(11,11,imu.gb_fix.mat_get(2,0));
        this->Pp.mat_set(12,12,imu.ab_fix.mat_get(0,0));
        this->Pp.mat_set(13,13,imu.ab_fix.mat_get(1,0));
        this->Pp.mat_set(14,14,imu.ab_fix.mat_get(2,0));
        this->Pp.mat_set(15,15,imu.gb_drift.mat_get(0,0));
        this->Pp.mat_set(16,16,imu.gb_drift.mat_get(1,0));
        this->Pp.mat_set(17,17,imu.gb_drift.mat_get(2,0));
        this->Pp.mat_set(18,18,imu.ab_drift.mat_get(0,0));
        this->Pp.mat_set(19,19,imu.ab_drift.mat_get(1,0));
        this->Pp.mat_set(20,20,imu.ab_drift.mat_get(2,0));

         this->xp.mat_set(9,0,imu.gb_fix.mat_get(0,0));
        this->xp.mat_set(10,0,imu.gb_fix.mat_get(1,0));
        this->xp.mat_set(11,0,imu.gb_fix.mat_get(2,0));
        this->xp.mat_set(12,0,imu.ab_fix.mat_get(0,0));
        this->xp.mat_set(13,0,imu.ab_fix.mat_get(1,0));
        this->xp.mat_set(14,0,imu.ab_fix.mat_get(2,0));
        this->xp.mat_set(15,0,imu.gb_drift.mat_get(0,0));
        this->xp.mat_set(16,0,imu.gb_drift.mat_get(1,0));
        this->xp.mat_set(17,0,imu.gb_drift.mat_get(2,0));
        this->xp.mat_set(18,0,imu.ab_drift.mat_get(0,0));
        this->xp.mat_set(19,0,imu.ab_drift.mat_get(1,0));
        this->xp.mat_set(20,0,imu.ab_drift.mat_get(2,0));    

        Matrix PPP(21,21,0),QQQ(12,12,0),RRR(6,6,0);
        Pp.mat_asign(PPP);
        RR.mat_asign(RRR);
        Q.mat_asign(QQQ);
        PPP.mat_mul(this->Pp,PPP);
        RRR.mat_mul(this->RR,RRR);
        QQQ.mat_mul(this->Q,QQQ);
                         
    }
    void NAV::earthrate(Matrix & omega_ie_n)
    {
         this->omega_ie_n.mat_set(0,0,7.2921155e-5*cos(this->lt));
         this->omega_ie_n.mat_set(0,1,0);
         this->omega_ie_n.mat_set(0,2,7.2921155e-5*-sin(this->lt));
    }
    void NAV::radius()
    {
        double e2=EARTH_E*EARTH_E;
        double den=1-e2*sin(this->lt)*sin(this->lt);
        this->R.mat_set(0,0,EARTH_A*(1-e2)/pow(den,3.0/2.0));
        this->R.mat_set(0,1,EARTH_A/sqrt(den));
    }
    void NAV::transportate(Matrix & omega_en_n)
    {
        this->radius();
        this->omega_en_n.mat_set(0,0,this->vel.mat_get(1,0)/(this->R.mat_get(1,0)+this->h));
        this->omega_en_n.mat_set(0,1,-this->vel.mat_get(0,0)/(this->R.mat_get(0,0)+this->h));
        this->omega_en_n.mat_set(0,2,-this->vel.mat_get(1,0)*tan(this->lt)/(this->R.mat_get(1,0)+this->h));
    }
    void NAV::update_att(Matrix & DCMbn_n,Matrix& wb,const double& dt)
    {
        Matrix S(3,3,0),A(3,3,0),S1(3,3,0),S2(3,3,0),S3(3,3,0),S4(3,3,0),S5(3,3,0),I(3,3,0),DCM_nb(3,3,0);
        double magn;
        Matrix euler_i(3,1,1);
        Matrix O1(3,1,0),O2(3,1,0),wb_n(3,1,0);
        this->DCM_bn.mat_t(DCM_nb);
        this->omega_ie_n.mat_add(O1,this->omega_en_n);
        DCM_nb.mat_mul(O2,O1);
        wb.mat_sub(wb_n,O2);
        wb_n.scalermultiply(euler_i,dt);
        magn=euler_i.mat_norm();
        I.I();
        if (magn!=0)
        {
            S.mat_skew(euler_i.mat_get(0,0),euler_i.mat_get(1,0),euler_i.mat_get(2,0));
            S.scalermultiply(S1,sin(magn)/magn);
            S.scalermultiply(S2,(1-cos(magn))/(magn*magn));
            S2.mat_mul(S3,S);
            S3.mat_add(S4,S1);
            S4.mat_add(A,I);
            
            
            //A = eye(3) + (sin(magn)/magn) * S + ((1-cos(magn))/(magn^2)) * S * S;
            

        }
        else
            I.mat_asign(A);

        this->DCM_bn.mat_mul(DCMbn_n,A);
        
        DCMbn_n.mat_asign(this->DCM_bn);
    
        // DCMbn_n = DCMbn * A;
        this->DCM_bn.dcm2euler(this->yaw,this->pitch,this->roll);



    }

    void NAV::gravity()
    {
        double hh,sin1,sin2,g0,R0,g;
        hh = abs(this->h);
        sin1 = sin(this->lt);
        sin2 = sin(2*this->lt);

        g0 = -9.780318 * ( 1 + 5.3024e-03*sin1*sin1 - 5.9e-06*sin2*sin2 );
        R0=sqrt(this->R.mat_get(0,0)*this->R.mat_get(1,0));
        g = (g0 / ((1 + (hh / R0))*(1 + (hh / R0))));
        this->g_n.mat_set(2,0,g);



    }
    void NAV::vel_update(Matrix & vel_n,Matrix& fn,const double& dt)
    {
        Matrix fn_c(3,1,0),fn_c1(3,1,0),S,S1(3,1,0),S2(3,1,0),S3,coriolis(3,1,0);
        fn.mat_asign(this->fn);
        S.mat_skew(this->vel.mat_get(0,0),this->vel.mat_get(1,0),this->vel.mat_get(2,0));
        this->omega_ie_n.scalermultiply(S1,2);
        S1.mat_add(S2,this->omega_en_n);
        S.mat_mul(coriolis,S2);
        coriolis.mat_add(S3,this->g_n);
        fn.mat_sub(fn_c1,S3);
        
        fn_c1.scalermultiply(fn_c1,dt);
        this->vel.mat_add(vel_n,fn_c1);
        vel_n.mat_asign(this->vel);
        //cout<<this->fn.mat_get(0,0)<<" "<<this->fn.mat_get(1,0)<<" "<<this->fn.mat_get(2,0)<<endl;
        //cout<<this->fn.mat_get(0,0)<<" "<<this->fn.mat_get(1,0)<<" "<<this->fn.mat_get(2,0)<<endl;

    }

    void NAV::pos_update(double& lt,double & lg , double& h,const double& dt){

        double vn_c,ve_c;

        this->h=this->h-vel.mat_get(2,0)*dt;
        this->radius(); 
        vn_c=this->vel.mat_get(0,0)/(this->R.mat_get(0,0)+this->h);
        this->lt=this->lt+vn_c*dt;
        this->radius(); 
        ve_c=this->vel.mat_get(1,0)/((this->R.mat_get(1,0)+this->h)*cos(this->lt));
        this->lg=this->lg+ve_c*dt;
        
        this->Tpr.mat_set(0,0,this->R.mat_get(0,0)+this->h);
        this->Tpr.mat_set(1,1,(this->R.mat_get(1,0)+this->h)*cos(this->lt));
        this->Tpr.mat_set(2,2,-1.0);
        
        lt=this->lt;
        
        lg=this->lg;
        h=this->h;
        
        cout.precision(10);
        
        cout<<this->pitch*180.0/3.1415<<" "<<this->roll*180.0/3.1415<<" "<<this->yaw*180.0/3.1415<<" "<<endl;
        
        //cout<<this->vel.mat_get(0,0)<<" "<<this->vel.mat_get(1,0)<<" "<<this->vel.mat_get(2,0)<<endl;
        
        
    }
    void NAV::get_Tpr(Matrix& Tpr)
    {

        this->Tpr.mat_asign(Tpr);
    }

    void NAV::get_F(IMU& imu)
    {
        const double Om=7.292115e-5;
        const double RO=sqrt(this->R.mat_get(0,0)*this->R.mat_get(1,0));
        const double Vn=this->vel.mat_get(0,0);
        const double Ve=this->vel.mat_get(1,0);
        const double Vd=this->vel.mat_get(2,0);
        const double lat=this->lt;
        const double fn=this->fn.mat_get(0,0);
        const double fe=this->fn.mat_get(1,0);
        const double fd=this->fn.mat_get(2,0);



        Matrix Fee(3,3,0),Fev(3,3,0),Fep(3,3,0);
        Matrix Fve(3,3,0),Fvv(3,3,0),Fvp(3,3,0);
        Matrix Fpe(3,3,0),Fpv(3,3,0),Fpp(3,3,0);
        Matrix Z(3,3,0),I(3,3,0),Fgg(3,3,0),Faa(3,3,0);
        Matrix DCMbn(3,3,0),M_DCMbn(3,3,0);
        Matrix F(21,21,0),G(21,12,0),H(6,21,0);

        this->DCM_bn.mat_asign(DCMbn);
        this->DCM_bn.scalermultiply(M_DCMbn,-1);
        I.I();
 


        Fee.mat_set(0,0,0);
        Fee.mat_set(0,1,-((Om * sin(lat)) + (Ve/(RO) * tan(lat))));
        Fee.mat_set(0,2,Vn/(RO));
        Fee.mat_set(1,0,(Om * sin(lat)) + (Ve/(RO) * tan(lat) ));
        Fee.mat_set(1,1,0);
        Fee.mat_set(1,2,(Om * cos(lat)) + (Ve/(RO))) ;
        Fee.mat_set(2,0,-Vn/(RO));
        Fee.mat_set(2,1,-Om * cos(lat) - Ve/RO);
        Fee.mat_set(2,2,0);

        Fev.mat_set(0,0,0);
        Fev.mat_set(0,1,1/(RO));
        Fev.mat_set(0,2,0);
        Fev.mat_set(1,0,-1/(RO));
        Fev.mat_set(1,1,0);
        Fev.mat_set(1,2,0);
        Fev.mat_set(2,0,0);
        Fev.mat_set(2,1,-tan(lat)/RO);
        Fev.mat_set(2,2,0);

        Fep.mat_set(0,0,-Om * sin(lat));
        Fep.mat_set(0,1,0);
        Fep.mat_set(0,2,-Ve/(RO*RO));
        Fep.mat_set(1,0,0);
        Fep.mat_set(1,1,0);
        Fep.mat_set(1,2,Vn/(RO*RO));
        Fep.mat_set(2,0, -Om*cos(lat) - (Ve/((RO)*(cos(lat)*cos(lat)))));
        Fep.mat_set(2,1,0);
        Fep.mat_set(2,2,(Ve * tan(lat)) / (RO*RO)) ;

        Fve.mat_set(0,0,0);
        Fve.mat_set(0,1,-fd);
        Fve.mat_set(0,2,fe);
        Fve.mat_set(1,0,fd);
        Fve.mat_set(1,1,0);
        Fve.mat_set(1,2,-fn);
        Fve.mat_set(2,0,-fe);
        Fve.mat_set(2,1,fn);
        Fve.mat_set(2,2,0);

        Fvv.mat_set(0,0,Vd/(RO));
        Fvv.mat_set(0,1,-2*((Om * sin(lat)) + ((Ve/(RO)) * tan(lat)))) ;
        Fvv.mat_set(0,2,Vn/RO) ;
        Fvv.mat_set(1,0,(2*Om * sin(lat)) + ( (Ve/(RO)) * tan(lat) ));
        Fvv.mat_set(1,1,(1/(RO)) * ((Vn * tan(lat)) + Vd)) ;
        Fvv.mat_set(1,2,2*Om * cos(lat) + (Ve/(RO)));
        Fvv.mat_set(2,0,(-2*Vn)/(RO));
        Fvv.mat_set(2,1,-2*(Om * cos(lat) +  (Ve/(RO)))) ;
        Fvv.mat_set(2,2,0); 

        Fvp.mat_set(0,0,-Ve*( (2*Om * cos(lat)) + (Ve/((RO)*(cos(lat)*cos(lat))))));
        Fvp.mat_set(0,1,0) ;
        Fvp.mat_set(0,2,1 / (RO*RO) * ( ((Ve*Ve) * tan(lat)) - (Vn * Vd) ));
        Fvp.mat_set(1,0,2*Om * ( (Vn * cos(lat)) - (Vd * sin(lat)) ) + ( (Vn * Ve) / (RO * (cos(lat)*cos(lat))) )) ;
        Fvp.mat_set(1,1,0) ;
        Fvp.mat_set(1,2,-(Ve/(RO*RO)) * (Vn*tan(lat)+Vd));
        Fvp.mat_set(2,0,2 * Om * Ve * sin(lat));
        Fvp.mat_set(2,1,0);
        Fvp.mat_set(2,2, 1/((RO*RO)) * ((Vn*Vn) + (Ve*Ve)));

        //Fpe=0

        Fpv.mat_set(0,0,1/(RO));
        Fpv.mat_set(0,1,0);
        Fpv.mat_set(0,2,0);
        Fpv.mat_set(1,0,0);
        Fpv.mat_set(1,1,1/((RO)*cos(lat)));
        Fpv.mat_set(1,2,0);
        Fpv.mat_set(2,0,0);
        Fpv.mat_set(2,1,0);
        Fpv.mat_set(2,2,-1);

        
        Fpp.mat_set(0,0,0);
        Fpp.mat_set(0,1,0);
        Fpp.mat_set(0,2,-Vn/(RO*RO));
        Fpp.mat_set(1,0,(Ve*tan(lat)) / (RO * cos(lat)));
        Fpp.mat_set(1,1,0);
        Fpp.mat_set(1,2,-Ve / (((RO*RO)) * cos(lat)));
        Fpp.mat_set(2,0,0);
        Fpp.mat_set(2,1,0);
        Fpp.mat_set(2,2,0);


        Fee.mat_sub_set(F,0,0);
        Fev.mat_sub_set(F,0,3);
        Fep.mat_sub_set(F,0,6);
        DCMbn.mat_sub_set(F,0,9);
        DCMbn.mat_sub_set(F,0,15);        
        
        Fve.mat_sub_set(F,3,0);
        Fvv.mat_sub_set(F,3,3);
        Fvp.mat_sub_set(F,3,6);
        M_DCMbn.mat_sub_set(F,3,12);
        M_DCMbn.mat_sub_set(F,3,18);

        Fpv.mat_sub_set(F,6,3);
        Fpp.mat_sub_set(F,6,6);

        Fgg.mat_set(0,0,-1/imu.gb_corr.mat_get(0,0));
        Fgg.mat_set(1,1,-1/imu.gb_corr.mat_get(0,1));
        Fgg.mat_set(2,2,-1/imu.gb_corr.mat_get(0,2));
        
        Faa.mat_set(0,0,-1/imu.ab_corr.mat_get(0,0));
        Faa.mat_set(1,1,-1/imu.ab_corr.mat_get(0,1));
        Faa.mat_set(2,2,-1/imu.ab_corr.mat_get(0,2));


        Fgg.mat_sub_set(F,15,15);
        Faa.mat_sub_set(F,18,18);




        

        DCMbn.mat_sub_set(G,0,0);
        M_DCMbn.mat_sub_set(G,3,3);
        I.mat_sub_set(G,15,6);
        I.mat_sub_set(G,18,9);

        I.mat_sub_set(H,0,3);
        this->Tpr.mat_sub_set(H,3,6);
        F.mat_asign(this->F);
        G.mat_asign(this->G);
        H.mat_asign(this->H);

        
    }
    void NAV::kalman(Matrix& xp,Matrix &z,double& dtg)
    {
        Matrix M(21,21,0),A(21,21,0),At(21,21,0),I(21,21,0),J(21,21,0),Jt(21,21,0),Qd(21,21,0),Pt(21,21,0),Gt(12,21,0),Ht(21,6,0),MHt(21,6,0);
        Matrix C(6,6,0),K(21,6,0),Kt(6,21,0),invC(6,6,0),Mx(6,1,0),MKt(6,21,0),MQ(12,21,0),PiJt(21,21,0);
        I.I();
        F.scalermultiply(M,dtg);
        I.mat_add(A,M);
        A.mat_t(At);
        this->G.mat_t(Gt);

        Q.mat_mul(MQ,Gt);
        MQ.scalermultiply(MQ,dtg);
        this->G.mat_mul(Qd,MQ);

        Pp.mat_mul(M,At);
        A.mat_mul(this->Pi,M);
        this->Pi.mat_add(this->Pi,Qd);
        this->Pi.mat_t(Pt);
        this->Pi.mat_add(this->Pi,Pt);
        this->Pi.scalermultiply(this->Pi,0.5);

        this->H.mat_t(Ht);
        this->Pi.mat_mul(MHt,Ht);
        this->H.mat_mul(C,MHt);
        C.mat_add(C,RR);
        C.mat_inv_GJ(invC);

        this->Pi.mat_mul(MHt,Ht);
        MHt.mat_mul(K,invC);
        
        A.mat_mul(this->xi,this->xp);
        H.mat_mul(Mx,this->xi);
        z.mat_sub(Mx,Mx);
        K.mat_mul(this->xp,Mx);
        this->xp.mat_add(this->xp,this->xi);

        K.mat_mul(J,H);
        I.mat_sub(J,J);

        J.mat_t(Jt);
        K.mat_t(Kt);
        this->RR.mat_mul(MKt,Kt);
        K.mat_mul(M,MKt);
        this->Pi.mat_mul(PiJt,Jt);
        J.mat_mul(A,PiJt);
        A.mat_add(this->Pp,M);

        this->xp.mat_asign(xp);



    }
    void NAV::update(Matrix& gb_fix,Matrix& ab_fix,Matrix& gb_drift,Matrix& ab_drift)
    {
        
        Matrix vel1(3,1,0);
        Matrix Z9(9,1,0);
        Matrix I(3,3,0),M(3,3,0),MM(3,3,0),E(3,3,0);
        I.I();
        
        this->roll=this->roll-this->xp.mat_get(0,0);
        this->pitch=this->pitch-this->xp.mat_get(1,0);
        this->yaw=this->yaw-this->xp.mat_get(2,0);  
        this->xp.mat_sub_get(vel1,3,0);
        this->vel.mat_sub(this->vel,vel1);
        this->lt=this->lt-this->xp.mat_get(6,0);
        this->lg=this->lg-this->xp.mat_get(7,0);
        this->h=this->h-this->xp.mat_get(8,0);
        this->xp.mat_sub_get(gb_fix,9,0);
        this->xp.mat_sub_get(ab_fix,12,0);
        this->xp.mat_sub_get(gb_drift,15,0);
        this->xp.mat_sub_get(ab_drift,18,0);
        E.mat_skew(this->xp.mat_get(0,0),this->xp.mat_get(1,0),this->xp.mat_get(2,0));
        E.mat_add(M,I);
        this->DCM_bn.mat_asign(MM);
        M.mat_mul(this->DCM_bn,MM);
        Z9.mat_sub_set(this->xp,0,0);
        



    }
