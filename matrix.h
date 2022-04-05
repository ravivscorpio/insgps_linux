#ifndef __matrix__H__
#define __matrix__H__


#include <math.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <queue>
#include <algorithm>
#pragma once

#define DCM_

#define MAX_FILTER_LEN 100

using namespace std;

class Matrix
{
protected:
    int row,column;
    double *A;

public:
    
    Matrix(int row,int column,double val);
    explicit Matrix();
    ~Matrix();
    void showmat() const;
    void mat_set(int row,int column,double val);
    double& mat_get(int row,int column) const ;
    void I();
    void mat_mul(Matrix& mat_out, Matrix &mat) const;

    
    void scalermultiply(Matrix& mat_out,const double& k) const;
    void mat_t(Matrix& mat_out) const;
    void mat_asign(Matrix& mat_out) const;
    void mat_sub_set(Matrix& mat_out,const int row,const int col);
    void mat_sub_get(Matrix& mat_out,const int row,const int col) const;
    void mat_det(double& det, const Matrix& mat) const;
    void Z();
    void mat_inv(Matrix& mat_o, const Matrix& mat) const;
    void mat_inv_GJ(Matrix& mat_o);
    double mat_norm() const;
    void mat_add(Matrix& mat_out,const Matrix& mat) const;
    void mat_sub(Matrix& mat_out,const Matrix& mat) const;
    void mat_fun(Matrix& mat_out,const double (*fun)(double)) const;

    

};



class Euler :public  Matrix
{


public:
    
    
    using Matrix::Matrix;
    ~Euler();
 
    void mat_skew(const  double& x, const double& y,const double& z);
    void euler2dcm(const  double& yaw, const double& pitch,const double& roll);
    void dcm2euler(double& yaw,double& pitch,double& roll) const;
    void  Rates_bn(double& roll , double& pitch);
    void  Rates_nb(double& roll , double& pitch);
    

};

class Quaternion :public  Matrix
{


public:
    
    
    Quaternion();
    ~Quaternion();
 
    
    void euler2quat(const  double& yaw, const double& pitch,const double& roll);
    void qua2dcm(Euler& DCMbn) const;
    void qua_update(const Quaternion& q, const Matrix & wb,const double dt);
    

};
class Filter
{
private:
    int N;
    double a[MAX_FILTER_LEN];
    double b[MAX_FILTER_LEN];
    deque<double> data_in;
    deque<double> data_out;
    deque<double> data;
    
public:
    
    Filter(const int N,const double* b,const double* a);
    Filter(const int N);
    ~Filter();
    void median_filter(double& filter_output,const double& x);
    void filter(double& filter_output,const double& x);
};
#endif
