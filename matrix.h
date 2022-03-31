#ifndef __matrix__H__
#define __matrix__H__


#include <math.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <queue>
#include <algorithm>
#pragma once

#define MAX_FILTER_LEN 100

using namespace std;

class Matrix
{
private:
    int row,column;
    double *A;

public:
    
    Matrix(int row,int column,double val);
    Matrix();
    ~Matrix();
    void showmat();
    void mat_set(int row,int column,double val);
    double& mat_get(int row,int column) const ;
    void I();
    void mat_mul(Matrix& mat_out, Matrix &mat);

    
    void scalermultiply(Matrix& mat_out,const double& k);
    void mat_t(Matrix& mat_out);
    void mat_asign(Matrix& mat_out);
    void mat_sub_set(Matrix& mat_out,const int row,const int col);
    void mat_sub_get(Matrix& mat_out,const int row,const int col);
    void mat_det(double& det, const Matrix& mat);

    void mat_inv(Matrix& mat_o, const Matrix& mat);
    void mat_skew(const  double& x, const double& y,const double& z);
    void euler2dcm(const  double& yaw, const double& pitch,const double& roll);
    void dcm2euler(double& yaw,double& pitch,double& roll);
    void mat_inv_GJ(Matrix& mat_o);
    double mat_norm();
    void mat_add(Matrix& mat_out,const Matrix& mat);
    void mat_sub(Matrix& mat_out,const Matrix& mat);
    void median_filter(double& filter_output,const double& x,const int N);
    void filter(double& filter_output,const double& x,const int& N,const double& a,const double& b);
    

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
