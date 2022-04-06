#include "matrix.h"

using namespace std;


Matrix::Matrix()
{
        this->row=3;
        this->column=3;
        A=(double*) new double[9];
        for (int i=0;i<row*column;i++)
              A[i]=0;
   

    
}

Matrix::Matrix(int row,int column,double val)
{
        
        this->row=row;
        this->column=column;
        A=(double*) new double[row*column];
        for (int i=0;i<row*column;i++)
              A[i]=val;
   
}

Matrix::~Matrix()
{
        delete this->A;
}

void Matrix::showmat() const
{
        cout.precision(10);
	if(this->row>0&&this->column>0){
		int k=0;
		cout<<"[";
		for(int i=0;i<this->row;i++){
			for (int j=0;j<this->column;j++){
				if(j<this->column){
					cout<<A[k++]<<"  ";
				}else{
					cout<<A[k++];
				}
			}
			if(i<this->row){
				cout<<"]\n";
			}else{
				cout<<"]\n";
			}
		}	
		cout<<"\n";
	}else{
		cout<<"[]";
	}
}

void Matrix::mat_set(int row,int column,double val){
        A[row*this->column+column]=val;

}

double& Matrix::mat_get(int row,int column) const {
        return A[row*this->column+column];

}
void Matrix::I(){
        for (int i=0;i<((this->column)*(this->row));i++)
                if ((i%this->row)==(i/this->column))
                        A[i]=1;

}
void Matrix::Z(){
        for (int i=0;i<row*column;i++)
              A[i]=0;

}
void Matrix::mat_mul(Matrix& mat_out,Matrix& mat) const
{

        Matrix M(mat_out.row,mat_out.column,0);
        int r1=this->row;
	int r2=mat.row;
	int c1=this->column;
	int c2=mat.column;
        
	if (r1==1&&c1==1){
		mat.scalermultiply(mat_out,this->A[0]);

	}else if (r2==1&&c2==1){
		this->scalermultiply(mat_out,mat.A[0]);

	}


	for(int i=1;i<=r1;i++){
		for(int j=1;j<=c2;j++){
			double de=0;
			for(int k=1;k<=r2;k++){
                                if ((this->A[(i-1)*this->column+k-1]==0) || (mat.A[(k-1)*mat.column+j-1]==0)) continue;
				de+=this->A[(i-1)*this->column+k-1]*mat.A[(k-1)*mat.column+j-1];
			}
			mat_out.A[(i-1)*mat_out.column+j-1]=de;
		}
	}
        

}





void Matrix::scalermultiply(Matrix& mat_out,const double& k) const
{

     
     for(int i=0;i<(this->row*this->column);i++)
     {
             mat_out.A[i]=this->A[i]*k;
     }

    

}


void Matrix::mat_t(Matrix& mat_out) const
{
	
	int k=0;
	for(int i=1;i<=this->column;i++){
		for(int j=1;j<=this->row;j++){
			mat_out.A[k]=this->A[(j-1)*this->column+i-1];
			k+=1;
		}
	}

}
void Matrix::mat_asign(Matrix& mat_out) const
{
	
	int k=0;
	for(int i=1;i<=this->column;i++){
		for(int j=1;j<=this->row;j++){
			mat_out.A[k]=this->A[k];
			k+=1;
		}
	}

}


double Matrix::mat_norm() const  
{
	
	int k=0;
        double norm=0;

	for(int i=1;i<=this->column;i++){
		for(int j=1;j<=this->row;j++){
			norm+=this->A[k]*this->A[k];
			k+=1;
		}
	}

        return sqrt(norm);

}

void Matrix::mat_sub_set(Matrix& mat_out,const int row,const int col)
{
        double val;
	for(int i=0;i<this->column;i++){
		for(int j=0;j<this->row;j++){
                        val = this->mat_get(j,i);
                        mat_out.mat_set(row+j,col+i,val);

                }
        }
}

void Matrix::mat_sub_get(Matrix& mat_out,const int row,const int col) const
{
        double val;
        int c=mat_out.column;
        int r=mat_out.row;

	for(int i=0;i<c;i++){
		for(int j=0;j<r;j++){
                        val = this->mat_get(row+j,col+i);
                        mat_out.mat_set(j,i,val);

                }
        }
}











void Matrix::mat_inv_GJ(Matrix& mat_o) 
{


int n=this->column;
int m=this->row;

Matrix a(m,n*2,0);
Matrix I(m,n,0);
I.I();
this->mat_sub_set(a,0,0);
I.mat_sub_set(a,0,n);


int k,i,j;
double ratio;

                 /* Applying Gauss Jordan Elimination */
                 for(i=0;i<n;i++)
                 {
                          if(a.mat_get(i,i) == 0.0)
                          {
//error
                          }
                          for(j=0;j<n;j++)
                          {
                                   if(i!=j)
                                   {
                                            ratio = a.mat_get(j,i)/a.mat_get(i,i);
                                            for(k=0;k<=2*n;k++)
                                            {
                                                a.mat_set(j,k,a.mat_get(j,k) - ratio*a.mat_get(i,k));
                                            }
                                   }
                          }
                 }
                 /* Row Operation to Make Principal Diagonal to 1 */
                 for(i=0;i<n;i++)
                 {
                          for(j=n;j<2*n;j++)
                          {
                                a.mat_set(i,j, a.mat_get(i,j)/a.mat_get(i,i));
                          }
                 }
                 for  (i=0;i<m;i++)
                        for (j=0;j<n;j++)
                             mat_o.mat_set(i,j,a.mat_get(i,j+n));



}
void Matrix::mat_add(Matrix& mat_out,const Matrix& mat) const
{
	int r=this->row;
	int c=this->column;
	int k=0;	
        Matrix C(r,c,0);
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			mat_out.A[k]=this->A[k]+mat.A[k];
			k+=1;
		}
	}



}

void Matrix::mat_sub(Matrix& mat_out,const Matrix& mat) const
{
	int r=this->row;
	int c=this->column;

	int k=0;	
	for(int i=0;i<r;i++){
		for(int j=0;j<c;j++){
			mat_out.A[k]=this->A[k]-mat.A[k];
			k+=1;
		}
	}


}

void Matrix::mat_fun(Matrix& mat_out,const double (*fun)(double)) const
{
        for (int i=0;i<(this->column*this->row);i++)
        {
                mat_out.A[i]=fun(this->A[i]);
        }
}


Euler::~Euler()
{

}
void Euler::euler2dcm(const  double& yaw, const double& pitch,const double& roll)
{
//ned to body
    Euler C1,C2,C3,C2C1,C3C2C1;
    
    double cy=cos(yaw);
    double sy=sin(yaw);
    double cp=cos(pitch);
    double sp=sin(pitch);   
    double cr=cos(roll);
    double sr=sin(roll);    

    C1.A[0]=cy;    C1.A[1]=sy; C1.A[2]=0;
    C1.A[3]=-sy;   C1.A[4]=cy; C1.A[5]=0;
    C1.A[6]=0;     C2.A[7]=0  ; C1.A[8]=1;

    C2.A[0]=cp;    C2.A[1]=0; C2.A[2]=-sp;
    C2.A[3]=0;   C2.A[4]=1; C2.A[5]=0;
    C2.A[6]=sp;     C2.A[7]=0  ;C2.A[8]=cp;

    C3.A[0]=1;   C3.A[1]=0;   C3.A[2]=0;
    C3.A[3]=0;   C3.A[4]=cr;  C3.A[5]=sr;
    C3.A[6]=0;   C3.A[7]=-sr; C3.A[8]=cr;   
    
    C2.mat_mul(C2C1,C1);
    C3.mat_mul(*this,C2C1);
    
}

void Euler::dcm2euler(double& yaw,double& pitch,double& roll) const
{
//body to ned
roll=atan(this->mat_get(2,1)/this->mat_get(2,2));
pitch=-asin(this->mat_get(2,0));
yaw=atan2(this->mat_get(1,0),this->mat_get(0,0));

}





void Euler::mat_skew(const  double& x, const double& y,const double& z)
{
//body to ned
    
    this->A[0]=0;
    this->A[1]=-z;
    this->A[2]=y;
    this->A[3]=z;
    this->A[4]=0;
    this->A[5]=-x;
    this->A[6]=-y;
    this->A[7]=x;
    this->A[8]=0;
    
}

void  Euler::Rates_bn(double& roll,  double& pitch)
{

this->A[0]=1;
this->A[1]=sin(roll)*tan(pitch);
this->A[2]=cos(roll)*tan(pitch);
this->A[3]=0;
this->A[4]=cos(roll);
this->A[5]=-sin(roll);
this->A[6]=0;
this->A[7]=sin(roll)/cos(pitch);
this->A[8]=cos(roll)/cos(pitch);

}

void  Euler::Rates_nb(double& roll,  double& pitch)
{

this->A[0]=1;
this->A[1]=0;
this->A[2]=-sin(pitch);
this->A[3]=0;
this->A[4]=cos(roll);
this->A[5]=cos(pitch)*sin(roll);
this->A[6]=0;
this->A[7]=-sin(roll);
this->A[8]=cos(roll)*cos(pitch);

}




Quaternion::Quaternion()
{
        this->row=4;
        this->column=1;
        A=(double*) new double[4];
        for (int i=0;i<row*column;i++)
              A[i]=0;     
}
Quaternion::~Quaternion()
{
    
}
void Quaternion::euler2quat(const  double& yaw, const double& pitch,const double& roll)
{
//ZRX rotation
        Matrix c_eul(1,3,0),s_eul(1,3,0);
        c_eul.mat_set(0,0,cos(yaw/2));
        c_eul.mat_set(0,1,cos(pitch/2));
        c_eul.mat_set(0,2,cos(roll/2));
        s_eul.mat_set(0,0,sin(yaw/2));
        s_eul.mat_set(0,1,sin(pitch/2));
        s_eul.mat_set(0,2,sin(roll/2));
        
        this->A[3]= c_eul.mat_get(0,0)*c_eul.mat_get(0,1)*c_eul.mat_get(0,2) + s_eul.mat_get(0,0)*s_eul.mat_get(0,1)*s_eul.mat_get(0,2);
        this->A[0]= c_eul.mat_get(0,0)*c_eul.mat_get(0,1)*s_eul.mat_get(0,2) - s_eul.mat_get(0,0)*s_eul.mat_get(0,1)*c_eul.mat_get(0,2);
        this->A[1]= c_eul.mat_get(0,0)*s_eul.mat_get(0,1)*c_eul.mat_get(0,2) + s_eul.mat_get(0,0)*c_eul.mat_get(0,1)*s_eul.mat_get(0,2);
        this->A[2]= s_eul.mat_get(0,0)*c_eul.mat_get(0,1)*c_eul.mat_get(0,2) - c_eul.mat_get(0,0)*s_eul.mat_get(0,1)*s_eul.mat_get(0,2);

}

void Quaternion::qua2dcm(Euler & DCMbn) const
{
        //body to ned
        double a,b,c,d;

        a=this->A[3];
        b=this->A[0];
        c=this->A[1];
        d=this->A[2];

        DCMbn.mat_set(0,0, a*a + b*b - c*c - d*d);
        DCMbn.mat_set(0,1, 2*(b*c - a*d));
        DCMbn.mat_set(0,2, 2*(a*c + b*d));
        DCMbn.mat_set(1,0, 2*(a*d + b*c));
        DCMbn.mat_set(1,1, a*a - b*b + c*c - d*d);
        DCMbn.mat_set(1,2, 2*(c*d - a*b));
        DCMbn.mat_set(2,0, 2*(b*d - a*c));
        DCMbn.mat_set(2,1, 2*(c*d + a*b));
        DCMbn.mat_set(2,2, a*a - b*b - c*c + d*d);        


}
void Quaternion::qua_update(const Quaternion& q,const Matrix & wb,const double dt)
{
        double norm,co,si,qw1,qw2,qw3,qw4;
        Matrix wnorm(3,1,0);
        Matrix Om(4,4,0);
        Matrix qout(4,1,0);
        double (*c)(double);
        c=&cos;

        //wb.mat_asign(wnorm);
        norm=wb.mat_norm();
        if (norm==0)
        {
                return;
        }
        co=cos(0.5*norm*dt);
        si=sin(0.5*norm*dt);

        wb.scalermultiply(wnorm,1/norm);

        qw1=wnorm.mat_get(0,0)*si;
        qw2=wnorm.mat_get(1,0)*si;
        qw3=wnorm.mat_get(2,0)*si;
        qw4=co;
        
        Om.mat_set(0,0,qw4);Om.mat_set(0,1,qw3);Om.mat_set(0,2,-qw2);Om.mat_set(0,3,qw1);
        Om.mat_set(1,0,-qw3);Om.mat_set(1,1,qw4);Om.mat_set(1,2,qw1);Om.mat_set(1,3,qw2);
        Om.mat_set(2,0,qw2);Om.mat_set(2,1,-qw1);Om.mat_set(2,2,qw4);Om.mat_set(2,3,qw3);
        Om.mat_set(3,0,qw1);Om.mat_set(3,1,-qw2);Om.mat_set(3,2,-qw3);Om.mat_set(3,3,qw4);

        Om.mat_mul(qout,(Matrix&)q);
        qout.mat_asign(*this);
     
        
        

        
        




}
Filter::Filter(const int N,const double* b,const double* a)
{
        this->N=N;
        for (int i=0;i<N;i++)
        {
                this->b[i]=b[i];
                this->data_in.push_back(0);
                this->data_out.push_back(0);
        }
        for (int i=0;i<N-1;i++)
        {
                this->a[i]=a[i+1];
        }
        this->a[N-1]=0;
}
Filter::Filter(const int N)
{
        this->N=N;
        
        for (int i=0;i<N;i++)
                this->data.push_back(0);
        
}
Filter::~Filter()
{
        
}
void Filter::median_filter(double& filter_output,const double& x)
{
    
    
    data.pop_front();
    data.push_back(x);
    std::sort(data.begin(),data.begin()+N);
    
    if (this->N%2==0)
        filter_output=(data[this->N/2]+data[this->N/2-1])/2;
    else
        filter_output=data[this->N/2];


}

void Filter::filter(double& filter_output,const double& x)
{


        filter_output=0;
        data_in.push_front(x);
        data_in.pop_back();
        
        for (int i=0;i<N;i++)
        {
                filter_output+=b[i]*data_in[i]-a[i]*data_out[i];

        
        }
        data_out.pop_back();
        data_out.push_front(filter_output);
        
}
 void Filter::coef()
 {

        for (int i=0;i<MAX_FILTER_LEN;i++)
        {
            this->a[i]=0;
            this->b[i]=1.0/100.0;
        }
 }











