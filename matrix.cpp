#include "matrix.h"

using namespace std;


Matrix::Matrix()
{
        this->row=3;
        this->column=3;
        A=(double*) new double[9];

   

    
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

void Matrix::showmat(){
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
void Matrix::mat_mul(Matrix& mat_out,Matrix& mat)
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





void Matrix::scalermultiply(Matrix& mat_out,const double& k)
{

     
     for(int i=0;i<(this->row*this->column);i++)
     {
             mat_out.A[i]=this->A[i]*k;
     }

    

}


void Matrix::mat_t(Matrix& mat_out)
{
	
	int k=0;
	for(int i=1;i<=this->column;i++){
		for(int j=1;j<=this->row;j++){
			mat_out.A[k]=this->A[(j-1)*this->column+i-1];
			k+=1;
		}
	}

}
void Matrix::mat_asign(Matrix& mat_out)
{
	
	int k=0;
	for(int i=1;i<=this->column;i++){
		for(int j=1;j<=this->row;j++){
			mat_out.A[k]=this->A[k];
			k+=1;
		}
	}

}


double Matrix::mat_norm()
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

void Matrix::mat_sub_get(Matrix& mat_out,const int row,const int col)
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

void Matrix::euler2dcm(const  double& yaw, const double& pitch,const double& roll)
{

    Matrix C1,C2,C3,C2C1,C3C2C1;
    
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

void Matrix::dcm2euler(double& yaw,double& pitch,double& roll)
{

roll=atan(this->mat_get(2,1)/this->mat_get(2,2));
pitch=-asin(this->mat_get(2,0));
yaw=atan2(this->mat_get(1,0),this->mat_get(0,0));
    
}





void Matrix::mat_skew(const  double& x, const double& y,const double& z)
{

    
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
void Matrix::mat_add(Matrix& mat_out,const Matrix& mat)
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

void Matrix::mat_sub(Matrix& mat_out,const Matrix& mat)
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
    if (N%2==0)
        filter_output=(data[N/2]+data[N/2-1])/2;
    else
        filter_output=data[N/2];

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











