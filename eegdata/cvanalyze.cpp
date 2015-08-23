/* this analysis is a learning experience for me :) */
#include <list>
#include <iostream> // for standard I/O
#include <armadillo>

#include "cvanalyze.h"

/*
#include <iostream>
#include <armadillo>

using namespace std;
using namespace arma;

int main(int argc, char** argv)
  {
  mat A = randu<mat>(4,5);
  mat B = randu<mat>(4,5);
  
  cout << A*B.t() << endl;
  
  return 0;
  }
*/




#define TRIGGERVALUE 1

using namespace std;
using namespace arma;




void fft(Col<float> in,Col<float> &out);
//double getPSNR ( const mat& I1, const mat& I2);
//Scalar getMSSIM( const mat& I1, const mat& I2);





void getpatternimage(unsigned short index){
/*
  list<list>(10) trigdata = new list<fvec>();
  list<list>(10) trigspect = new list<fvec>();

  //data_array trigspect[4];
  for(int i=0;i<SAMPLESIZE;i++){
      trigdata[index](0,i)=triggers[index].channel1[i];
  } 
  dodft(trigdata[index],trigspect[index]); 
*/
  
  
  
  
  
}





    

float interp1( float x, fvec a )
{
    if( x <= 0 )  return a[0];
    if( x >= a.n_rows - 1 )  return a[a.n_rows-1];
    int j = int(x);
    return a[j] + (x - j) * (a[j+1] - a[j]);
}


    
void interpolate( fvec a, fvec &b )
{
    float step = float( a.n_rows - 1 ) / (b.n_rows - 1);
    for( unsigned int j = 0; j < b.n_rows; j ++ ){
        b[j] = interp1( j*step, a );

      
    }
}



void compare_signals(void){
  
  int SAMPLERATE=8;
  fvec chdata(SAMPLESIZE);
  fvec resampled(SAMPLESIZE*SAMPLERATE);
  cx_fvec spectrum(SAMPLESIZE);
  fvec result(SAMPLESIZE);
  
/*
 I want to draw a smooth line for the frequency components.
  I can only figure out how to do that if the samples and fft are equal length.
  Then I interpolate the data to get it the right scale for the screen.
*/

  for(unsigned int i=0;i<SAMPLESIZE;i++){
    chdata[i]=channel1[i]-ZEROSAMPLE;
  }
  
  spectrum = fft(chdata,SAMPLESIZE,0);
  result=abs(spectrum);
  result*=0.05;
  result=clamp(result, 0, 65535);
  interpolate(result,resampled);
  //result=clamp(result, 0, 65535);
  
    for(unsigned int i=0;i<SAMPLESIZE/2;i++){
	spectrum1[i]=resampled[i];
    }

   
    for(unsigned int i=0;i<SAMPLESIZE;i++){
    chdata[i]=channel2[i]-ZEROSAMPLE;
  }

  
  spectrum = fft(chdata,SAMPLESIZE);
  result=abs(spectrum);
  result*=0.05;
  result=clamp(result, 0, 65535);
  interpolate(result,resampled);
  
  
    for(unsigned int i=0;i<SAMPLESIZE/2;i++){
	spectrum2[i]=resampled[i];
    }


  for(unsigned int i=0;i<SAMPLESIZE;i++){
    chdata[i]=channel3[i]-ZEROSAMPLE;
  }

  spectrum = fft(chdata,SAMPLESIZE);
  result=abs(spectrum);
  result*=0.05;
  result=clamp(result, 0, 65535);
  interpolate(result,resampled);
  
    for(unsigned int i=0;i<SAMPLESIZE/2;i++){
	spectrum3[i]=resampled[i];
    }


  for(unsigned int i=0;i<SAMPLESIZE;i++){
    chdata[i]=channel4[i]-ZEROSAMPLE;
  }

  spectrum = fft(chdata,SAMPLESIZE);
  result=abs(spectrum);
  result*=0.05;
  result=clamp(result, 0, 65535);
  interpolate(result,resampled);
  
    for(unsigned int i=0;i<SAMPLESIZE/2;i++){
	spectrum4[i]=resampled[i];
    }

/*
    double psnrV;
    Scalar mssimV;

    for(unsigned int j=0;j<4;j++){
	for(unsigned int i=0;i<TRIGGERS;i++){ //Show the image captured in the window and repeat

	  psnrV = getPSNR(mat(chdata[j]),mat(trigdata[j][i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel "<<j<<" Trigger "<< i <<" data";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(mat(chdata[j]), mat(trigdata[j][i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	//spectrum
	  psnrV = getPSNR(mat(chspect[j]),mat(trigspect[j][i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel "<<j<<" Trigger "<< i << " spectrum ";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(mat(chspect[j]), mat(trigspect[j][i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	
	}

    }
  
  */
  
}

/* code from gnucap to do fft

void fft(COMPLEX* x, int n, int inv)
{
  int s = (inv) ? 1 : -1;
  int nxp, nxp2;
  for (nxp=n;  (nxp2=nxp/2) > 0;  nxp=nxp2) {
    double wpwr = M_TWO_PI / nxp;
    for (int m = 0;  m < nxp2;  ++m) {
      double argg = m * wpwr;
      COMPLEX w(cos(argg), s*sin(argg));
      for (int jj1 = m;  jj1+nxp-m <= n;  jj1 += nxp) {
	int jj2 = jj1 + nxp2;
	COMPLEX t = x[jj1] - x[jj2];
	x[jj1] += x[jj2];
	x[jj2] = t * w;
      }
    }
  }
  // unscramble 
  {
    int i, j;
    for ( i = j = 0;  i < n-1;  ++i) {
      if (i < j) {
	swap(x[i],x[j]);
      }
      int k;
      for (k = n/2;  k <= j;  k /= 2) {
	j -= k;
      }
      j += k;
    }
  }
  // fix level 
  if (!inv) {
    for (int i = 0;  i < n;  ++i) {
      x[i] /= n;
    }
  }
}

/*
double getPSNR(const mat& I1, const mat& I2)
{
    mat s1;
    absdiff(I1, I2, s1);       // |I1 - I2|
    s1.convertTo(s1, CV_32F);  // cannot make a square on 8 bits
    s1 = s1.mul(s1);           // |I1 - I2|^2

    Scalar s = sum(s1);        // sum elements per channel

    double sse = s.val[0] + s.val[1] + s.val[2]; // sum channels

    if( sse <= 1e-10) // for small values return zero
        return 0;
    else
    {
        double mse  = sse / (double)(I1.channels() * I1.total());
        double psnr = 10.0 * log10((255 * 255) / mse);
        return psnr;
    }
}

Scalar getMSSIM( const mat& i1, const mat& i2)
{
 const double C1 = 6.5025, C2 = 58.5225;
 int d     = CV_32F;

 mat I1, I2;
 i1.convertTo(I1, d);           // cannot calculate on one byte large values
 i2.convertTo(I2, d);

 mat I2_2   = I2.mul(I2);        // I2^2
 mat I1_2   = I1.mul(I1);        // I1^2
 mat I1_I2  = I1.mul(I2);        // I1 * I2


 mat mu1, mu2;   //
 GaussianBlur(I1, mu1, Size(11, 11), 1.5);
 GaussianBlur(I2, mu2, Size(11, 11), 1.5);

 mat mu1_2   =   mu1.mul(mu1);
 mat mu2_2   =   mu2.mul(mu2);
 mat mu1_mu2 =   mu1.mul(mu2);

 mat sigma1_2, sigma2_2, sigma12;

 GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
 sigma1_2 -= mu1_2;

 GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
 sigma2_2 -= mu2_2;

 GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
 sigma12 -= mu1_mu2;

 ///////////////////////////////// FORMULA ////////////////////////////////
 mat t1, t2, t3;

 t1 = 2 * mu1_mu2 + C1;
 t2 = 2 * sigma12 + C2;
 t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

 t1 = mu1_2 + mu2_2 + C1;
 t2 = sigma1_2 + sigma2_2 + C2;
 t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

 mat ssim_map;
 divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

 Scalar mssim = mean( ssim_map ); // mssim = average of ssim map
 return mssim;
}
*/
