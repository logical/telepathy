
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

#include "cvanalyze.h"

#define TRIGGERVALUE 1

using namespace std;
using namespace cv;

typedef Matx<float, 1, SAMPLES*4> Matxdata;
typedef Matx<float, 1, SAMPLES> Matxspect;

vector<float> trigdata[4][10];
vector<float> trigspect[4][10];

void dodft(vector<float> in,vector<float> &out);
double getPSNR ( const Mat& I1, const Mat& I2);
Scalar getMSSIM( const Mat& I1, const Mat& I2);





void getpatternimage(unsigned short index){
  trigdata[0][index].clear();
  trigdata[1][index].clear();
  trigdata[2][index].clear();
  trigdata[3][index].clear();
  
  for(int i=0;i<SAMPLES*4;i++){
      trigdata[0][index].push_back(triggers[index].channel1[i]);
      trigdata[1][index].push_back(triggers[index].channel2[i]);
      trigdata[2][index].push_back(triggers[index].channel3[i]);
      trigdata[3][index].push_back(triggers[index].channel4[i]);
  } 

  dodft(trigdata[0][index],trigspect[0][index]); 
  dodft(trigdata[1][index],trigspect[1][index]); 
  dodft(trigdata[2][index],trigspect[2][index]); 
  dodft(trigdata[3][index],trigspect[3][index]); 
  
}

void dodft(vector<float> in,vector<float> &out){
  

    Mat I=Mat(in);
//     Mat padded;                            //expand input image to optimal size
//     int m = getOptimalDFTSize( I.rows );
//     int n = getOptimalDFTSize( I.cols ); // on the border add zero values
//     copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(I), Mat::zeros(I.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
    dft(complexI, complexI,DFT_SCALE);            // this way the result may fit in the source matrix
    
    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
      split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
      magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
      Mat magI = planes[0];

      magI += Scalar::all(1);                    // switch to logarithmic scale
      log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
//    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));


    
//    magI.col(0).copyTo(out);

    
      
      Mat result(1,SAMPLES,CV_32F);
      magI(Rect(0,0,1,SAMPLES)).copyTo(result);
      normalize(result, result, 0, 4096, CV_MINMAX);
      result(Rect(0,0,1,SAMPLES)).copyTo(out);
    
    
  
}

void compare_signals(void){
  
//4 images per pattern one for each channel of the signal and the fft of the signal
//THIS MIGHT CHANGE

  vector<float> chdata[4];
  vector<float> chspect[4];

  for(unsigned int i=0;i<(SAMPLES*4);i++){
    chdata[0].push_back(channel1[i]-ZEROSAMPLE);
    chdata[1].push_back(channel2[i]-ZEROSAMPLE);
    chdata[2].push_back(channel3[i]-ZEROSAMPLE);
    chdata[3].push_back(channel4[i]-ZEROSAMPLE);
  }
  
  dodft(chdata[0],chspect[0]);
  dodft(chdata[1],chspect[1]);
  dodft(chdata[2],chspect[2]);
  dodft(chdata[3],chspect[3]);

  
    for(unsigned int i=0;i<SAMPLES;i++){
	spectrum1[i]=chspect[0][i];
	spectrum2[i]=chspect[1][i];
	spectrum3[i]=chspect[2][i];
	spectrum4[i]=chspect[3][i];
    }

    double psnrV;
    Scalar mssimV;

    for(unsigned int j=0;j<4;j++){
	for(unsigned int i=0;i<TRIGGERS;i++){ //Show the image captured in the window and repeat

	  psnrV = getPSNR(Mat(chdata[j]),Mat(trigdata[j][i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel "<<j<<" Trigger "<< i <<" data";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(Mat(chdata[j]), Mat(trigdata[j][i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	//spectrum
	  psnrV = getPSNR(Mat(chspect[j]),Mat(trigspect[j][i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel "<<j<<" Trigger "<< i << " spectrum ";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(Mat(chspect[j]), Mat(trigspect[j][i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	
	}

    }
  
  
  
}





double getPSNR(const Mat& I1, const Mat& I2)
{
    Mat s1;
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

Scalar getMSSIM( const Mat& i1, const Mat& i2)
{
 const double C1 = 6.5025, C2 = 58.5225;
 /***************************** INITS **********************************/
 int d     = CV_32F;

 Mat I1, I2;
 i1.convertTo(I1, d);           // cannot calculate on one byte large values
 i2.convertTo(I2, d);

 Mat I2_2   = I2.mul(I2);        // I2^2
 Mat I1_2   = I1.mul(I1);        // I1^2
 Mat I1_I2  = I1.mul(I2);        // I1 * I2

 /***********************PRELIMINARY COMPUTING ******************************/

 Mat mu1, mu2;   //
 GaussianBlur(I1, mu1, Size(11, 11), 1.5);
 GaussianBlur(I2, mu2, Size(11, 11), 1.5);

 Mat mu1_2   =   mu1.mul(mu1);
 Mat mu2_2   =   mu2.mul(mu2);
 Mat mu1_mu2 =   mu1.mul(mu2);

 Mat sigma1_2, sigma2_2, sigma12;

 GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
 sigma1_2 -= mu1_2;

 GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
 sigma2_2 -= mu2_2;

 GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
 sigma12 -= mu1_mu2;

 ///////////////////////////////// FORMULA ////////////////////////////////
 Mat t1, t2, t3;

 t1 = 2 * mu1_mu2 + C1;
 t2 = 2 * sigma12 + C2;
 t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

 t1 = mu1_2 + mu2_2 + C1;
 t2 = sigma1_2 + sigma2_2 + C2;
 t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

 Mat ssim_map;
 divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

 Scalar mssim = mean( ssim_map ); // mssim = average of ssim map
 return mssim;
}
