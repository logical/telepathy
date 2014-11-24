
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

#include "cvanalyze.h"

#define TRIGGERVALUE 5

using namespace std;
using namespace cv;

typedef Matx<float, 4, SAMPLES> Matxdata;
typedef Matx<float, 1, SAMPLES> Matxspect;

Matxdata trigch1data[10];
Matxdata trigch2data[10];
Matxspect trigch1spect[10];
Matxspect trigch2spect[10];

void dodft(Matxdata in,Matxspect &out);
double getPSNR ( const Mat& I1, const Mat& I2);
Scalar getMSSIM( const Mat& I1, const Mat& I2);





void getpatternimage(unsigned short index){

  for(int i=0;i<4;i++){
    for(int j=0;j<SAMPLES;j++){
      trigch1data[index](i,j)=triggers[index].channel1[i][j];
      trigch2data[index](i,j)=triggers[index].channel2[i][j];
    }
  } 
  dodft(trigch1data[index],trigch1spect[index]); 
  dodft(trigch2data[index],trigch2spect[index]); 
  
}

void dodft(Matxdata in,Matxspect &out){
  


//     Mat padded;                            //expand input image to optimal size
//    int m = getOptimalDFTSize( in.rows );
//    int n = getOptimalDFTSize( in.cols ); // on the border add zero values
//    copyMakeBorder(in, padded, 0, m - in.rows, 0, n - in.cols, BORDER_CONSTANT, Scalar::all(0));
    Mat planes[] = {Mat_<float>(Mat(in)), Mat::zeros(Mat(in).size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
    dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];

    resize(magI,magI,Size(4*magI.cols,4*magI.rows),4,4,INTER_LINEAR);
    
    magI(Rect(8, 0, out.cols, out.rows)).copyTo(out);
//    normalize(out, out, 0, 4096, NORM_L1);
    
    
  
}

void compare_signals(void){
  
//4 images per pattern one for each channel of the signal and the fft of the signal
//THIS MIGHT CHANGE

  Matxdata ch1data,ch2data;
  Matxspect ch1spect,ch2spect;

 for(unsigned int i=0;i<4;i++){
    for(unsigned int j=0;j<SAMPLES;j++){
      ch1data(i,j)=channel1[i][j];
      ch2data(i,j)=channel2[i][j];
    }
  }
  dodft(ch1data,ch1spect);
  dodft(ch2data,ch2spect);

  for(unsigned int i=0;i<SAMPLES;i++){
      spectrum1[i]=ch1spect(0,i);
      spectrum2[i]=ch2spect(0,i);
  }

  
/*  
    int psnrTriggerValue;
    cout << "PSNR trigger value " << setiosflags(ios::fixed) << setprecision(3)
         << psnrTriggerValue << endl;
*/

    double psnrV;
    Scalar mssimV;

    for(unsigned int i=0;i<TRIGGERS;i++){ //Show the image captured in the window and repeat
/*
	  if (liveimages[j].empty() || patterns[i*4+j].empty())
	  {
	      cout << " < < <  Game over!  > > > ";
	      break;
	  }
*/

	  ///////////////////////////////// PSNR ////////////////////////////////////////////////////
	  psnrV = getPSNR(Mat(ch1data),Mat(trigch1data[i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel 1 data ";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(Mat(ch1data), Mat(trigch1data[i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	  psnrV = getPSNR(Mat(ch2data),Mat(trigch2data[i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel 2 data ";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(Mat(ch1data), Mat(trigch1data[i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	  psnrV = getPSNR(Mat(ch1spect),Mat(trigch1spect[i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel 1 spectrum ";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(Mat(ch1data), Mat(trigch1data[i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	  psnrV = getPSNR(Mat(ch2spect),Mat(trigch2spect[i]));
	  if(abs(psnrV) < TRIGGERVALUE ){
	    cout <<triggers[i].name<< " Channel 2 spectrum ";
	    cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB"<<endl;
	    mssimV = getMSSIM(Mat(ch1data), Mat(trigch1data[i]));
	    cout << " MSSIM: " <<  setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%"<<endl;
	  }
	  
	  
	  //////////////////////////////////// MSSIM /////////////////////////////////////////////////
    

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
