
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

#include "cvanalyze.h"


using namespace std;
using namespace cv;


Mat patterns[TRIGGERS*4];

double getPSNR ( const Mat& I1, const Mat& I2);
Scalar getMSSIM( const Mat& I1, const Mat& I2);



void imagesfromdata(unsigned short *data,unsigned short length,Mat image){
  Point p1,p2;
  p1.x=0;p1.y=data[0];
  for(int i=0;i<length;i++){
    p2.y=data[i];
    line(image,p1, p2,Scalar(255,255,255),1,8);
    p2.x++;
  }
}

void getpatternimage(unsigned short index){
//	the images will be
//  plot1 / fft1 / plot2 / fft2
  
  unsigned short data[SAMPLES*4],spectrum[SAMPLES],**dataptr;
  dataptr=(unsigned short**)malloc( 4* sizeof(unsigned short*) ) ;
  //flatten the arrays
  
  for(unsigned int i=0;i<4;i++){
    for(unsigned int j=0;j<SAMPLES;j++){
      data[i*SAMPLES+j]=triggers[index].channel1[i][j];
    }
  }
  patterns[index*4]= Mat::zeros( SAMPLES*4,MAXSAMPLE, CV_8UC1);
  imagesfromdata(data,SAMPLES*4, patterns[index*4]);

  for(unsigned int i=0;i<4;i++){
    dataptr[i]=triggers[index].channel1[i];
  }
  frequencies(dataptr,spectrum); 
  patterns[index*4+1]=  Mat::zeros( SAMPLES,MAXSAMPLE, CV_8UC1);
  imagesfromdata(spectrum,SAMPLES, patterns[index*4+1]);

  for(unsigned int i=0;i<4;i++){
    for(unsigned int j=0;j<SAMPLES;j++){
      data[i*SAMPLES+j]=triggers[index].channel2[i][j];
    }
  }
  patterns[index*4+2]= Mat::zeros( SAMPLES*4,MAXSAMPLE, CV_8UC1);
  imagesfromdata(data,SAMPLES*4, patterns[index*4+2]);

  for(unsigned int i=0;i<4;i++){
    dataptr[i]=triggers[index].channel2[i];
  }
  frequencies(dataptr,spectrum); 
  patterns[index*4+3]=  Mat::zeros( SAMPLES,MAXSAMPLE, CV_8UC1);
  imagesfromdata(spectrum,SAMPLES, patterns[index*4+3]);
  free(dataptr);
  
}




void compare_signals(void){
  
//for matching purposes I'm making the resolution larg with no color 
//4 images per pattern one for each channel of the signal and the fft of the signal
//the height is MAXSAMPLE and the resolution will be 4*SAMPLES
//THIS MIGHT CHANGE
  unsigned short data[SAMPLES*4];
  Mat liveimages[4];
  for(unsigned int i=0;i<4;i++){
    for(unsigned int j=0;j<SAMPLES;j++){
      data[i*SAMPLES+j]=channel1[i][j];
    }
  }
  liveimages[0]=Mat::zeros( SAMPLES*4,MAXSAMPLE, CV_8UC1);
  imagesfromdata(data,SAMPLES*4, liveimages[0]);

  liveimages[1]=Mat::zeros( SAMPLES,MAXSAMPLE, CV_8UC1);
  imagesfromdata(spectrum1,SAMPLES, liveimages[1]);

  for(unsigned int i=0;i<4;i++){
    for(unsigned int j=0;j<SAMPLES;j++){
      data[i*SAMPLES+j]=channel2[i][j];
    }
  }
  liveimages[2]=Mat::zeros( SAMPLES*4,MAXSAMPLE, CV_8UC1);
  imagesfromdata(data,SAMPLES*4, liveimages[2]);

  liveimages[3]=Mat::zeros( SAMPLES,MAXSAMPLE, CV_8UC1);
  imagesfromdata(spectrum2,SAMPLES, liveimages[3]);
  
/*  
    int psnrTriggerValue;
    cout << "PSNR trigger value " << setiosflags(ios::fixed) << setprecision(3)
         << psnrTriggerValue << endl;
*/

    double psnrV;
//    Scalar mssimV;

    for(unsigned int i=0;i<TRIGGERS;i++){ //Show the image captured in the window and repeat
      for(unsigned int j=0;j<4;j++){ //Show the image captured in the window and repeat

	  if (liveimages[j].empty() || patterns[i*4+j].empty())
	  {
	      cout << " < < <  Game over!  > > > ";
	      break;
	  }


	  ///////////////////////////////// PSNR ////////////////////////////////////////////////////
	  psnrV = getPSNR(liveimages[j],patterns[i*4+j]);
	  if(psnrV<5){
	      cout << "Signal: " << i ;
	      switch(j){
		case 0:
		  cout <<" Channel: 1 time      ";
		  break;
		case 1:  
		  cout <<" Channel: 1 spectrum  ";
		  break;
		case 2:
 		  cout <<" Channel: 2 time      ";
		  break;
		case 3:
  		  cout <<" Channel: 2 spectrum  ";
		  break;
	      }		
		
	      cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB";
	      cout << endl;
	  }
    /*
	  //////////////////////////////////// MSSIM /////////////////////////////////////////////////
	  if (psnrV < psnrTriggerValue && psnrV)
	  {
	      mssimV = getMSSIM(frameReference, frameUnderTest);

	      cout << " MSSIM: "
		  << " R " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[2] * 100 << "%"
		  << " G " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[1] * 100 << "%"
		  << " B " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%";
	  }
    */

      }
    }
  
  
  
  
  
}



/*

{
     for(;;) //Show the image captured in the window and repeat
    {
        captRefrnc >> frameReference;
        captUndTst >> frameUnderTest;

        if (frameReference.empty() || frameUnderTest.empty())
        {
            cout << " < < <  Game over!  > > > ";
            break;
        }

        ++frameNum;
        cout << "Frame: " << frameNum << "# ";

        ///////////////////////////////// PSNR ////////////////////////////////////////////////////
        psnrV = getPSNR(frameReference,frameUnderTest);
        cout << setiosflags(ios::fixed) << setprecision(3) << psnrV << "dB";

        //////////////////////////////////// MSSIM /////////////////////////////////////////////////
        if (psnrV < psnrTriggerValue && psnrV)
        {
            mssimV = getMSSIM(frameReference, frameUnderTest);

            cout << " MSSIM: "
                << " R " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[2] * 100 << "%"
                << " G " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[1] * 100 << "%"
                << " B " << setiosflags(ios::fixed) << setprecision(2) << mssimV.val[0] * 100 << "%";
        }

        cout << endl;

        ////////////////////////////////// Show Image /////////////////////////////////////////////
        imshow(WIN_RF, frameReference);
        imshow(WIN_UT, frameUnderTest);

        c = (char)cvWaitKey(delay);
        if (c == 27) break;
    } 
  
  
  
}
*/





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
