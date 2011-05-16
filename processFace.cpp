#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include <stdio.h>
#include <conio.h>		// For _kbhit()
#include <direct.h>		// For mkdir()
#include <vector>
#include <string>
#include <math.h>
#include "libface.h"

using namespace std;

#define PI 3.14159265

CvHaarClassifierCascade *cascade_f;
CvHaarClassifierCascade *cascade_le;
CvHaarClassifierCascade *cascade_re;
CvHaarClassifierCascade *cascade_n;
CvHaarClassifierCascade *cascade_m;
//CvMemStorage			*storage;

IplImage* detectFaceFeaturesAndProcessImage(IplImage *img);
IplImage* convertImageToGreyscale(const IplImage *imageSrc);
IplImage* SQI(IplImage* inp);

int normalFaceWidth = 92;	// Default dimensions for faces in the face recognition database. 
int normalFaceHeight = 112;	//	"

//int main(int argc, char** argv)
//{
//    /* usage: eyedetect <image> */
//    //assert(argc == 2);
//
//    /* load image */
//    IplImage *img = cvLoadImage("OriginalImages/jian/j2.jpg");
//	
//	//int i;
//	//int j=1;
//	//char cstr[256];
//	//for(i =1;i<10;i++) {
//	//	sprintf(cstr,"OriginalImages/song/s%d.jpg",i);
//	//	 IplImage *img = cvLoadImage(cstr);
//	//     IplImage* processedImage = detectFaceFeaturesAndProcessImage(img);
//	//	 if(processedImage!= NULL) {
//	//		 sprintf(cstr,"TestImages/s11/%d.pgm",j);
//	//		 cvSaveImage(cstr,processedImage);
//	//		 j++;
//	//	 }
//	//}
//
//
//
//    cvNamedWindow(argv[1], 1);
//
//    /* detect eyes and display image */
//    IplImage* processedImage = detectFaceFeaturesAndProcessImage(img);
//    cvShowImage(argv[1], processedImage);
//
//    cvWaitKey(0);
//    cvDestroyWindow(argv[1]);
//    cvReleaseImage(&img);
//
//    return 0;
//}

void displayImage(IplImage *img) {
 cvNamedWindow( "MyJPG", CV_WINDOW_AUTOSIZE );
 cvShowImage("MyJPG", img);
 cvWaitKey(0);
 cvReleaseImage( &img );
 cvDestroyWindow( "MyJPG" );
}

void getCorrectEyesPosition(CvSeq* leftEyes, CvSeq* rightEyes, CvRect face, CvRect* leftEye, CvRect* rightEye) {

	int midPointX = face.width/2;
	int i=0;

	int numLeftEyes = 0;
	int leftEyeSumX = 0;
	int leftEyeSumY = 0;
	int leftEyeAverageX = 0;
	int leftEyeAverageY = 0;

	int numrightEyes = 0;
	int rightEyeSumX = 0;
	int rightEyeSumY = 0;
	int rightEyeAverageX = 0;
	int rightEyeAverageY = 0;

	for( i = 0; i < (leftEyes ? leftEyes->total : 0); i++ ) {
		CvRect *eye = (CvRect*)cvGetSeqElem( leftEyes, i );
		if(eye->x + eye->width < midPointX) {
			numLeftEyes ++;
			leftEyeSumX +=eye->x + eye->width/2;
			leftEyeSumY +=eye->y + eye->height/2;
		}
		else if(eye->x > midPointX){
			numrightEyes ++;
			rightEyeSumX +=eye->x + eye->width/2;
			rightEyeSumY +=eye->y + eye->height/2;
		}
	}

	for( i = 0; i < (rightEyes ? rightEyes->total : 0); i++ ) {
		CvRect *eye = (CvRect*)cvGetSeqElem( rightEyes, i );
		if(eye->x + eye->width < midPointX) {
			numLeftEyes ++;
			leftEyeSumX +=eye->x + eye->width/2;
			leftEyeSumY +=eye->y + eye->height/2;
		}
		else if(eye->x > midPointX){
			numrightEyes ++;
			rightEyeSumX +=eye->x + eye->width/2;
			rightEyeSumY +=eye->y + eye->height/2;
		}
	}
	
	if(numLeftEyes !=0 && numrightEyes!=0) {
		leftEyeAverageX = leftEyeSumX/numLeftEyes;
		leftEyeAverageY = leftEyeSumY/numLeftEyes;

		rightEyeAverageX = rightEyeSumX/numrightEyes;
		rightEyeAverageY = rightEyeSumY/numrightEyes;

		leftEye->x = leftEyeAverageX;
		leftEye->y = leftEyeAverageY;

		rightEye->x = rightEyeAverageX;
		rightEye->y = rightEyeAverageY;
	}
}

double getRotationAngle(CvRect leftEye, CvRect rightEye, CvRect nose) 
{
	int distanceY = rightEye.y - leftEye.y;
	int distanceX = rightEye.x - leftEye.x;

	double angle;
	angle = atan2 ((double)distanceY,(double)distanceX) * 180 / PI;

	return angle;
}

IplImage *rotateImage(const IplImage *src, float angleDegrees)
{
	// Create a map_matrix, where the left 2x2 matrix
	// is the transform and the right 2x1 is the dimensions.
	float m[6];
	CvMat M = cvMat(2, 3, CV_32F, m);
	int w = src->width;
	int h = src->height;
	float angleRadians = angleDegrees * ((float)CV_PI / 180.0f);
	m[0] = (float)( cos(angleRadians) );
	m[1] = (float)( sin(angleRadians) );
	m[3] = -m[1];
	m[4] = m[0];
	m[2] = w*0.5f;  
	m[5] = h*0.5f;  

	// Make a spare image for the result
	CvSize sizeRotated;
	sizeRotated.width = cvRound(w);
	sizeRotated.height = cvRound(h);

	// Rotate
	IplImage *imageRotated = cvCreateImage( sizeRotated,
		src->depth, src->nChannels );

	// Transform the image
	cvGetQuadrangleSubPix( src, imageRotated, &M);

	return imageRotated;
}

IplImage* cropImage(const IplImage *img, const CvRect region)
{
	IplImage *imageCropped;
	CvSize size;

	if (img->width <= 0 || img->height <= 0
		|| region.width <= 0 || region.height <= 0) {
		//cerr << "ERROR in cropImage(): invalid dimensions." << endl;
		exit(1);
	}

	if (img->depth != IPL_DEPTH_8U) {
		//cerr << "ERROR in cropImage(): image depth is not 8." << endl;
		exit(1);
	}

	// Set the desired region of interest.
	cvSetImageROI((IplImage*)img, region);
	// Copy region of interest into a new iplImage and return it.
	size.width = region.width;
	size.height = region.height;
	imageCropped = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
	cvCopy(img, imageCropped);	// Copy just the region.

	return imageCropped;
}

IplImage* resizeImageWithAspectRatioOption(const IplImage *origImg, int newWidth,
	int newHeight, bool keepAspectRatio)
{
	IplImage *outImg = 0;
	int origWidth;
	int origHeight;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if (newWidth <= 0 || newHeight <= 0 || origImg == 0
		|| origWidth <= 0 || origHeight <= 0) {
		//cerr << "ERROR: Bad desired image size of " << newWidth
		//	<< "x" << newHeight << " in resizeImageWithAspectRatioOption().\n";
		exit(1);
	}

	if (keepAspectRatio) {
		// Resize the image without changing its aspect ratio,
		// by cropping off the edges and enlarging the middle section.
		CvRect r;
		// input aspect ratio
		float origAspect = (origWidth / (float)origHeight);
		// output aspect ratio
		float newAspect = (newWidth / (float)newHeight);
		// crop width to be origHeight * newAspect
		if (origAspect > newAspect) {
			int tw = (origHeight * newWidth) / newHeight;
			r = cvRect((origWidth - tw)/2, 0, tw, origHeight);
		}
		else {	// crop height to be origWidth / newAspect
			int th = (origWidth * newHeight) / newWidth;
			r = cvRect(0, (origHeight - th)/2, origWidth, th);
		}
		IplImage *croppedImg = cropImage(origImg, r);

		// Call this function again, with the new aspect ratio image.
		// Will do a scaled image resize with the correct aspect ratio.
		outImg = resizeImageWithAspectRatioOption(croppedImg, newWidth, newHeight, false);
		cvReleaseImage( &croppedImg );

	}
	else {

		// Scale the image to the new dimensions,
		// even if the aspect ratio will be changed.
		outImg = cvCreateImage(cvSize(newWidth, newHeight),
			origImg->depth, origImg->nChannels);
		if (newWidth > origImg->width && newHeight > origImg->height) {
			// Make the image larger
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_LINEAR: good at enlarging.
			// CV_INTER_CUBIC: good at enlarging.			
			cvResize(origImg, outImg, CV_INTER_LINEAR);
		}
		else {
			// Make the image smaller
			cvResetImageROI((IplImage*)origImg);
			// CV_INTER_AREA: good at shrinking (decimation) only.
			cvResize(origImg, outImg, CV_INTER_AREA);
		}

	}
	return outImg;
}

IplImage* detectFaceFeaturesAndProcessImage(IplImage *img)
{

	char *file1 = "haarcascade_frontalface_alt.xml";
    char *file2 = "RightEye.xml";
	char *file3 = "LeftEye.xml";
	char *file4 = "haarcascade_mcs_nose.xml";
	char *file5 = "Mouth.xml";


    /* load the face classifier */
	cascade_f = (CvHaarClassifierCascade*)cvLoad(file1, 0, 0, 0);

    /* load the eye classifier */
	cascade_re = (CvHaarClassifierCascade*)cvLoad(file2, 0, 0, 0);
    cascade_le = (CvHaarClassifierCascade*)cvLoad(file3, 0, 0, 0);

    cascade_n = (CvHaarClassifierCascade*)cvLoad(file4, 0, 0, 0);
    cascade_m = (CvHaarClassifierCascade*)cvLoad(file5, 0, 0, 0);
    /* setup memory storage, needed by the object detector */
    storage = cvCreateMemStorage(0);

    /* always check */
    assert(cascade_f && cascade_le && storage);

	int i;
	
	int D = 0;

	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face

    /* detect faces */
	CvSeq *faces = cvHaarDetectObjects(
		img, cascade_f, storage,
		1.1, 3, flags, cvSize( 40, 40 ) );

    /* return if not found */
    if (faces->total == 0) return NULL;

    /* draw a rectangle */
	CvRect *r = (CvRect*)cvGetSeqElem(faces, 0);
	CvRect faceRect = *r;
	if(D) {
		cvRectangle(img,
					cvPoint(r->x, r->y),
					cvPoint(r->x + r->width, r->y + r->height),
					CV_RGB(255, 0, 0), 1, 8, 0);
	}

    /* reset buffer for the next object detection */
    cvClearMemStorage(storage);

    /* Set the Region of Interest: estimate the eyes' position */
    cvSetImageROI(img, cvRect(r->x, r->y + (r->height/5.5), r->width, r->height/3.0));

    /* detect eyes */
	CvSeq* leftEyes = cvHaarDetectObjects( 
        img, cascade_le, storage,
		1.1, 3, 0, cvSize(18, 12));

    /* draw a rectangle for each eye found */
	for( i = 0; i < (leftEyes ? leftEyes->total : 0); i++ ) {
		r = (CvRect*)cvGetSeqElem( leftEyes, i );
		//cvRectangle(img, 
		//			cvPoint(r->x, r->y), 
		//			cvPoint(r->x + r->width, r->y + r->height),
		//			CV_RGB(0, 255, 0), 1, 8, 0);
	}

	CvSeq* rightEyes = cvHaarDetectObjects( 
    img, cascade_re, storage,
	1.1, 3, 0, cvSize(18, 12));

    /* draw a rectangle for each eye found */
	for( i = 0; i < (rightEyes ? rightEyes->total : 0); i++ ) {
		r = (CvRect*)cvGetSeqElem( rightEyes, i );
		//cvRectangle(img, 
		//			cvPoint(r->x, r->y), 
		//			cvPoint(r->x + r->width, r->y + r->height),
		//			CV_RGB(0, 255, 255), 1, 8, 0);
	}

	//Get Correct Eyes Position
	CvRect leftEye;
	CvRect rightEye;
	leftEye.x = -1;
	rightEye.x = -1;
	getCorrectEyesPosition(leftEyes,rightEyes,faceRect,&leftEye,&rightEye);
	
	if(!D && (leftEye.x == -1 || rightEye.x == -1)) {
		return NULL;
	}

	leftEye.width = 5;
	leftEye.height = 5;
	rightEye.width = 5;
	rightEye.height = 5;
	
	if(D) {
		cvRectangle(img, 
					cvPoint(leftEye.x - leftEye.width/2, leftEye.y - leftEye.height/2), 
					cvPoint(leftEye.x + leftEye.width/2, leftEye.y + leftEye.height/2),
					CV_RGB(255, 255, 255), 1, 8, 0);

		cvRectangle(img, 
					cvPoint(rightEye.x - rightEye.width/2, rightEye.y - rightEye.height/2), 
					cvPoint(rightEye.x + rightEye.width/2, rightEye.y + rightEye.height/2),
				CV_RGB(255, 255, 255), 1, 8, 0);
	}

    cvResetImageROI(img);


	//Get Nose position
	//Image divided into 16 squares. Interest of region is the middle 4 squares. 
	cvSetImageROI(img, cvRect(faceRect.x + faceRect.width/4, faceRect.y + faceRect.height/4, faceRect.width/2,faceRect.height/2));

	CvSeq* noses = cvHaarDetectObjects( 
    img, cascade_n, storage,
	1.1, 3, flags, cvSize(18,15));
	
	if (!D && noses->total == 0) return NULL;

	r = (CvRect*)cvGetSeqElem( noses,0);
	CvRect nose = *r;
	int noseWidth = 7;
	int noseHeight = 7;
	int noseMidX = r->x + r->width/2;
	int noseMidY = r->y + r->height/2;

	if(D) {
		cvRectangle(img, 
					cvPoint(noseMidX - noseWidth/2, noseMidY - noseHeight/2), 
					cvPoint(noseMidX + noseWidth/2, noseMidY + noseHeight/2),
					CV_RGB(0, 0, 255), 1, 8, 0);
	}
	/* draw a rectangle for each nose found */
	//for( i = 0; i < (noses ? noses->total : 0); i++ ) {
	//	r = (CvRect*)cvGetSeqElem( noses, i );
	//	cvRectangle(img, 
	//				cvPoint(r->x, r->y), 
	//				cvPoint(r->x + r->width, r->y + r->height),
	//				CV_RGB(0, 0, 255), 1, 8, 0);
	//}

	cvResetImageROI(img);

	double angle = getRotationAngle(leftEye,rightEye,nose);

	//Get Mouth
	//cvSetImageROI(img, cvRect(faceRect.x + faceRect.width/4, faceRect.y + faceRect.height/2, faceRect.width/2,faceRect.height/2));

	//CvSeq* mouths = cvHaarDetectObjects( 
 //   img, cascade_m, storage,
	//1.1, 3, flags, cvSize(25,15));

	//r = (CvRect*)cvGetSeqElem( mouths,0);

	//int mouthHeight = 5;
	//int mouthMidY = r->y + r->height/2;

	//if(D) {
	//	cvRectangle(img, 
	//			cvPoint(r->x, mouthMidY - mouthHeight/2), 
	//			cvPoint(r->x + r->width, mouthMidY + mouthHeight/2),
	//			CV_RGB(0, 255,0), 1, 8, 0);
	//}
	/* draw a rectangle for each eye found */
	//for( i = 0; i < (mouths ? mouths->total : 0); i++ ) {
	//	r = (CvRect*)cvGetSeqElem( mouths, i );
	//	cvRectangle(img, 
	//				cvPoint(r->x, r->y), 
	//				cvPoint(r->x + r->width, r->y + r->height),
	//				CV_RGB(0, 255,255), 1, 8, 0);
	//}

	cvResetImageROI(img);

	int absoluteNoseX = noseMidX + faceRect.width/4 + faceRect.x;
	int absoluteNoseY = noseMidY + faceRect.height/4 + faceRect.y;
	
	int normalizedFaceRectWidth = faceRect.width;

	if(absoluteNoseX - faceRect.width/2 < 0) {
		normalizedFaceRectWidth = absoluteNoseX *2;
	} 

	if(absoluteNoseX + faceRect.width/2 > img->width) {
		normalizedFaceRectWidth = (img->width - absoluteNoseX) * 2;
	}

	CvRect noseCenteredRect = cvRect(absoluteNoseX - normalizedFaceRectWidth/2,absoluteNoseY - faceRect.height/2, normalizedFaceRectWidth,faceRect.height);

	IplImage *faceImg = cropImage(img, noseCenteredRect);	// Get the detected face image.
	IplImage* rotatedImage = rotateImage(faceImg,-angle);
	IplImage *resizedImage = resizeImageWithAspectRatioOption(rotatedImage, normalFaceWidth, normalFaceHeight, true);
	//IplImage *sqiImage = SQI(resizedImage);
	//IplImage *sqiImage = convertImageToGreyscale(resizedImage);
	IplImage *greyImg = convertImageToGreyscale(resizedImage);
	IplImage *equalizedImg = cvCreateImage(cvGetSize(greyImg), 8, 1);	// Create an empty greyscale image
	cvEqualizeHist(greyImg, equalizedImg);
	IplImage* sqiImage = equalizedImg;

	if(D) {
		displayImage(equalizedImg);
	}

	return equalizedImg;
}

IplImage* convertImageToGreyscale(const IplImage *imageSrc)
{
	IplImage *imageGrey;
	// Either convert the image to greyscale, or make a copy of the existing greyscale image.
	// This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
	if (imageSrc->nChannels == 3) {
		imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
		cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
	}
	else {
		imageGrey = cvCloneImage(imageSrc);
	}
	return imageGrey;
}


/*!

   This function computes SQI(self quotient image) of an input image.
   Input:
    IplImage* - input image
   Output:
    IplImage* - the SQI of the input image.
   
*/

IplImage* SQI(IplImage* inp)
{
  int num_filters;
  int size[3];
	IplImage *ttt;
  CvMat *filtered_image[3];
  CvMat *qi[3];
  CvMat *inp_mat;
  CvMat *res;
  IplImage *res_img;
	int i,j,k;
	double  tmp1,tmp2,tmp3;
	CvMat *g_ker;
	
  num_filters=3;

  size[0]=3;
  size[1]=9;
  size[2]=15;
	
 
  
  
  inp_mat=IplImage2Mat(inp);
  
// cvNamedWindow("ttt",1);
 
 //cvShowImage("ttt",inp);
 //cvWaitKey(0);
 
  for(i=0;i<num_filters;i++)
  {
	  g_ker=Gaussian(size[i]);
	 filtered_image[i]=Conv_Weighted_Gaussian(inp,g_ker);
	
	/*  ttt=Mat2IplImage(filtered_image[i],0);
	  cvShowImage("ttt",ttt);
	  cvWaitKey(0);
	  cvReleaseImage(&ttt);
*/
	  cvReleaseMat(&g_ker);
	  qi[i]=cvCreateMat(inp_mat->rows,inp_mat->cols,CV_64FC1);
	  for(j=0;j<inp_mat->rows;j++)
	  {
		  for(k=0;k<inp_mat->cols;k++)
		  {
			  tmp1=cvmGet(inp_mat,j,k);
			  tmp2=cvmGet(filtered_image[i],j,k);
			  
			 //  if(tmp1==0.0 || tmp2==0.0 )
			   //{
				 //  tmp3=0.0;
				   //}
				  // else{
			  tmp3=log10((tmp1+1.0)/(tmp2+1.0));
			   //}
			 // printf("%g *",tmp3);
			  cvmSet(qi[i],j,k,tmp3);
			  
		  }
	  }
	  cvReleaseMat(&filtered_image[i]);
  }
  
  res=cvCreateMat(inp_mat->rows,inp_mat->cols,CV_64FC1);
  cvSetZero(res);
  for(i=0;i<num_filters;i++)
  {
	  for(j=0;j<inp_mat->rows;j++)
	  {
		  for(k=0;k<inp_mat->cols;k++)
		  {
			  tmp1=cvmGet(qi[i],j,k);
			  tmp2=cvmGet(res,j,k);
	#ifdef DEBUG
		//	 printf("%g * ",tmp1+tmp2);
	#endif
			  cvmSet(res,j,k,tmp1+tmp2);
		  }
	  }
	  cvReleaseMat(&qi[i]);
	  
  }
  
  Scale_Mat(res,255);
  res_img=Mat2IplImage(res,0);
  
  
  cvReleaseMat(&res);
  
  return res_img;
}
 

