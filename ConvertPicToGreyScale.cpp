//#include <stdio.h>
//#include <conio.h>		// For _kbhit()
//#include <direct.h>		// For mkdir()
//#include <vector>
//#include <string>
////#include <string.h>
//#include "cv.h"
//#include "cvaux.h"
//#include "highgui.h"
//
//using namespace std;
//
//const char *faceCascadeFilename = "haarcascade_frontalface_alt.xml";
//int faceWidth = 120;	// Default dimensions for faces in the face recognition database. Added by Shervin.
//int faceHeight = 90;	//	"
//
//IplImage* convertImageToGreyscale(const IplImage *imageSrc)
//{
//	IplImage *imageGrey;
//	// Either convert the image to greyscale, or make a copy of the existing greyscale image.
//	// This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
//	if (imageSrc->nChannels == 3) {
//		imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
//		cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
//	}
//	else {
//		imageGrey = cvCloneImage(imageSrc);
//	}
//	return imageGrey;
//}
//IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight)
//{
//	IplImage *outImg = 0;
//	int origWidth;
//	int origHeight;
//	if (origImg) {
//		origWidth = origImg->width;
//		origHeight = origImg->height;
//	}
//	if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
//		printf("ERROR in resizeImage: Bad desired image size of %dx%d\n.", newWidth, newHeight);
//		exit(1);
//	}
//
//	// Scale the image to the new dimensions, even if the aspect ratio will be changed.
//	outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
//	if (newWidth > origImg->width && newHeight > origImg->height) {
//		// Make the image larger
//		cvResetImageROI((IplImage*)origImg);
//		cvResize(origImg, outImg, CV_INTER_LINEAR);	// CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
//	}
//	else {
//		// Make the image smaller
//		cvResetImageROI((IplImage*)origImg);
//		cvResize(origImg, outImg, CV_INTER_AREA);	// CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
//	}
//
//	return outImg;
//}
//
//IplImage* cropImage(const IplImage *img, const CvRect region)
//{
//	IplImage *imageTmp;
//	IplImage *imageRGB;
//	CvSize size;
//	size.height = img->height;
//	size.width = img->width;
//
//	if (img->depth != IPL_DEPTH_8U) {
//		printf("ERROR in cropImage: Unknown image depth of %d given in cropImage() instead of 8 bits per pixel.\n", img->depth);
//		exit(1);
//	}
//
//	// First create a new (color or greyscale) IPL Image and copy contents of img into it.
//	imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
//	cvCopy(img, imageTmp, NULL);
//
//	// Create a new image of the detected region
//	// Set region of interest to that surrounding the face
//	cvSetImageROI(imageTmp, region);
//	// Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
//	size.width = region.width;
//	size.height = region.height;
//	imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
//	cvCopy(imageTmp, imageRGB, NULL);	// Copy just the region.
//
//    cvReleaseImage( &imageTmp );
//	return imageRGB;		
//}
//
//// Get an 8-bit equivalent of the 32-bit Float image.
//// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
//IplImage* convertFloatImageToUcharImage(const IplImage *srcImg)
//{
//	IplImage *dstImg = 0;
//	if ((srcImg) && (srcImg->width > 0 && srcImg->height > 0)) {
//
//		// Spread the 32bit floating point pixels to fit within 8bit pixel range.
//		double minVal, maxVal;
//		cvMinMaxLoc(srcImg, &minVal, &maxVal);
//
//		//cout << "FloatImage:(minV=" << minVal << ", maxV=" << maxVal << ")." << endl;
//
//		// Deal with NaN and extreme values, since the DFT seems to give some NaN results.
//		if (cvIsNaN(minVal) || minVal < -1e30)
//			minVal = -1e30;
//		if (cvIsNaN(maxVal) || maxVal > 1e30)
//			maxVal = 1e30;
//		if (maxVal-minVal == 0.0f)
//			maxVal = minVal + 0.001;	// remove potential divide by zero errors.
//
//		// Convert the format
//		dstImg = cvCreateImage(cvSize(srcImg->width, srcImg->height), 8, 1);
//		cvConvertScale(srcImg, dstImg, 255.0 / (maxVal - minVal), - minVal * 255.0 / (maxVal-minVal));
//	}
//	return dstImg;
//}
//
////void convertImageToIplImage() {
//// IplImage *greyImg = convertImageToGreyscale(img);
////}
//
//void displayImage(IplImage *img) {
// cvNamedWindow( "MyJPG", CV_WINDOW_AUTOSIZE );
// cvShowImage("MyJPG", img);
// cvWaitKey(0);
// cvReleaseImage( &img );
// cvDestroyWindow( "MyJPG" );
//}
//
//CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade )
//{
//	const CvSize minFeatureSize = cvSize(20, 20);
//	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face.
//	const float search_scale_factor = 1.1f;
//	IplImage *detectImg;
//	IplImage *greyImg = 0;
//	CvMemStorage* storage;
//	CvRect rc;
//	double t;
//	CvSeq* rects;
//	int i;
//
//	storage = cvCreateMemStorage(0);
//	cvClearMemStorage( storage );
//
//	// If the image is color, use a greyscale copy of the image.
//	detectImg = (IplImage*)inputImg;	// Assume the input image is to be used.
//	if (inputImg->nChannels > 1) 
//	{
//		greyImg = cvCreateImage(cvSize(inputImg->width, inputImg->height), IPL_DEPTH_8U, 1 );
//		cvCvtColor( inputImg, greyImg, CV_BGR2GRAY );
//		detectImg = greyImg;	// Use the greyscale version as the input.
//	}
//
//	// Detect all the faces.
//	t = (double)cvGetTickCount();
//	rects = cvHaarDetectObjects( detectImg, (CvHaarClassifierCascade*)cascade, storage,
//				search_scale_factor, 3, flags, minFeatureSize );
//	t = (double)cvGetTickCount() - t;
//	printf("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );
//
//	// Get the first detected face (the biggest).
//	if (rects->total > 0) {
//        rc = *(CvRect*)cvGetSeqElem( rects, 0 );
//    }
//	else
//		rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.
//
//	//cvReleaseHaarClassifierCascade( &cascade );
//	//cvReleaseImage( &detectImg );
//	if (greyImg)
//		cvReleaseImage( &greyImg );
//	cvReleaseMemStorage( &storage );
//
//	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
//}
//
//IplImage* processImage() {
//
//		IplImage *camImg;
//		IplImage *greyImg;
//		IplImage *faceImg;
//		IplImage *sizedImg;
//		IplImage *equalizedImg;
//		IplImage *processedFaceImg;
//		CvRect faceRect;
//
//		CvHaarClassifierCascade* faceCascade;
//		faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0 );
//		if( !faceCascade ) {
//			printf("ERROR in recognizeFromCam(): Could not load Haar cascade Face detection classifier in '%s'.\n", faceCascadeFilename);
//			exit(1);
//		}
//		
//	
//		IplImage* img = cvLoadImage( "test2.jpg" );
//		greyImg = convertImageToGreyscale(img);
//		faceRect = detectFaceInImage(greyImg, faceCascade );
//		// Make sure a valid face was detected.
//		if (faceRect.width > 0) {
//			faceImg = cropImage(greyImg, faceRect);	// Get the detected face image.
//			// Make sure the image is the same dimensions as the training images.
//			sizedImg = resizeImage(faceImg, faceWidth, faceHeight);
//			// Give the image a standard brightness and contrast, in case it was too dark or low contrast.
//			equalizedImg = cvCreateImage(cvGetSize(sizedImg), 8, 1);	// Create an empty greyscale image
//			cvEqualizeHist(sizedImg, equalizedImg);
//			processedFaceImg = equalizedImg;
//		}
//
//		return processedFaceImg;
//}
//
//int main( int argc, char** argv )
//{
//	//convertImageToIplImage();
//	IplImage *processedImage = processImage();
//	displayImage(processedImage);
//	return 1;
//}