#ifndef PROCESSFACE_H
#define PROCESSFACE_H

IplImage* detectFaceFeaturesAndProcessImage(IplImage *img); // function prototype for add.h
IplImage* convertImageToGreyscale(const IplImage *imageSrc);
IplImage* cropImage(const IplImage *img, const CvRect region);
IplImage* resizeImageWithAspectRatioOption(const IplImage *origImg, int newWidth,int newHeight, bool keepAspectRatio);

#endif