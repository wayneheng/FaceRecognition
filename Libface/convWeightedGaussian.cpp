
#include "libface.h"

CvMat* Conv_Weighted_Gaussian(IplImage *inp_img,CvMat *kernel)
 {
         //IplImage *result;
         int i,j;
         CvPoint start;
         CvMat *tmp;
         CvMat *ddd;
         CvMat *w_gauss;
         //result=cvCloneImage(inp_img);
 
         double val;
 
         ddd=cvCreateMat(inp_img->height,inp_img->width,CV_64FC1);
 
         for(i=0;i<inp_img->height;i++)
         {
                 for(j=0;j<inp_img->width;j++)
                 {
                         start.x=j-(kernel->cols/2);
                         start.y=i-(kernel->rows/2);
 
                         tmp=Get_Mat(start,kernel->cols,kernel->rows,inp_img);
                         
                         w_gauss=Weighted_Gaussian(tmp,kernel);
                         
                         val=cvDotProduct(w_gauss,tmp);
                         
                         
                         
                         cvmSet(ddd,i,j,val);
 
                         cvReleaseMat(&tmp);
                         cvReleaseMat(&w_gauss);
                         
                 }
         }


         /*
         cvNamedWindow("fdf",1);
        cvShowImage("fdf",ddd);
         cvWaitKey(0);
     */

        return ddd;

}