/*
    ** Copyright (C) 2006 Sujith K.R. All rights reserved.
    ** Email: sujith.k.raman@gmail.com,sujithkr@au-kbc.org
    
    ** Redistribution and use in source and binary forms, with or without
    ** modification, are permitted provided that the following conditions
    ** are met:
    ** 1. Redistributions of source code must retain the above copyright
    **    notice, this list of conditions and the following disclaimer.
    ** 2. Redistributions in binary form must reproduce the above copyright
    **    notice, this list of conditions and the following disclaimer in the
    **    documentation and/or other materials provided with the distribution.
    ** 3. The name of the author may not be used to endorse or promote products
    **    derived from this software without specific prior written permission.
    **
    ** THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
    ** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    ** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ** ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
    ** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    ** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    ** OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    ** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    ** LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    ** OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    ** SUCH DAMAGE. */


#include "libface.h"

/*!
     This function is used to extract a small region within an image.
	 
	 Input:
	  CvPoint - Specifying the starting point.
	  int- width, 
	  int - height,
	  IplImage* - The input image.
	 Output:
	  CvMat* - The extrated portion.
*/

CvMat* Get_Mat(CvPoint a,int width,int height,IplImage *image)
{
 
  CvMat *fea_ar;

  unsigned char t_val;
	int h_i,w_i;
  

  fea_ar=cvCreateMat(height,width,CV_64FC1);
 

  cvSetZero(fea_ar);

 
  int i,j;

  
  

  for(i=a.y;i<(a.y+height);i++)
  {
	  for(j=a.x;j<(a.x+width);j++)
	  {
		 if((i>=0)&&(j>=0)&&(i<(image->height))&&(j<(image->width)))
		  {
			  t_val=(unsigned char)image->imageData[(i*image->widthStep)+j];
			  cvmSet(fea_ar,i-a.y,j-a.x,(double)t_val);
		  }
		  else{
			  if(j<0)
			  {
				  w_i=image->width+j;
			  }
			  else if(j>=image->width)
			  {
				  w_i=j-image->width;
			  }
			  else
			  {
				  w_i=j;
			  }
			  
			  if(i<0)
			  {
				  h_i=-i;
			  }
			  else if(i>=image->height)
			  {
				  h_i=image->height-(i-image->height);
			  }
			  else
			  {
				  h_i=i;
			  }
			  
			  t_val=(unsigned char)image->imageData[(h_i*image->widthStep)+w_i];
			  cvmSet(fea_ar,i-a.y,j-a.x,(double)t_val);
		  }
	  }

  }
    
  return (fea_ar);
}
