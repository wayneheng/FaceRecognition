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
 Function to convert color image to a gray scale image
 
 Input:
  IplImage* - input color imageData
 Output:
   IplImage* - the gray scale image
*/

IplImage* Rgb2Gray(IplImage *src)
{
	IplImage *result;
int i,j;

	int step_src,step_res;

	result=cvCreateImage(cvSize(src->width,src->height),src->depth,1);


	unsigned char *src_data;
	unsigned char *res_data;

	src_data=(unsigned char*)src->imageData;
	res_data=(unsigned char*)result->imageData;

	step_src=src->widthStep;
	step_res=result->widthStep;

	for(i=0;i<src->height;i=i+1)
	{
		for(j=0;j<(src->width*src->nChannels);j=j+src->nChannels)
		{
			  res_data[j/src->nChannels]=(unsigned char)((0.3*(double)src_data[j+2])+(0.59*(double)src_data[j+1])+(0.11*(double)src_data[j]));
             //res_data[j+2]=src_data[j+2]; BGR format gray
		}
		src_data+=step_src;
		res_data+=step_res;
	}


	return result;
}
