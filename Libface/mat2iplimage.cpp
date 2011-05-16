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
   This function is used to convert a matrix into a image.
   
   Input:
     CvMat* - the input matrix that needs to be converted.
	 int - This is used to specify the type of output image(eg:IPL_DEPTH_32F or
	       IPL_DEPTH_8U)
   Output:
     IplImage* - The converted image.
*/

IplImage* Mat2IplImage(CvMat *inp_mat,int type)
{

	IplImage *result;
	int i,j;
	double tmp_val;

	if(type==0)
	{
	    result=cvCreateImage(cvSize(inp_mat->cols,inp_mat->rows),IPL_DEPTH_8U,1);
	}
	else if(type==1)
	{
		result=cvCreateImage(cvSize(inp_mat->cols,inp_mat->rows),IPL_DEPTH_32F,1);
	}
	else
	{
		return 0;
	}

	for(i=0;i<result->height;i++)
	{
		for(j=0;j<result->width;j++)
		{
			tmp_val=cvmGet(inp_mat,i,j);
			result->imageData[(i*result->widthStep)+j]=(unsigned char) tmp_val;
		}
	}

	return result;
}
