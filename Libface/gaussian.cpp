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
  This function returns a gaussian kernel matrix of prescribed  size.
  
  Input:
      int -- The size of the gaussian kernel
  Output:
      CvMat* -- The Gaussian kernel matrix

*/

CvMat* Gaussian(int size)
{
    CvMat *res;
	int i,j;
	int x,y;
	double tmp;
	double sigma;
	int halfsize;
	
	sigma=(double)size/5;
	res=cvCreateMat(size,size,CV_64FC1);
	halfsize=size/2;
	
	for(i=0;i<res->rows;i++)
	{
	   for(j=0;j<res->cols;j++)
	   {
	       x=j-halfsize;
		   y=i-halfsize;
		   tmp=exp(-(double)(x*x+y*y)/sigma);
		   cvmSet(res,i,j,tmp);
       }
    }
	
	return res;
}
