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
    This function is used to compute weighted gaussian kernel matrix
	
	Input:
	  CvMat* - A small portion of the image where weighted gaussian needs to be applied.
	           The size of this portion should be same as the Gaussian filter.
	  CvMat* - The gaussian filter.
	Output:
	  CvMat* - The weighted gaussian kernel filter.
*/

CvMat* Weighted_Gaussian(CvMat *inp,CvMat *gaussian)
{
	double sum;
	double threshold;
	int i,j;
	double tmp1,tmp2;
	double scl_factor;
	double lt_cnt;
	double gt_cnt;
	bool mr_t_thr;
	
	CvMat* w_gauss=cvCreateMat(gaussian->rows,gaussian->cols,CV_64FC1);
	cvSetZero(w_gauss);
	sum=0.0;
	for(i=0;i<inp->rows;i++)
	{
		for(j=0;j<inp->cols;j++)
		{
			sum=sum+cvmGet(inp,i,j);
		}
	}
	
	threshold=sum/(inp->cols*inp->rows);
	lt_cnt=0;
	gt_cnt=0;
	for(i=0;i<inp->rows;i++)
	{
		for(j=0;j<inp->cols;j++)
		{
			tmp1=cvmGet(inp,i,j);
			if(tmp1>threshold)
			{
			  gt_cnt=gt_cnt+1;	
			}
			else
			{
				lt_cnt=lt_cnt+1;
			}
		}
	}
	
	if(gt_cnt>lt_cnt)
	{
	   mr_t_thr=true;
	}
	else
	{
		mr_t_thr=false;
	}
	
	
	
	scl_factor=0.0;
	for(i=0;i<inp->rows;i++)
	{
		for(j=0;j<inp->cols;j++)
		{
			tmp1=cvmGet(inp,i,j);
			if(((tmp1>threshold)&& mr_t_thr==false) ||((tmp1<threshold)&& mr_t_thr==true))
			{
				
				cvmSet(w_gauss,i,j,0.0);
			}
			else
			{
				tmp2=cvmGet(gaussian,i,j);
				scl_factor=scl_factor+tmp2;
				cvmSet(w_gauss,i,j,tmp2);
			}
		}
	}
	
	
	/*Normalizing the weighted gaussian matrix*/
	
	for(i=0;i<inp->rows;i++)
	{
		for(j=0;j<inp->cols;j++)
		{
			tmp1=cvmGet(w_gauss,i,j);
			tmp2=tmp1/scl_factor;
			cvmSet(w_gauss,i,j,tmp2);
		}
	}
		
	return w_gauss;
}
