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
    This function is used to scale a input matrix
	Input:
	     CvMat* - The input matrix, that needs to be scaled(the same matrix is 
		              scaled)
		 double - The maximum range(eg:255)
		 
   Return :
         int - 1 is returned when everything works fine.
		 
*/


int Scale_Mat(CvMat *input,double scale)
{
    double tmp;
	double val;
	double min;
	double max;
	min=20000.0;
	max=-20000.0;
	int i,j;
for(i=0;i<input->rows;i++)
{
for(j=0;j<input->cols;j++)
{
  tmp=cvmGet(input,i,j);
	//if(tmp==-INF)
	//	printf("%d--%d\n",i,j);
	if(tmp<min)
	  min=tmp;
	if(tmp>max)
		max=tmp;
}
}

//printf("%g - %g\n",min,max);

for(i=0;i<input->rows;i++)
{
for(j=0;j<input->cols;j++)
{
  tmp=cvmGet(input,i,j);
  val=scale*((tmp-min)/(max-min));
//	printf("%g * ",val);
	cvmSet(input,i,j,val);
}
}

/*max=0.0;
for(i=0;i<input->rows;i++)
{
	for(j=0;j<input->cols;j++)
	{
		tmp=cvmGet(input,i,j);
		if(max<tmp)
			max=tmp;
	}
}

printf("max =%g\n",max);
*/
return 1;
}
