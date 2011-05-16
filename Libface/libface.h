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
    

#ifndef _LIBFACE_H
#define _LIBFACE_H
#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#endif

#ifndef _EiC
#include <iostream>
#include "cvaux.h"
#include "cxcore.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#endif

#ifdef _EiC
#define WIN32
#endif

#include <vector>
#include <fstream>
#include "gnuplot_i.h"

using namespace std;

#define COUNTER_CLOCKWISE 1
#define CLOCKWISE -1
#define ORIG_WIN_SIZE  24

static CvMemStorage* storage1 = 0;
static CvMemStorage* storage = 0;
static CvHidHaarClassifierCascade* hid_cascade = 0;

#define PI 3.14
#define ORIG_WIN_SIZE  24
#define IMG_WIDTH 180
#define IMG_HEIGHT 180
#define PHASE_AND_MAG 0
#define MAG 1
#define MAG_EUC 2
#define DIS 4




#define TRC 90


#define XML_PATH "haarcascade_frontalface_default.xml"
#define MASK_PATH "Mask.jpg"

typedef struct pola
{
  double r;
  double theta;
}Polar;

typedef struct vec
{
  double x;
  double y;
}Vec;

typedef struct CNum
{
  double real;
  double img;
}ComplexNum;

typedef struct Information
{
	int a_rowsz;
	int a_colsz;
	int ev_aaT_rowsz;
	int ev_aaT_colsz;
	int avgsz;
}Info;

typedef struct OnlyTwoCvPoints
{
	CvPoint p;
	CvPoint q;

}TwoCvPoint;



typedef struct Pnt
{
	int x;
	int y;

	unsigned int intensity;

}Point_Info;

typedef struct cir
{
	CvPoint centre;
	int radius;
}Circle;

typedef struct feat
{
	string name;
	CvPoint l_eye;
	CvPoint r_eye;
	CvPoint nose;
}FaceFeat;


typedef struct feature
{
	string name;
	CvPoint l_eyel;
	CvPoint l_eyer;
	CvPoint r_eyel;
	CvPoint r_eyer;

}EyeCoor;

typedef struct Res
{
	string name;
	double distance;
        int index;
        int id;

}ResInf;

typedef struct InputInfo
{
     string filename;
     int id;
}InpInf;

/**
  @brief    Plot a curve of given equation y=f(x).
  @param    h           Gnuplot session control handle.
  @param    equation    Equation to plot.
  @param    title       Title of the plot.
  @return   void

  Plots out a curve of given equation. The general form of the
  equation is y=f(x), you only provide the f(x) side of the equation.

  Example:

  @code
        gnuplot_ctrl    *h ;
        char            eq[80] ;

        h = gnuplot_init() ;
        strcpy(eq, "sin(x) * cos(2*x)") ;
        gnuplot_plot_equation(h, eq, "sine wave", normal) ;
        gnuplot_close(h) ;
  @endcode
 */

double mul_vec(Vec a, Vec b);
CvMat* Get_Mat(CvPoint a,int width,int height,IplImage *image);

TwoCvPoint Detect_and_Draw( IplImage* img, CvMemStorage *storage,  CvHaarClassifierCascade *cascade);
double Face_Distance(IplImage *image, string cascade_name);
IplImage* Face_Detect(IplImage *image,const char* cascade_name,const char* sMaskImgPath);
IplImage* Histogram_Equalize(IplImage* src);
bool Test_For_Hist(IplImage* inp);

IplImage* EdgeImage(IplImage*);
int Distance(CvPoint point1,CvPoint point2);
IplImage* rotate (IplImage *src, int sense, int delta);
float Angle(CvPoint point1,CvPoint point2,CvPoint point3,CvPoint point4);
IplImage* Rgb2Gray(IplImage *src);


void PCA(CvMat *inp_mat,CvMat *eval,CvMat *ev_aaT,CvMat *ev_aTa,CvMat *avg);

void Draw_Feature(IplImage *fea_img,vector<CvPoint> fpt);

double Get_Angle(Vec axis1,Vec axis2);
void Normalize_Mat_Cols(CvMat *inp_mat);

CvPoint Rotate_Pt(CvPoint inp,double angle,int tx,int ty);

Vec Normalize_Vec(Vec a);

Vec Rotate_And_Scale_Vec(Vec a,double scale,double theta);

int Get_Trace_Vectors(vector<double> eigen_vals);



int*** matrix3d(int size_x, int size_y, int size_z);
void error_fun(char *error_text);
IplImage* Rotate_Face(IplImage *input_image,CvPoint l_eye,CvPoint r_eye,CvPoint nose,double ref_dis,int img_width,int img_height);

IplImage* Mat2IplImage(CvMat *inp_mat,int type);
void Histogram_Equalize_Batch(CvMat* src);
void Batch_DCT(CvMat *inp_mat,int i_img_row,int i_img_col,int r_img_row,int r_img_col,CvMat *res_mat);

CvMat* Get_Sub_Mat(CvPoint start_pt,int width,int height,CvMat *inp_mat);
CvMat* IplImage2Mat(IplImage *inp_img);

IplImage* Remove_Non_Skin(IplImage *src);


void Plot2D_XY(double *x,double *y,int number_of_points);

int Write_CvMat(CvMat *inp,string filename);
CvMat* Read_CvMat(string filename);
int Roc_Curve(vector<int> true_class,vector<double> scores,vector<double> &tp,vector<double> &fp);

CvMat* lda(CvMat *input_mat,vector<int> class_info);
CvMat* Covariance_Mat(CvMat *input_mat);
CvMat* Scatter_Between(CvMat* input_mat,vector<int> class_info);
CvMat* Class_Mean_Mat(CvMat* input_mat,vector<int> class_info);
CvMat* Scatter_Within(CvMat* input_mat,vector<int> class_info);
int count_number_of_class(vector<int> class_info);
CvMat* Mean_Vector(CvMat *inp);
CvMat *LDA_Projection_Mat(CvMat *scatter_within,CvMat *scatter_between);

int Scale_Image(IplImage* inp,double scl);
int Scale_Mat(CvMat* inp,double scl);
CvMat* Gaussian(int size);
CvMat* Conv_Weighted_Gaussian(IplImage *inp_img,CvMat *kernel);
CvMat* Weighted_Gaussian(CvMat *inp,CvMat *gaussian);
IplImage* SQI(IplImage* inp);
vector<ResInf> test_lda(CvMat *trans_mat,CvMat *avg,CvMat *inp_wgt_mat,vector<InpInf> inp_info,CvMat *test_mat);
vector<ResInf> test_pca(CvMat *trans_mat,CvMat *avg,CvMat *inp_wgt_mat,vector<InpInf> inp_info,CvMat *test_mat);
CvMat* Construct_Mat(vector<string> filelist,int f_detect,string xml_path,string Mask_img_path);
int print_mat(CvMat *ip);
#endif
