/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __OPENCV_CONDENS_HPP__
#define __OPENCV_CONDENS_HPP__

#ifdef __cplusplus
extern "C" {
#endif

/************************************************************************/
/************************ Copy from compat.cpp **************************/
/************************************************************************/

typedef struct CvRandState
{
    CvRNG     state;    /* RNG state (the current seed and carry)*/
    int       disttype; /* distribution type */
    CvScalar  param[2]; /* parameters of RNG */
} CvRandState;

/* Changes RNG range while preserving RNG state */
CV_EXPORTS void  cvRandSetRange( CvRandState* state, double param1,
                                 double param2, int index CV_DEFAULT(-1));

CV_EXPORTS void  cvRandInit( CvRandState* state, double param1,
                             double param2, int seed,
                             int disttype CV_DEFAULT(CV_RAND_UNI));

/* Fills array with random numbers */
CV_EXPORTS void cvRand( CvRandState* state, CvArr* arr );

#define cvRandNext( _state ) cvRandInt( &(_state)->state )

CV_EXPORTS void cvbRand( CvRandState* state, float* dst, int len );

/************************************************************************/

typedef struct CvConDensation
{
    int MP;
    int DP;
    float* DynamMatr;       /* Matrix of the linear Dynamics system  */
    float* State;           /* Vector of State                       */
    int SamplesNum;         /* Number of the Samples                 */
    float** flSamples;      /* arr of the Sample Vectors             */
    float** flNewSamples;   /* temporary array of the Sample Vectors */
    float* flConfidence;    /* Confidence for each Sample            */
    float* flCumulative;    /* Cumulative confidence                 */
    float* Temp;            /* Temporary vector                      */
    float* RandomSample;    /* RandomVector to update sample set     */
    struct CvRandState* RandS; /* Array of structures to generate random vectors */
} CvConDensation;

/* Creates ConDensation filter state */
CVAPI(CvConDensation*)  cvCreateConDensation( int dynam_params,
                                             int measure_params,
                                             int sample_count );

/* Releases ConDensation filter state */
CVAPI(void)  cvReleaseConDensation( CvConDensation** condens );

/* Updates ConDensation filter by time (predict future state of the system) */
CVAPI(void)  cvConDensUpdateByTime( CvConDensation* condens);

/* Initializes ConDensation filter samples  */
CVAPI(void)  cvConDensInitSampleSet( CvConDensation* condens, CvMat* lower_bound, CvMat* upper_bound );

CV_INLINE int iplWidth( const IplImage* img )
{
    return !img ? 0 : !img->roi ? img->width : img->roi->width;
}

CV_INLINE int iplHeight( const IplImage* img )
{
    return !img ? 0 : !img->roi ? img->height : img->roi->height;
}

#ifdef __cplusplus
}
#endif

#endif

/* End of file. */
