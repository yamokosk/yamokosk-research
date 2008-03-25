#include "mex.h"
#include <ctype.h> 
#include <math.h>  
#include <stdarg.h>
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>

/* Degrees to radians conversion */
#define Pi			3.14159265358979323846264338327950288419716939937510582
#define DEGtoRAD	0.01745329251994329576923690768488612713442871888541725
#define	RADtoDEG	57.2957795130823208767981548141051703324054724665643215
#define _NAN        9.99E+305

void     define(const double *Q);
void	 evaluate(void);
void     output(double J[]);

double	Q1,Q2,Q3;
double	z[95], J[6][3];

/* ................................ MAIN ............................. */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	int nrows = 6; int ncols = 3;
	int m = 0;
	double *Q, *pJ;

    /* Assign pointers to each input and output. */
	Q  = mxGetPr(prhs[0]);
    if (mxGetM(prhs[0]) != 3)
    {
        mexErrMsgTxt("Not enough Qs specified.");
    }
 	
	/* Evaluate output quantities */
	define(Q);
	evaluate();
	
   	/* Create matrix for the return argument. */
	plhs[0] = mxCreateDoubleMatrix(nrows,ncols, mxREAL);
	pJ = mxGetPr(plhs[0]);

    output(pJ);
}

/* ............................. DEFINITIONS ........................... */

void define(const double *Q)
{
	// define joint angles from input
	Q1 = Q[0];
	Q2 = Q[1];
	Q3 = Q[2];
}

/* ................................ OUTPUT ............................. */

void output(double pJ[])
{
	int r = 0, c = 0;
	for (r=0; r < 6; ++r)
	{
		for (c = 0; c < 3; ++c) {
			pJ[c*6 + r] = J[r][c];
			pJ[c*6 + r] = J[r][c];
		}
	}
}

/* ................................ EQNS .............................. */

void evaluate(void)
{
  z[1] = cos(Q1);
  z[2] = sin(Q1);
  z[3] = cos(Q2);
  z[4] = sin(Q2);
  z[5] = cos(Q3);
  z[6] = sin(Q3);
  z[49] = 1.349243573243274E-11*z[1] - 0.9999999999932537*z[2];
  z[50] = 3.67320510348261E-06*z[1] + 3.67320510350739E-06*z[2];
  z[51] = 1.349243573243274E-11*z[2] + 0.9999999999932537*z[1];
  z[52] = 3.67320510348261E-06*z[2] - 3.67320510350739E-06*z[1];
  z[53] = z[3]*z[49] - z[4]*z[51];
  z[54] = 0.9999999999932537*z[1]*z[3] - 0.9999999999932537*z[2]*z[4];
  z[55] = z[3]*z[50] - z[4]*z[52];
  z[56] = z[3]*z[51] + z[4]*z[49];
  z[57] = 0.9999999999932537*z[1]*z[4] + 0.9999999999932537*z[2]*z[3];
  z[58] = z[3]*z[52] + z[4]*z[50];
  z[59] = z[5]*z[53] - z[6]*z[56];
  z[60] = z[5]*z[54] - z[6]*z[57];
  z[61] = z[5]*z[55] - z[6]*z[58];
  z[83] = z[3]*(1.349243573243274E-11*z[1]-0.9999999999932537*z[2]) + z[4]*(
  -0.9999999999932537*z[1]-1.349243573243274E-11*z[2]);
  z[84] = z[3]*(-0.9999999999932537*z[1]-1.349243573243274E-11*z[2]) - z[4]*(
  1.349243573243274E-11*z[1]-0.9999999999932537*z[2]);
  z[85] = z[5]*z[83] + z[6]*z[84];
  z[86] = -z[3]*z[51] - z[4]*z[49];
  z[87] = z[5]*z[53] + z[6]*z[86];
  z[88] = -0.9999999999932537*z[1]*z[4] - 0.9999999999932537*z[2]*z[3];
  z[89] = z[5]*z[54] + z[6]*z[88];
  z[90] = z[3]*(3.67320510348261E-06*z[1]+3.67320510350739E-06*z[2]) + z[4]*(
  3.67320510350739E-06*z[1]-3.67320510348261E-06*z[2]);
  z[91] = z[3]*(3.67320510350739E-06*z[1]-3.67320510348261E-06*z[2]) - z[4]*(
  3.67320510348261E-06*z[1]+3.67320510350739E-06*z[2]);
  z[92] = z[5]*z[90] + z[6]*z[91];
  z[93] = -z[3]*z[52] - z[4]*z[50];
  z[94] = z[5]*z[55] + z[6]*z[93];
  
  J[0][0] = 0.02*z[85];
  J[0][1] = 0.02*z[87];
  J[0][2] = 0.02*z[59];
  J[1][0] = 0.02*z[89];
  J[1][1] = 0.02*z[89];
  J[1][2] = 0.02*z[60];
  J[2][0] = 0.02*z[92];
  J[2][1] = 0.02*z[94];
  J[2][2] = 0.02*z[61];
  J[3][0] = 0;
  J[3][1] = 0;
  J[3][2] = 0;
  J[4][0] = 0;
  J[4][1] = 0;
  J[4][2] = 0;
  J[5][0] = 0;
  J[5][1] = 0;
  J[5][2] = 0;
}
