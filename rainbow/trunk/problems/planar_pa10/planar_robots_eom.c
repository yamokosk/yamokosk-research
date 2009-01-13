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

#define P_TRQ   prhs[0]     
#define P_Q     prhs[1]     
#define P_QP    prhs[2]     
#define P_QPP   plhs[0]     

void	 evaluateZees(void);

const double	G = 9.801;
double	Q1,Q2,Q3,T1,T2,T3,U1,U2,U3;
double	TF1,TF2,TF3,U1p,U2p,U3p;
double	z[96];

/* ................................ MAIN ............................. */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    int nrows=0, ncols=0, n=0;
	double *pTorque=NULL, *pQ=NULL, *pQP=NULL, *pQPP=NULL;

    // Check input for correct number of rows and columns
    if (mxGetM( P_TRQ ) != 6)   mexErrMsgTxt("Not enough torques specified.");
    if (mxGetM( P_Q ) != 6)     mexErrMsgTxt("Not enough Qs specified.");
    if (mxGetM( P_QP ) != 6)    mexErrMsgTxt("Not enough Qps specified.");
 	
    if ( ( mxGetN( P_TRQ ) != mxGetN( P_Q ) ) || ( mxGetN( P_TRQ ) != mxGetN( P_QP ) ) )
        mexErrMsgTxt("U, Q, and QP must all have the same number of columns.");
    
    // Assign pointers to each input and output.
	pTorque = mxGetPr( P_TRQ );
	pQ  = mxGetPr( P_Q );
	pQP = mxGetPr( P_QP );
    
    // Create storage for solutions
    nrows = mxGetM(P_Q); ncols = mxGetN(P_Q);
	P_QPP = mxCreateDoubleMatrix(nrows, ncols, mxREAL);
	pQPP = mxGetPr(P_QPP);
    
    // Evaluate the EOM for each column in the input data
    for (n=0; n < ncols; ++n)
    {
        // Robot #1
        // Define joint torques from input
        T1 = *(pTorque + n*6);
        T2 = *(pTorque + n*6 + 1);
        T3 = *(pTorque + n*6 + 2);

        // Define joint angles from input
        Q1 = *(pQ + n*6);
        Q2 = *(pQ + n*6 + 1);
        Q3 = *(pQ + n*6 + 2);
	
        // Define joint velocities from input
        U1 = *(pQP + n*6);
        U2 = *(pQP + n*6 + 1);
        U3 = *(pQP + n*6 + 2);
        
        evaluateZees();
        
        // Record output
        pQPP[n*6]       = U1p;
        pQPP[n*6 + 1]   = U2p;
        pQPP[n*6 + 2]   = U3p;
        
        
        // Robot #2
        // Define joint torques from input
        T1 = *(pTorque + n*6 + 3);
        T2 = *(pTorque + n*6 + 4);
        T3 = *(pTorque + n*6 + 5);

        // Define joint angles from input
        Q1 = *(pQ + n*6 + 3);
        Q2 = *(pQ + n*6 + 4);
        Q3 = *(pQ + n*6 + 5);
	
        // Define joint velocities from input
        U1 = *(pQP + n*6 + 3);
        U2 = *(pQP + n*6 + 4);
        U3 = *(pQP + n*6 + 5);
        
        evaluateZees();
        
        // Record output
        pQPP[n*6 + 3]   = U1p;
        pQPP[n*6 + 4]   = U2p;
        pQPP[n*6 + 5]   = U3p;
    }
}

/* ................................ EQNS .............................. */
void evaluateZees(void)
{
  z[5] = cos(Q3);
  z[6] = sin(Q3);
  z[9] = pow(z[5],2) + pow(z[6],2);
  z[3] = cos(Q2);
  z[1] = cos(Q1);
  z[2] = sin(Q1);
  z[7] = pow(z[1],2) + pow(z[2],2);
  z[20] = z[3]*z[7];
  z[23] = 0.45*z[20] + 0.48*z[7];
  z[4] = sin(Q2);
  z[21] = z[4]*z[7];
  z[26] = z[5]*z[23] - 0.45*z[6]*z[21];
  z[29] = z[26] - 0.042*z[7];
  z[31] = z[26] + 0.07000000000000001*z[7];
  z[33] = z[31] - 0.048*z[7];
  z[70] = z[9]*(10.35*z[29]-2.750000000000001*z[33]-z[7]);
  z[8] = pow(z[3],2) + pow(z[4],2);
  z[28] = z[6]*z[8];
  z[25] = z[5]*z[8];
  z[32] = 0.07000000000000001*z[8] + 0.48*z[25];
  z[34] = z[32] - 0.048*z[8];
  z[30] = 0.48*z[25] - 0.042*z[8];
  z[73] = 0.5140846022000001*pow(z[8],2) + 0.7188479999999999*pow(z[28],2) + 
  1.05*pow(z[34],2) + 2.07*pow(z[30],2);
  z[22] = 0.0485*z[7] + 0.45*z[20];
  z[24] = z[23] - 0.1122*z[7];
  z[27] = z[6]*z[23] + 0.45*z[5]*z[21];
  z[69] = 0.08600000000000001*z[7]*z[8] + 0.23571*z[8]*z[22] + 1.05*z[33]*
  z[34] + 1.132824*z[8]*z[24] + 1.4976*z[27]*z[28] + 2.07*z[29]*z[30];
  z[74] = z[9]*(10.35*z[30]-2.750000000000001*z[34]-z[8]);
  z[85] = 0.0084*z[70]*z[73] - 0.0084*z[69]*z[74];
  TF3 = 1.84*U3 + 1.58*tanh(10*U3);
  z[48] = T3 - TF3;
  z[50] = 3.67320510348261E-06*z[1] + 3.67320510350739E-06*z[2];
  z[52] = 3.67320510348261E-06*z[2] - 3.67320510350739E-06*z[1];
  z[55] = z[3]*z[50] - z[4]*z[52];
  z[58] = z[3]*z[52] + z[4]*z[50];
  z[61] = z[5]*z[55] - z[6]*z[58];
  z[67] = z[9]*(15.66416040100251*z[48]+G*z[61]);
  z[10] = z[7]*U1;
  z[35] = pow(z[10],2);
  z[36] = z[4]*z[35];
  z[37] = z[3]*z[35];
  z[11] = z[8]*U2;
  z[39] = -0.45*z[37] - 0.48*(z[7]*U1+z[8]*U2)*(z[10]+z[11]);
  z[41] = 0.45*z[5]*z[36] - z[6]*z[39];
  z[79] = z[9]*z[41];
  z[82] = 0.06383999999999999*z[67] + 0.06383999999999998*z[79];
  z[78] = pow(z[9],2);
  z[77] = z[9]*(10.35*z[30]-2.750000000000001*z[34]-z[8]);
  z[87] = 0.01255968*z[73]*z[78] - 7.055999999999999E-05*z[74]*z[77];
  TF2 = 9.390000000000001*U2 + 10.89*tanh(10*U2);
  z[47] = T2 - TF2;
  TF1 = 14.7*U1 + 23.14*tanh(10*U1);
  z[46] = T1 - TF1;
  z[64] = z[5]*z[58] + z[6]*z[55];
  z[65] = z[7]*z[47] + z[7]*(z[46]-z[47]) - 2.07*G*(z[27]*z[64]+z[29]*z[61]) - 
  1.05*G*(z[27]*z[64]+z[33]*z[61]) - 0.3729*G*z[7]*(z[52]+4.300000000000001*
  z[50]) - 2.187*G*(z[21]*z[58]+2.222222222222222*z[22]*z[55]) - 1.386*G*(
  z[21]*z[58]+2.222222222222222*z[24]*z[55]);
  z[42] = z[5]*z[39] + 0.45*z[6]*z[36];
  z[12] = z[9]*U3;
  z[44] = z[42] - 0.07000000000000001*(z[7]*U1+z[8]*U2+z[9]*U3)*(z[10]+z[11]+
  z[12]);
  z[45] = z[44] + 0.048*(z[7]*U1+z[8]*U2+z[9]*U3)*(z[10]+z[11]+z[12]);
  z[40] = z[39] + 0.1122*(z[7]*U1+z[8]*U2)*(z[10]+z[11]);
  z[43] = z[42] + 0.042*(z[7]*U1+z[8]*U2+z[9]*U3)*(z[10]+z[11]+z[12]);
  z[38] = -0.45*z[37] - 0.0485*(z[7]*U1+z[8]*U2)*(z[10]+z[11]);
  z[71] = 1.05*z[27]*z[45] + 1.05*z[33]*z[41] + 1.386*z[21]*z[40] + 1.386*
  z[24]*z[36] + 2.07*z[27]*z[43] + 2.07*z[29]*z[41] + 2.187*z[21]*z[38] + 
  2.187*z[22]*z[36] - 5.390653201597928E-18*z[7]*z[35];
  z[80] = z[65] - z[71];
  z[90] = 0.01255968*z[69]*z[78] - 7.055999999999999E-05*z[70]*z[77];
  z[66] = z[8]*z[47] - 1.368534*G*z[8]*z[55] - 0.9935999999999999*G*(z[28]*
  z[64]+2.083333333333334*z[30]*z[61]) - 0.504*G*(z[28]*z[64]+2.083333333333334*
  z[34]*z[61]);
  z[75] = 0.504*z[28]*z[45] + 0.6158403000000001*z[8]*z[36] + 0.9936*z[28]*
  z[43] + 1.05*z[34]*z[41] + 2.07*z[30]*z[41];
  z[81] = z[66] - z[75];
  z[76] = z[9]*(10.35*z[29]-2.750000000000001*z[33]-z[7]);
  z[68] = 0.6798346300000001*pow(z[7],2) + 1.05*pow(z[33],2) + 1.60785*pow(
  z[21],2) + 2.07*pow(z[29],2) + 3.08*pow(z[24],2) + 3.12*pow(z[27],2) + 4.86*pow(z[22],2);
  z[72] = 0.08600000000000001*z[7]*z[8] + 0.23571*z[8]*z[22] + 1.05*z[33]*
  z[34] + 1.132824*z[8]*z[24] + 1.4976*z[27]*z[28] + 2.07*z[29]*z[30];
  z[83] = z[68]*z[73] - z[69]*z[72];
  z[84] = 0.0084*z[70]*z[72] - 0.0084*z[68]*z[74];
  z[86] = 0.0084*z[76]*z[85] - 0.01255968*z[78]*z[83] - 0.0084*z[77]*z[84];
  z[93] = (z[85]*z[82]+z[87]*z[80]-z[90]*z[81])/z[86];
  U1p = -z[93];
  z[88] = 0.01255968*z[72]*z[78] - 7.055999999999999E-05*z[74]*z[76];
  z[91] = 0.01255968*z[68]*z[78] - 7.055999999999999E-05*z[70]*z[76];
  z[94] = (z[84]*z[82]+z[88]*z[80]-z[91]*z[81])/z[86];
  U2p = z[94];
  z[89] = 0.0084*z[73]*z[76] - 0.0084*z[72]*z[77];
  z[92] = 0.0084*z[69]*z[76] - 0.0084*z[68]*z[77];
  z[95] = (z[83]*z[82]+z[89]*z[80]-z[92]*z[81])/z[86];
  U3p = -z[95];
}

