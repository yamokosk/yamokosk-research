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
#define G			9.81
#define _NAN        9.99E+305

void     define(const double *Torque, const double *Q, const double *U, const double *Theta);
void	 evaluate(void);
void     output(double Up[]);

double   BETA,C1,C2,C3,C4,C5,C6,L1,L2,LN,M1,M2,M3,R,T1,T2,T3,T4,T5,T6,V1,V2,V3,
  V4,V5,V6,Q1,Q2,Q3,Q4,Q5,Q6,U1,U2,U3,U4,U5,U6;
double   TF1,TF2,TF3,TF4,TF5,TF6,U1p,U2p,U3p,U4p,U5p,U6p;
double   z[182];


/* ................................ MAIN ............................. */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	int nrows = 6; int ncols = 1;
	int ii, jj, n; int m = 0;
	double *Torque, *Q, *U, *Theta;
	double *Up;

    /* Assign pointers to each input and output. */
	Torque = mxGetPr(prhs[0]);
	Q  = mxGetPr(prhs[1]);
	U = mxGetPr(prhs[2]);
	Theta = mxGetPr(prhs[3]);
    if (mxGetM(prhs[0]) != 6)
    {
        mexErrMsgTxt("Not enough torques specified.");
    }

    if (mxGetM(prhs[1]) != 6)
    {
        mexErrMsgTxt("Not enough Qs specified.");
    }

    if (mxGetM(prhs[2]) != 6)
    {
        mexErrMsgTxt("Not enough Qps specified.");
    }

    if (mxGetM(prhs[3]) != 20)
    {
        mexErrMsgTxt("Not enough arguments specified for theta vector.");
    }
	
	/* Evaluate output quantities */
	define(Torque,Q,U,Theta);
	evaluate();
	
   	/* Create matrix for the return argument. */
	plhs[0] = mxCreateDoubleMatrix(nrows,ncols, mxREAL);
	Up = mxGetPr(plhs[0]);

    output(Up);
}

/* ............................. DEFINITIONS ........................... */

void define(const double *Torque, const double *Q, const double *U, const double *Theta)
{
	// define joint torques from input
	T1 = Torque[0];
	T2 = Torque[1];
	T3 = Torque[2];
	T4 = Torque[3];
	T5 = Torque[4];
	T6 = Torque[5];

	// define joint angles from input
	Q1 = Q[0];
	Q2 = Q[1];
	Q3 = Q[2];
	Q4 = Q[3];
	Q5 = Q[4];
	Q6 = Q[5];
	
	// define joint velocities from input
	U1 = U[0];
	U2 = U[1];
	U3 = U[2];
	U4 = U[3];
	U5 = U[4];
	U6 = U[5];
	
	// define robot parameters from theta
    LN = Theta[0];
    L1 = Theta[1];
    L2 = Theta[2];
    R = Theta[3];
    M1 = Theta[4];
    M2 = Theta[5];
    M3 = Theta[6];
    C1 = Theta[7];
    C2 = Theta[8];
    C3 = Theta[9];
    C4 = Theta[10];
    C5 = Theta[11];
    C6 = Theta[12];
    V1 = Theta[13];
    V2 = Theta[14];
    V3 = Theta[15];
    V4 = Theta[16];
    V5 = Theta[17];
    V6 = Theta[18];
    BETA = Theta[19];
}

/* ................................ OUTPUT ............................. */

void output(double Up[])
{
	Up[0]=U1p;
	Up[1]=U2p;
	Up[2]=U3p;
	Up[3]=U4p;
	Up[4]=U5p;
	Up[5]=U6p;
}

/* ................................ EQNS .............................. */

void evaluate(void)
{
/* Evaluate constants */
  z[4] = cos(Q1);
  z[5] = sin(Q1);
  z[16] = pow(z[4],2) + pow(z[5],2);
  TF1 = V1*U1 + C1*tanh(BETA*U1);
  z[84] = T1 - TF1;
  z[90] = z[16]*z[84];
  z[6] = cos(Q2);
  z[34] = LN*z[16];
  z[40] = z[5]*z[34];
  z[7] = sin(Q2);
  z[39] = z[4]*z[34];
  z[35] = L1*z[16];
  z[41] = z[39] - z[35];
  z[45] = -z[6]*z[40] - z[7]*z[41];
  z[22] = z[16]*U1;
  z[64] = z[34]*U1*z[22];
  z[69] = z[4]*z[64];
  z[17] = pow(z[6],2) + pow(z[7],2);
  z[36] = L1*z[17];
  z[23] = z[17]*U2;
  z[70] = z[69] - (z[35]*U1+z[36]*U2)*(z[22]+z[23]);
  z[68] = z[5]*z[64];
  z[72] = z[6]*z[70] - z[7]*z[68];
  z[37] = L2*z[16];
  z[38] = L2*z[17];
  z[8] = cos(Q3);
  z[9] = sin(Q3);
  z[18] = pow(z[8],2) + pow(z[9],2);
  z[46] = L2*z[18];
  z[24] = z[18]*U3;
  z[73] = z[72] - (z[37]*U1+z[38]*U2+z[46]*U3)*(z[22]+z[23]+z[24]);
  z[42] = z[6]*z[41] - z[7]*z[40];
  z[47] = z[42] - z[37];
  z[71] = z[6]*z[68] + z[7]*z[70];
  z[65] = z[35]*U1*z[22];
  z[66] = (z[35]*U1+z[36]*U2)*(z[22]+z[23]);
  z[67] = (z[37]*U1+z[38]*U2)*(z[22]+z[23]);
  z[120] = -z[4]*z[7] - z[5]*z[6];
  z[119] = z[4]*z[7] + z[5]*z[6];
  z[127] = M3*(z[45]*z[73]+z[47]*z[71]) + 0.5*M1*z[5]*(z[34]*z[65]-z[35]*
  z[64]) + 0.5*M2*(z[7]*z[37]*z[66]+2*z[5]*z[34]*z[66]-2*z[5]*z[35]*z[64]-
  z[7]*z[35]*z[67]-z[34]*z[120]*z[67]-z[37]*z[119]*z[64]);
  z[148] = z[90] - z[127];
  z[2] = M2*pow(L2,2);
  z[101] = z[2]*z[17];
  z[3] = M3*pow(R,2);
  z[110] = z[3]*z[17];
  z[44] = z[7]*z[36];
  z[43] = z[6]*z[36];
  z[48] = -z[38] - z[43];
  z[118] = z[4]*z[6] - z[5]*z[7];
  z[125] = 0.08333333333333333*z[16]*z[101] + 0.4*z[16]*z[110] + M3*(z[44]*
  z[45]+z[47]*z[48]) + 0.25*M2*(z[37]*z[38]+4*z[35]*z[36]+2*z[6]*z[35]*z[38]+
  2*z[6]*z[36]*z[37]-4*z[4]*z[34]*z[36]-2*z[34]*z[38]*z[118]);
  z[100] = z[2]*z[16];
  z[109] = z[3]*z[16];
  z[128] = 0.08333333333333333*z[17]*z[100] + 0.4*z[17]*z[109] + M3*(z[44]*
  z[45]+z[47]*z[48]) + 0.25*M2*(z[37]*z[38]+4*z[35]*z[36]+2*z[6]*z[35]*z[38]+
  2*z[6]*z[36]*z[37]-4*z[4]*z[34]*z[36]-2*z[34]*z[38]*z[118]);
  z[1] = M1*pow(L1,2);
  z[96] = z[1]*z[16];
  z[124] = 0.08333333333333333*z[16]*z[96] + 0.08333333333333333*z[16]*z[100] + 
  0.4*z[16]*z[109] + M3*(pow(z[45],2)+pow(z[47],2)) + 0.25*M1*(pow(z[35],2)+4*
  pow(z[34],2)-4*z[4]*z[34]*z[35]) + 0.25*M2*(pow(z[37],2)+4*pow(z[34],2)+4*
  pow(z[35],2)+4*z[6]*z[35]*z[37]-8*z[4]*z[34]*z[35]-4*z[34]*z[37]*z[118]);
  z[154] = z[128]/z[124];
  TF2 = V2*U2 + C2*tanh(BETA*U2);
  z[85] = T2 - TF2;
  z[91] = z[17]*z[85];
  z[131] = M3*(z[44]*z[73]+z[48]*z[71]) + 0.5*M2*(z[7]*z[38]*z[66]-2*z[5]*
  z[36]*z[64]-z[7]*z[36]*z[67]-z[38]*z[119]*z[64]);
  z[149] = z[91] - z[131];
  z[157] = z[154]*z[148] - z[149];
  z[111] = z[3]*z[18];
  z[126] = 0.4*z[16]*z[111] - M3*z[46]*z[47];
  z[130] = 0.4*z[17]*z[111] - M3*z[46]*z[48];
  z[156] = z[126]*z[154] - z[130];
  z[132] = 0.4*z[18]*z[109] - M3*z[46]*z[47];
  z[158] = z[132]/z[124];
  TF3 = V3*U3 + C3*tanh(BETA*U3);
  z[86] = T3 - TF3;
  z[92] = z[18]*z[86];
  z[135] = M3*z[46]*z[71];
  z[150] = z[92] + z[135];
  z[161] = z[158]*z[148] - z[150];
  z[133] = 0.4*z[18]*z[110] - M3*z[46]*z[48];
  z[159] = z[125]*z[158] - z[133];
  z[129] = 0.08333333333333333*z[17]*z[101] + 0.4*z[17]*z[110] + M3*(pow(
  z[44],2)+pow(z[48],2)) + 0.25*M2*(pow(z[38],2)+4*pow(z[36],2)+4*z[6]*z[36]*
  z[38]);
  z[155] = z[125]*z[154] - z[129];
  z[162] = z[159]/z[155];
  z[164] = z[161] - z[162]*z[157];
  z[134] = 0.4*z[18]*z[111] + M3*pow(z[46],2);
  z[160] = z[126]*z[158] - z[134];
  z[163] = z[160] - z[156]*z[162];
  z[165] = z[164]/z[163];
  z[166] = (z[157]-z[156]*z[165])/z[155];
  z[167] = (z[148]-z[125]*z[166]-z[126]*z[165])/z[124];
  U1p = z[167];
  U2p = z[166];
  U3p = z[165];
  z[10] = cos(Q4);
  z[11] = sin(Q4);
  z[19] = pow(z[10],2) + pow(z[11],2);
  TF4 = V4*U4 + C4*tanh(BETA*U4);
  z[87] = T4 - TF4;
  z[93] = z[19]*z[87];
  z[49] = LN*z[19];
  z[50] = L1*z[19];
  z[28] = z[19]*U4;
  z[75] = z[50]*U4*z[28];
  z[74] = z[49]*U4*z[28];
  z[12] = cos(Q5);
  z[55] = z[11]*z[49];
  z[13] = sin(Q5);
  z[54] = z[10]*z[49];
  z[56] = z[54] - z[50];
  z[60] = -z[12]*z[55] - z[13]*z[56];
  z[52] = L2*z[19];
  z[20] = pow(z[12],2) + pow(z[13],2);
  z[53] = L2*z[20];
  z[14] = cos(Q6);
  z[15] = sin(Q6);
  z[21] = pow(z[14],2) + pow(z[15],2);
  z[61] = L2*z[21];
  z[29] = z[20]*U5;
  z[30] = z[21]*U6;
  z[83] = (z[52]*U4+z[53]*U5+z[61]*U6)*(z[28]+z[29]+z[30]);
  z[51] = L1*z[20];
  z[81] = L1*z[21];
  z[82] = (z[50]*U4+z[51]*U5+z[81]*U6)*(z[28]+z[29]+z[30]);
  z[57] = z[12]*z[56] - z[13]*z[55];
  z[62] = z[57] - z[52];
  z[121] = z[11]*z[13] - z[10]*z[12];
  z[122] = -z[10]*z[13] - z[11]*z[12];
  z[77] = (z[52]*U4+z[53]*U5)*(z[28]+z[29]);
  z[76] = (z[50]*U4+z[51]*U5)*(z[28]+z[29]);
  z[123] = z[10]*z[13] + z[11]*z[12];
  z[139] = 0.5*M1*z[11]*(z[49]*z[75]-z[50]*z[74]) - M3*(z[60]*z[83]+z[12]*
  z[60]*z[82]+z[13]*z[62]*z[82]+z[60]*z[121]*z[74]+z[62]*z[122]*z[74]) - 0.5*
  M2*(z[13]*z[50]*z[77]+2*z[11]*z[50]*z[74]-2*z[11]*z[49]*z[76]-z[13]*z[52]*
  z[76]-z[49]*z[123]*z[77]-z[52]*z[122]*z[74]);
  z[151] = z[93] - z[139];
  z[105] = z[2]*z[20];
  z[116] = z[3]*z[20];
  z[137] = 0.08333333333333333*z[19]*z[105] + 0.4*z[19]*z[116] - M3*(z[53]*
  z[62]+z[12]*z[51]*z[62]-z[13]*z[51]*z[60]) - 0.25*M2*(4*z[10]*z[49]*z[51]-4*
  z[50]*z[51]-z[52]*z[53]-2*z[12]*z[50]*z[53]-2*z[12]*z[51]*z[52]-2*z[49]*
  z[53]*z[121]);
  z[104] = z[2]*z[19];
  z[115] = z[3]*z[19];
  z[59] = z[13]*z[51];
  z[58] = z[12]*z[51];
  z[63] = -z[53] - z[58];
  z[140] = 0.08333333333333333*z[20]*z[104] + 0.4*z[20]*z[115] + M3*(z[13]*
  z[50]*z[59]-z[52]*z[63]-z[12]*z[50]*z[63]-z[49]*z[59]*z[123]-z[49]*z[63]*
  z[121]) - 0.25*M2*(4*z[10]*z[49]*z[51]-4*z[50]*z[51]-z[52]*z[53]-2*z[12]*
  z[50]*z[53]-2*z[12]*z[51]*z[52]-2*z[49]*z[53]*z[121]);
  z[97] = z[1]*z[19];
  z[136] = 0.08333333333333333*z[19]*z[97] + 0.08333333333333333*z[19]*z[104] + 
  0.4*z[19]*z[115] + 0.25*M1*(pow(z[50],2)+4*pow(z[49],2)-4*z[10]*z[49]*z[50]) + 
  M3*(z[13]*z[50]*z[60]-z[52]*z[62]-z[12]*z[50]*z[62]-z[49]*z[60]*z[123]-
  z[49]*z[62]*z[121]) - 0.25*M2*(8*z[10]*z[49]*z[50]-4*pow(z[49],2)-4*pow(
  z[50],2)-pow(z[52],2)-4*z[12]*z[50]*z[52]-4*z[49]*z[52]*z[121]);
  z[168] = z[140]/z[136];
  TF5 = V5*U5 + C5*tanh(BETA*U5);
  z[88] = T5 - TF5;
  z[94] = z[20]*z[88];
  z[143] = -M3*(z[59]*z[83]+z[12]*z[59]*z[82]+z[13]*z[63]*z[82]+z[59]*z[121]*
  z[74]+z[63]*z[122]*z[74]) - 0.5*M2*(z[13]*z[51]*z[77]+2*z[11]*z[51]*z[74]-
  z[13]*z[53]*z[76]-z[53]*z[122]*z[74]);
  z[152] = z[94] - z[143];
  z[171] = z[168]*z[151] - z[152];
  z[117] = z[3]*z[21];
  z[138] = 0.4*z[19]*z[117] - M3*(z[61]*z[62]+z[12]*z[62]*z[81]-z[13]*z[60]*
  z[81]);
  z[142] = 0.4*z[20]*z[117] - M3*(z[61]*z[63]+z[12]*z[63]*z[81]-z[13]*z[59]*
  z[81]);
  z[170] = z[138]*z[168] - z[142];
  z[144] = 0.4*z[21]*z[115] + M3*z[61]*(z[52]+z[12]*z[50]+z[49]*z[121]);
  z[172] = z[144]/z[136];
  TF6 = V6*U6 + C6*tanh(BETA*U6);
  z[89] = T6 - TF6;
  z[95] = z[21]*z[89];
  z[147] = M3*z[61]*(z[13]*z[82]+z[122]*z[74]);
  z[153] = z[95] - z[147];
  z[175] = z[172]*z[151] - z[153];
  z[145] = 0.4*z[21]*z[116] + M3*z[61]*(z[53]+z[12]*z[51]);
  z[173] = z[137]*z[172] - z[145];
  z[141] = 0.08333333333333333*z[20]*z[105] + 0.4*z[20]*z[116] + 0.25*M2*(
  pow(z[53],2)+4*pow(z[51],2)+4*z[12]*z[51]*z[53]) - M3*(z[53]*z[63]+z[12]*
  z[51]*z[63]-z[13]*z[51]*z[59]);
  z[169] = z[137]*z[168] - z[141];
  z[176] = z[173]/z[169];
  z[178] = z[175] - z[176]*z[171];
  z[146] = 0.4*z[21]*z[117] + M3*z[61]*(z[61]+z[12]*z[81]);
  z[174] = z[138]*z[172] - z[146];
  z[177] = z[174] - z[170]*z[176];
  z[179] = z[178]/z[177];
  z[180] = (z[171]-z[170]*z[179])/z[169];
  z[181] = (z[151]-z[137]*z[180]-z[138]*z[179])/z[136];
  U4p = z[181];
  U5p = z[180];
  U6p = z[179];
}

