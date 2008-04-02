#include "mex_common.h"

ROBOOP_MEX_FUNC_START
    
    if (nrhs != 4) ERROR_MSG(INVALID_NUM_ARGS, "Expecting four arguments.");
    
	ColumnVector q = NMArrayFromMxArray(RHS_ARG_1);
	double eps = mxGetScalar(RHS_ARG_2);
	double lambda_max = mxGetScalar(RHS_ARG_3);
	int ref = (int)mxGetScalar(RHS_ARG_4);
    	
	// Calling ROBOOP function
	robj.set_q(q);
	Matrix J_inv = robj.jacobian_DLS_inv(eps, lambda_max, ref);

	// Setting output
	LHS_ARG_1 = mxArrayFromNMArray(J_inv);
        
ROBOOP_MEX_FUNC_STOP