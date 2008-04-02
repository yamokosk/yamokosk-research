#include "mex_common.h"

ROBOOP_MEX_FUNC_START
    
    if (nrhs != 2) ERROR_MSG(INVALID_NUM_ARGS, "Expecting two arguments.");
    
	ColumnVector q = NMArrayFromMxArray(RHS_ARG_1);
	int link = (int)mxGetScalar(RHS_ARG_2);
    	
	// Calling ROBOOP function
	robj.set_q(q);
	Matrix T = robj.kine(link);

	// Setting output
	LHS_ARG_1 = mxArrayFromNMArray(T);
        
ROBOOP_MEX_FUNC_STOP