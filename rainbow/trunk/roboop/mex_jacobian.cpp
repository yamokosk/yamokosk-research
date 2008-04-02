#include "mex_common.h"

ROBOOP_MEX_FUNC_START
    
    if (nrhs != 3) ERROR_MSG(INVALID_NUM_ARGS, "Expecting three arguments.");
    
	ColumnVector q = NMArrayFromMxArray(RHS_ARG_1);
	int endlink = (int)mxGetScalar(RHS_ARG_2);
	int ref = (int)mxGetScalar(RHS_ARG_3);
    	
	// Calling ROBOOP function
	robj.set_q(q);
	Matrix J = robj.jacobian(endlink, ref);

	// Setting output
	LHS_ARG_1 = mxArrayFromNMArray(J);
        
ROBOOP_MEX_FUNC_STOP