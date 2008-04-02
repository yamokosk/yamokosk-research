#include "mex_common.h"

ROBOOP_MEX_FUNC_START
    
    if (nrhs != 3) ERROR_MSG(INVALID_NUM_ARGS, "Expecting three arguments.");
    
	ColumnVector q = NMArrayFromMxArray(RHS_ARG_1);
	ColumnVector qp = NMArrayFromMxArray(RHS_ARG_2);
	int ref = (int)mxGetScalar(RHS_ARG_3);
    	
	// Calling ROBOOP function
	robj.set_q(q);
	robj.set_qp(qp);
	Matrix J_dot = robj.jacobian_dot(ref);

	// Setting output
	LHS_ARG_1 = mxArrayFromNMArray(J_dot);
        
ROBOOP_MEX_FUNC_STOP