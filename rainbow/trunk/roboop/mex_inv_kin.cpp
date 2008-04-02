#include "mex_common.h"

ROBOOP_MEX_FUNC_START
    
	if (nrhs != 3) ERROR_MSG(INVALID_NUM_ARGS, "Expecting three arguments.");
	
    bool hasConverged = false;
    Matrix Tobj = NMArrayFromMxArray( RHS_ARG_1 );
	int mj = (int)mxGetScalar(RHS_ARG_2);
    int endlink = (int)mxGetScalar(RHS_ARG_3);
    	
    // Calling ROBOOP function
    ColumnVector Q = robj.inv_kin(Tobj, mj, endlink, hasConverged);
	mexPrintf("size of Q = %d\n", Q.Storage());
	
    // Setting output
	LHS_ARG_1 = mxArrayFromNMArray(Q);
    
    if ( nlhs > 1 )
        LHS_ARG_2 = mxCreateLogicalScalar(hasConverged);
        
ROBOOP_MEX_FUNC_STOP