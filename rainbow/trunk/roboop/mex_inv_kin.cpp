#include "mex_common.h"

ROBOOP_MEX_FUNC_START
    
	if (nrhs != 3) ERROR_MSG(INVALID_NUM_ARGS, "Expecting three arguments.");
	
    bool hasConverged = false;
    int mj = 0;
    int endlink = robj.get_dof();
    
    Matrix Tobj = NMArrayFromMxArray( RHS_ARG_1 );
	mj = (int)mxGetScalar(RHS_ARG_2);
    endlink = (int)mxGetScalar(RHS_ARG_3);
    
    // Calling ROBOOP function
    ColumnVector Q = robj.inv_kin(Tobj, mj, endlink, hasConverged);

    // Setting output
	LHS_ARG_1 = mxArrayFromNMArray(Q);
    
    if ( nlhs > 1 )
        LHS_ARG_2 = mxCreateLogicalScalar(hasConverged);
        
ROBOOP_MEX_FUNC_STOP