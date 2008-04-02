#include "common.h"

ROBOOP_MEX_FUNC_START
    
    bool hasConverged = false;
    int mj = 0;
    int endlink = robj.get_dof();
    
    Matrix Tobj(4,4);
    MxArrayToNMMatrix(Tobj, mxGetPr(RHS_ARG_1));
    
    if (nrhs > 1) {
        mj = (int)mxGetScalar(RHS_ARG_2);
    
        if (nrhs > 2)
            endlink = (int)mxGetScalar(RHS_ARG_3);
    }
    
    // Calling ROBOOP function
    ColumnVector Q = robj.inv_kin(Tobj, mj, endlink, hasConverged);

    // Setting output
    LHS_ARG_1 = mxCreateDoubleMatrix( 1, Q.Storage(), mxREAL );
    double *pQ = mxGetPr(LHS_ARG_1);
    for (int n=0; n < Q.Storage(); ++n) pQ[n] = Q(n+1);
    
    if ( nlhs > 1 )
        LHS_ARG_2 = mxCreateLogicalScalar(hasConverged);
        
ROBOOP_MEX_FUNC_STOP