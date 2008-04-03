/*
mexROBOOP -- A Matlab wrapper of the RoboOp C++ library
Copyright (C) 2008	J.D. Yamokoski

This library is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation; either version 2.1 of the
License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

-------------------------------------------------------------------------------
Revision_history:

2008/03/10: J.D. Yamokoski
	- Created.
-------------------------------------------------------------------------------
*/

#include "mex_common.h"

ROBOOP_ROBOT_MEX_FUNC_START
    
	if (nrhs != 4) ERROR_MSG(INVALID_NUM_ARGS, "Expecting four arguments.");
	
    bool hasConverged = false;
    Matrix Tobj = NMArrayFromMxArray( RHS_ARG_2 );
	int mj = (int)mxGetScalar(RHS_ARG_3);
    int endlink = (int)mxGetScalar(RHS_ARG_4);
    	
    // Calling ROBOOP function
    ColumnVector Q = robj.inv_kin(Tobj, mj, endlink, hasConverged);
	
    // Setting output
	LHS_ARG_1 = mxArrayFromNMArray(Q);
    
    if ( nlhs > 1 )
        LHS_ARG_2 = mxCreateLogicalScalar(hasConverged);
        
ROBOOP_ROBOT_MEX_FUNC_STOP