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

#include <vector>

#define NUM_ROBOT_FIELDS 7
const char *robot_field_names[] = {"name", "gravity", "DH", "dof", "available_dof", "immobile_joints", "links"};
#define NUM_LINK_FIELDS 18
const char *link_field_names[] = {"joint_type", "theta", "d", "a", "alpha", "q", "theta_min", "theta_max",
                                  "joint_offset", "r", "p", "m", "Im", "Gr", "B", "Cf", "I", "immobile"};
								  
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{                                                                            
	try {                                                 
		char* conffile = mxArrayToString(prhs[0]);
		char* robotname = mxArrayToString(prhs[1]);
        
		//mexPrintf("conffile = %s", conffile);
		Robot robj(conffile, robotname);

		// Create link structure
		mxArray *links = mxCreateStructMatrix(robj.get_dof(), 1, NUM_LINK_FIELDS, link_field_names);
		double *ptr = NULL;
		Link *rlinks = robj.links;
		std::vector<double> immobileID;
		for (int i=1; i <= robj.get_dof(); ++i) 
		{
			// Return the joint type.
			mxSetField(links, i-1, "joint_type", mxCreateDoubleScalar( rlinks[i].get_joint_type() ));
            
			// Return theta.
			mxSetField(links, i-1, "theta", mxCreateDoubleScalar( rlinks[i].get_theta() ));
            
			//!< Return d.
			mxSetField(links, i-1, "d", mxCreateDoubleScalar( rlinks[i].get_d() ));
            
			// Return a.
			mxSetField(links, i-1, "a", mxCreateDoubleScalar( rlinks[i].get_a() ));
            
			// Return alpha.
			mxSetField(links, i-1, "alpha", mxCreateDoubleScalar( rlinks[i].get_alpha() ));
            
			//!< Return q
			mxSetField(links, i-1, "q", mxCreateDoubleScalar( rlinks[i].get_q() ));
            
			// Return theta_min.
			mxSetField(links, i-1, "theta_min", mxCreateDoubleScalar( rlinks[i].get_theta_min() ));
            
			// Return theta_max.
			mxSetField(links, i-1, "theta_max", mxCreateDoubleScalar( rlinks[i].get_theta_max() ));
            
			// Return joint_offset.
			mxSetField(links, i-1, "joint_offset", mxCreateDoubleScalar( rlinks[i].get_joint_offset() ));
            
			// Return r.
			mxArray *mr = mxArrayFromNMArray( rlinks[i].get_r() );
			//mxArray *mr = mxCreateDoubleMatrix(1,3,mxREAL); ptr = mxGetPr(mr);
			//ColumnVector r = rlinks[i].get_r();
			//for (int n=0; n < 3; ++n) ptr[n] = r(n+1);
			mxSetField(links, i-1, "r", mr);
            
			// Return p.
			mxArray *mp = mxArrayFromNMArray( rlinks[i].get_p() );
			//mxArray *mp = mxCreateDoubleMatrix(1,3,mxREAL); ptr = mxGetPr(mp);
			//ColumnVector p = rlinks[i].get_p();
			//for (int n=0; n < 3; ++n) ptr[n] = p(n+1);
			mxSetField(links, i-1, "p", mp);
            
			// Return m.
			mxSetField(links, i-1, "m", mxCreateDoubleScalar( rlinks[i].get_m() ));
            
			// Return Im.
			mxSetField(links, i-1, "Im", mxCreateDoubleScalar( rlinks[i].get_Im() ));
            
			// Return Gr.
			mxSetField(links, i-1, "Gr", mxCreateDoubleScalar( rlinks[i].get_Gr() ));
            
			// Return B.
			mxSetField(links, i-1, "B", mxCreateDoubleScalar( rlinks[i].get_B() ));
            
			// Return Cf.
			mxSetField(links, i-1, "Cf", mxCreateDoubleScalar( rlinks[i].get_Cf() ));
            
			// Return I.
			mxArray *mI = mxArrayFromNMArray( rlinks[i].get_I() );
			//mxArray *mI = mxCreateDoubleMatrix(3,3,mxREAL);
			//Matrix I = rlinks[i].get_I();
			//NMMatrixToMxArray(mxGetPr(mI), I, 3, 3);
			mxSetField(links, i-1, "I", mI);
            
			// Return immobile.
			mxSetField(links, i-1, "immobile", mxCreateLogicalScalar( rlinks[i].get_immobile() ));
			if ( rlinks[i].get_immobile() )
			{
				immobileID.push_back((double)i);
			}
		}
        
		// Create robot structure
		LHS_ARG_1 = mxCreateStructMatrix(1, 1, NUM_ROBOT_FIELDS, robot_field_names);
        
		// Set name
		mxSetField(LHS_ARG_1, 0, "name", mxCreateString(robotname) );
		
		// Set gravity
		mxSetField(LHS_ARG_1, 0, "gravity", mxArrayFromNMArray( robj.gravity ) );
		
		// Return DH type
		mxSetField(LHS_ARG_1, 0, "DH", mxCreateLogicalScalar( robj.get_DH() ));
        
		// Return DOF
		mxSetField(LHS_ARG_1, 0, "dof", mxCreateDoubleScalar( (double)robj.get_dof() ));
        
		// Return available DOF
		mxSetField(LHS_ARG_1, 0, "available_dof", mxCreateDoubleScalar( robj.get_available_dof() ));
        
		// Return IDs of immobile joints
		int nImmobileJnts = (int)immobileID.size();
		if ( nImmobileJnts > 0 )
		{
			mxArray *tempArray = mxCreateDoubleMatrix(1,nImmobileJnts,mxREAL);
			memcpy( mxGetPr(tempArray), &immobileID[0], nImmobileJnts*sizeof(double) );
			mxSetField(LHS_ARG_1, 0, "immobile_joints", tempArray);
		} else {
			mxSetField(LHS_ARG_1, 0, "immobile_joints", mxCreateDoubleMatrix(0,0, mxREAL));
		}
		
		// Return links
		mxSetField(LHS_ARG_1, 0, "links", links);
		
		// Free allocated stuff
		mxFree( conffile );
		mxFree( robotname );
	
	} catch(Exception) {
		std::ostringstream msg;
		msg << mexFunctionName() << ":: " << Exception::what();
		mexErrMsgTxt(msg.str().c_str());
	} catch (const std::runtime_error& e) {
		std::ostringstream msg;
		msg << mexFunctionName() << ":: " << e.what();
		mexErrMsgTxt(msg.str().c_str());
	} catch (...) {
		std::ostringstream msg;
		msg << mexFunctionName() << ":: Unknown failure during operation";
		mexErrMsgTxt(msg.str().c_str());
	}                                                                           
}
   
