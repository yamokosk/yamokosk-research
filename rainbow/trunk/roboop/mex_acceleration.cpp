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
    
	if (nrhs != 6) ERROR_MSG(INVALID_NUM_ARGS, "Expecting six arguments.");
	
	ColumnVector q = NMArrayFromMxArray( RHS_ARG_2 );
	ColumnVector qp = NMArrayFromMxArray( RHS_ARG_3 );
	ColumnVector tau_cmd = NMArrayFromMxArray( RHS_ARG_4 );
	ColumnVector Fext = NMArrayFromMxArray( RHS_ARG_5 );
	ColumnVector Next = NMArrayFromMxArray( RHS_ARG_6 );
	
	/* HACK ALERT! I am too lazy to change the Roboop source and recompile, but
	 * in dynamics.cpp, there is an if-statement checking whether the length of
	 * q and qp is equal to the total number of dof. So I am just going to
	 * recreate new vectors that have the correct lengths. But this should be
	 * addressed as a bug in Roboop. */
	int na = robj.get_available_dof();
	if ( na < dof )
	{
		// Create new input vectors of the right length and fill them with the appropriate data
		ColumnVector qn(dof), qpn(dof), taun_cmd(dof);
		qn = 0.; qpn = 0.; taun_cmd = 0.;
		
		int j = 1;
		for (int n=1; n <= dof; ++n)
		{
			if ( robj.links[n].get_immobile() )
			{
				qn(n) = q_fixed(n);
			} else {
				qn(n) = q(j);
				qpn(n) = qp(j);
				taun_cmd(n) = tau_cmd(j);
				j++;
			}
		}
		// Calling ROBOOP function
		ColumnVector qpp = robj.acceleration(qn,qpn,taun_cmd,Fext,Next);
		
		// Setting output
		ColumnVector qpp_out(na);
		qpp_out = 0.; j = 1;
		
		for (int n=1; n <= dof; ++n)
		{
			if ( robj.links[n].get_immobile() )
			{
				qpp_out(j) = qpp(n);
				j++;
			}
		}
		
		LHS_ARG_1 = mxArrayFromNMArray(qpp_out);
		
	} else {
		// Calling ROBOOP function
		ColumnVector qpp = robj.acceleration(q,qp,tau_cmd,Fext,Next);

		// Setting output
		LHS_ARG_1 = mxArrayFromNMArray(qpp);
	}    
        
ROBOOP_ROBOT_MEX_FUNC_STOP