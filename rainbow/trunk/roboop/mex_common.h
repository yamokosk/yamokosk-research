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

// Required matlab include
#include "mex.h"

// Roboop includes
#include "robot.h"
#include "newmatio.h"

#ifdef use_namespace
using namespace ROBOOP;
using namespace RBD_LIBRARIES;
#endif

// System includes
#include <string>
#include <sstream>
#include <stdexcept>

#include "mex_error.h"

// DEFINES
#define DUMP_ME(x, fmt) printf("%s:%u: %s=" fmt, __FILE__, __LINE__, #x, x)

// Convience defines for the LHS and RHS Matlab arguments
#define RHS_ARG_2   prhs[1]
#define RHS_ARG_3   prhs[2]
#define RHS_ARG_4   prhs[3]
#define RHS_ARG_5   prhs[4]
#define RHS_ARG_6   prhs[5]
#define RHS_ARG_7   prhs[6]
#define RHS_ARG_8   prhs[7]
#define RHS_ARG_9   prhs[8]
#define RHS_ARG_10  prhs[9]

#define LHS_ARG_1   plhs[0]
#define LHS_ARG_2   plhs[1]
#define LHS_ARG_3   plhs[2]
#define LHS_ARG_4   plhs[3]
#define LHS_ARG_5   plhs[4]
#define LHS_ARG_6   plhs[5]
#define LHS_ARG_7   plhs[6]
#define LHS_ARG_8   plhs[7]
#define LHS_ARG_9   plhs[8]

/* The ROBOT_MEX_FUNC_START and ROBOT_MEX_FUNC_STOP macros are here to make
 * the wrapping of all the RoboOp Robot class functions easier. The Start macro 
 * does two important things. 1. It takes in the initrobot matrix which describes
 * the robot and creates a Robot object for use in the wrapping function.
 * 2. If there are any immobile joints that were specified in the conf file,
 * it sets there values here so member functions return expected values.
 */
#define ROBOOP_ROBOT_MEX_FUNC_START													\
	void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])	\
	{																				\
		try {																		\
			if ( !mxIsStruct( prhs[0] ) )											\
				ERROR_MSG(INVALID_ARG, "Initrobot structure was not defined");		\
																					\
			int initRobotFieldNum, baseTransFieldNum, toolTransFieldNum;			\
																					\
			if ( (initRobotFieldNum = mxGetFieldNumber(prhs[0], "initrobot")) < 0 )	\
				ERROR_MSG(INVALID_ARG, "initrobot field not specified");			\
																					\
			if ( (baseTransFieldNum = mxGetFieldNumber(prhs[0], "T_f_base")) < 0 )	\
				ERROR_MSG(INVALID_ARG, "T_f_base field not specified");				\
																					\
			if ( (toolTransFieldNum = mxGetFieldNumber(prhs[0], "T_EE_tool")) < 0 )	\
				ERROR_MSG(INVALID_ARG, "T_EE_tool field not specified");			\
																					\
			mxArray *mxInitRobot = mxGetFieldByNumber(prhs[0], 0, initRobotFieldNum);	\
																					\
			int dof = mxGetM( mxInitRobot );										\
			if ( mxGetN( mxInitRobot ) != 24 )										\
				ERROR_MSG(INVALID_ARG, "Initrobot matrix has incorrect number of columns");	\
																					\
			Matrix mxRobotParams = NMArrayFromMxArray( mxInitRobot );				\
			ColumnVector q_fixed = mxRobotParams.Column(24);						\
			Matrix initrobot = mxRobotParams.Columns(1,23);							\
			Robot robj( initrobot );												\
																					\
			mxArray *mxBaseTrans = mxGetFieldByNumber(prhs[0], 0, baseTransFieldNum);	\
			robj.T_f_base = NMArrayFromMxArray(mxBaseTrans);						\
			mxArray *mxToolTrans = mxGetFieldByNumber(prhs[0], 0, toolTransFieldNum);	\
			robj.T_6_tool = NMArrayFromMxArray(mxToolTrans);						\
																					\
			for (int nji=1; nji <= robj.get_dof(); ++nji) {							\
				if ( robj.links[nji].get_immobile() ) {								\
					robj.set_q(q_fixed(nji), nji);									\
				}																	\
			}																		

#define ROBOOP_ROBOT_MEX_FUNC_STOP                                                  \
        } catch(Exception) {                                                        \
            std::ostringstream msg;                                                 \
            msg << "Newmat error: " << Exception::what() << std::endl      			\
				<< "Should a column vector be a row vector? Or vice-versa?";		\
            ERROR_MSG(NEWMAT_ERROR, msg.str().c_str());								\
        } catch (const std::runtime_error& e) {                                     \
            std::ostringstream msg;                                                 \
            msg << "Runtime error: " << e.what();									\
            ERROR_MSG(RUNTIME_ERROR, msg.str().c_str());							\
		}																			\
    }
        
/* The following are just a collection of functions to ease the conversions to
 * and from Newmat and Matlab arrays.
 */
#define COLUMN_VECTOR	0
#define ROW_VECTOR		1

#define ASSERT_VECTOR_TYPE(mxObj,nmType)											\
	if ( (nmType == COLUMN_VECTOR) && (mxGetN(mxObj) > 1) ) {						\
		ERROR_MSG(INVALID_ARG, "Expecting column vector for argument " #mxObj);		\
	}																				\
	if ( (nmType == ROW_VECTOR) && (mxGetM(mxObj) > 1) ) {							\
		ERROR_MSG(INVALID_ARG, "Expecting row vector for argument " #mxObj);		\
	}
 
#define _MX(i,j,off) MX[(j)*off+i]

void MxArrayToNMMatrix(Matrix& T, const double *MX, int nr, int nc)
{
	for (int r=0; r<nr; ++r) {
		for (int c=0; c<nc; ++c) T(r+1,c+1) = _MX(r,c,nr);
	}
}

void NMMatrixToMxArray(double *MX, const Matrix& T, int nr, int nc)
{
	for (int r=0; r<nr; ++r) {
		for (int c=0; c<nc; ++c) _MX(r,c,nr) = T(r+1,c+1);
	}
}

mxArray* mxArrayFromNMArray(const Matrix& nmMatrix)
{
	mxArray* mlMatrix = mxCreateDoubleMatrix(nmMatrix.Nrows(), nmMatrix.Ncols(), mxREAL);
	NMMatrixToMxArray(mxGetPr(mlMatrix), nmMatrix, nmMatrix.Nrows(), nmMatrix.Ncols());
	return mlMatrix;
}

ReturnMatrix NMArrayFromMxArray(const mxArray* mlMatrix)
{
	int rows = mxGetM(mlMatrix);
	int cols = mxGetN(mlMatrix);
	
	if ( rows == 1 ) {
		// Row vector
		RowVector out(cols);
		MxArrayToNMMatrix(out, mxGetPr(mlMatrix), rows, cols);
		out.Release();
		return out;
	} else if ( cols == 1 ) {
		// Column vector
		ColumnVector out(rows);
		MxArrayToNMMatrix(out, mxGetPr(mlMatrix), rows, cols);
		out.Release();
		return out;
	} else {
		// Plain-jane matrix
		Matrix out(rows, cols);
		MxArrayToNMMatrix(out, mxGetPr(mlMatrix), rows, cols);
		out.Release();
		return out;
	}
}