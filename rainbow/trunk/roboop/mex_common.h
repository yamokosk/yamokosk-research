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
#define NUM_ROBOT_FIELDS 5
const char *robot_field_names[] = {"name", "DH", "dof", "available_dof", "links"};
#define NUM_LINK_FIELDS 18
const char *link_field_names[] = {"joint_type", "theta", "d", "a", "alpha", "q", "theta_min", "theta_max",
                                  "joint_offset", "r", "p", "m", "Im", "Gr", "B", "Cf", "I", "immobile"};

#define DUMP_ME(x, fmt) printf("%s:%u: %s=" fmt, __FILE__, __LINE__, #x, x)

#define ROBOOP_MEX_FUNC_START														\
	void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])	\
	{																				\
		try {																		\
			if ( !mxIsNumeric( prhs[0] ) )											\
				ERROR_MSG(INVALID_ARG, "Initrobot matrix was not defined");			\
																					\
			int dof = mxGetM( prhs[0] );											\
			if ( mxGetN( prhs[0] ) != 24 )											\
				ERROR_MSG(INVALID_ARG, "Initrobot matrix has incorrect number of columns");	\
																					\
			Matrix fromMatlab = NMArrayFromMxArray( prhs[0] );						\
			ColumnVector q_fixed = fromMatlab.Column(24);							\
			Matrix initrobot = fromMatlab.Columns(1,23);							\
			Robot robj( initrobot );												\
																					\
			for (int nji=1; nji <= 6; ++nji) {										\
				if ( robj.links[nji].get_immobile() ) {								\
					mexPrintf("found immobile joint at %d\n", nji);					\
					robj.set_q(q_fixed(nji), nji);									\
				}																	\
			}																		\
			nrhs--;

#define ROBOOP_MEX_FUNC_STOP                                                        \
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
        
#define RHS_ARG_1   prhs[1]
#define RHS_ARG_2   prhs[2]
#define RHS_ARG_3   prhs[3]
#define RHS_ARG_4   prhs[4]
#define RHS_ARG_5   prhs[5]
#define RHS_ARG_6   prhs[6]
#define RHS_ARG_7   prhs[7]
#define RHS_ARG_8   prhs[8]
#define RHS_ARG_9   prhs[9]

#define LHS_ARG_1   plhs[0]
#define LHS_ARG_2   plhs[1]
#define LHS_ARG_3   plhs[2]
#define LHS_ARG_4   plhs[3]
#define LHS_ARG_5   plhs[4]
#define LHS_ARG_6   plhs[5]
#define LHS_ARG_7   plhs[6]
#define LHS_ARG_8   plhs[7]
#define LHS_ARG_9   plhs[8]

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