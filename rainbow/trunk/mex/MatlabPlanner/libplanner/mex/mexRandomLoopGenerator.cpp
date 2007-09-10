// Required MATLAB includes
#include "mex.h"

#include "planner\rlg.h"

#define COMMAND			prhs[0]
#define POINTCLOUD		prhs[1]
#define NUMNEIGHBORS	prhs[1]
#define QUERYPOINT		prhs[2]
#define FUNCHANDLE		prhs[3]

// Globals
bool g_bLibraryIsInit = false;
RandomLoopGenerator* rlg;

// Function prototypes
void InitLibrary(void);
void CheckInputs(std::string, int, const mxArray* []);

/* RLG - Random loop generator for the GatorRay system
 *
 *	q = rlg(N, target) will generate N quasi-random states about a
 *	desired point and orientation specified in target. target must
 *	be a 4x4 transformation matrix describing this desired point.
 *
 *	q = rlg(..., opts) allows the user to modify the parameters which
 *	are used to generate the quasi-random states. 
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (!g_bLibraryIsInit) InitLibrary();


}

void InitLibrary(void)
{

}

void CheckInputs(std::string, int, const mxArray* [])
{

}