#ifdef _MSC_VER
	#pragma warning(disable: 4267)
	#pragma warning(disable: 4244)
	#pragma warning(disable: 4996)
#endif

#include "utility.h"

// Standard library includes
#include <iostream>
#include <sstream>
#include <exception>
using namespace std;

// Required MATLAB includes
#include "mex.h"

// Required CGAL includes
#include "CGALPreqs.h"

#define COMMAND			prhs[0]
#define POINTCLOUD		prhs[1]
#define NUMNEIGHBORS	prhs[1]
#define QUERYPOINT		prhs[2]
#define FUNCHANDLE		prhs[3]

// Globals
bool g_bLibraryIsInit = false;
Tree* g_Tree;

// Function prototypes
void InitLibrary(void);
string GetCommand(int, const mxArray*);
void CheckInputs(std::string, int, const mxArray* []);
void CheckDistanceFunction(const mxArray*, const char* funcname);
void InitTree(const mxArray*);
void NearestNeighbor(const int numNeighbors, const mxArray*, mxArray*, mxArray**);
void MyExitFcn(void);
void my_failure_handler(const char *type, const char *expr, const char* file, int line, const char* msg);

// Entry point
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	if (!g_bLibraryIsInit) InitLibrary();

	// Get command type
	string type = GetCommand(nrhs, COMMAND);
	CheckInputs(type, nrhs, prhs);

	SSWITCH( type )
	{
		case evInit:
		{
			InitTree( POINTCLOUD );
			break;
		}
		case evSearch:
		{
			NearestNeighbor( mxGetScalar(NUMNEIGHBORS), QUERYPOINT, const_cast<mxArray*>(FUNCHANDLE), plhs );
			break;
		}
		case evReset:
		{
			delete g_Tree;
            g_Tree = NULL;
			mexPrintf("Kd-tree has been deleted successfully.\n");
			break;
		}
		default:
		{
			mexErrMsgIdAndTxt("SpatialSearch:badCommand","This should never have happened. Check outside to see if the sky falling.");
			break;
		}
	}

	return;
}

void InitLibrary(void)
{
	InitializeStringValueMap(); // Initialize string/switch map

	if ( mexAtExit(MyExitFcn) ) mexPrintf("Error! myExitFcn not registered properly");
	
	CGAL::set_error_handler(my_failure_handler);

	g_bLibraryIsInit = true;
}

std::string GetCommand(int nrhs, const mxArray* mcmd)
{
	// Check for minimum number of inputs
	if ( !(mxIsChar( mcmd )) ) {
		mexErrMsgIdAndTxt("SpatialSearch:badCommand","First argument must specify the operation of the library.");
	}
	char* ccmd = mxArrayToString(mcmd);
	std::string scmd(ccmd);
	mxFree(ccmd);

	if (!IsValidCommand(scmd)) mexErrMsgIdAndTxt("SpatialSearch:badCommand","Command, %s,does not exist.", scmd.c_str());

	return scmd;
}

void CheckInputs(std::string cmd, int nrhs, const mxArray* prhs[])
{

	SSWITCH( cmd )
	{
		case evInit:
		{
			if (--nrhs != 1) 
				mexErrMsgIdAndTxt("SpatialSearch:initialization","Initialization requires a point cloud to be specified as the second argument.");
			if ( !mxIsNumeric( POINTCLOUD ) || !mxIsDouble( POINTCLOUD ) ) 
				mexErrMsgIdAndTxt("SpatialSearch:initialization","Point cloud should be a (num dimensions)x(num points) array");
			break;
		}
		case evSearch:
		{
			if (--nrhs != 3) 
				mexErrMsgIdAndTxt("SpatialSearch:search","Search requires specification of the number of neighbors, query point and a function handle to calculate point distances.");
			if ( !mxIsNumeric( NUMNEIGHBORS ) || !mxIsDouble( NUMNEIGHBORS ) ) 
				mexErrMsgIdAndTxt("SpatialSearch:search","Number of neighbors should be a single integer");
			if ( !mxIsNumeric( QUERYPOINT ) || !mxIsDouble( QUERYPOINT ) ) 
				mexErrMsgIdAndTxt("SpatialSearch:search","Query points should be a (num dimensions)x(num points) array");

			CheckDistanceFunction( FUNCHANDLE, "transformed_distance");
			CheckDistanceFunction( FUNCHANDLE, "min_distance_to_rectangle");
			CheckDistanceFunction( FUNCHANDLE, "max_distance_to_rectangle");
			CheckDistanceFunction( FUNCHANDLE, "new_distance");

			// Make sure we have a tree built and the dimensions match
			if (!g_Tree) 
				mexErrMsgIdAndTxt("SpatialSearch:search","Kd-Tree has not yet been created!");
			Tree::iterator it = g_Tree->begin();
			Point p(*it);
			if (p.N != mxGetM(QUERYPOINT)) 
				mexErrMsgIdAndTxt("SpatialSearch:search","Query point not same dimension as point cloud!");
			
			break;
		}
		case evReset:
		{
			break;
		}
		default:
		{
			mexErrMsgIdAndTxt("SpatialSearch:badCommand","This should never have happened. Check outside to see if the sky falling.");
			break;
		}
	}
}

void CheckDistanceFunction(const mxArray* handle, const char* funcname)
{
	if (mxGetFieldNumber(handle, funcname) == -1) 
		mexErrMsgIdAndTxt("SpatialSearch:invalidInput", "%s not defined in the function handle structure.", funcname);
	else
		if ( mxGetClassID( mxGetField(handle, 0, funcname) ) != mxFUNCTION_CLASS )
			mexErrMsgIdAndTxt("SpatialSearch:invalidInput", "%s is not a function handle.", funcname);
}

void InitTree(const mxArray* data)
{
	if ( g_Tree != NULL ) mexErrMsgIdAndTxt("SpatialSearch:initialization","Tree has already been initialized. To re-initialize, reset the data structure first.");

	int nDim = mxGetM(data); int nSamples = mxGetN(data);
	mexPrintf("Building kd-tree data structure with %d samples...", nSamples);

	// Create a new tree ...
	g_Tree = new Tree;

	// .. and fill it with data
	double* ptr = mxGetPr(data);
	for (int n=0; n < nSamples; ++n) {
		g_Tree->insert( Point(nDim, n+1, ptr + (n * nDim)) );
	}

	mexPrintf("%d samples added.\n", g_Tree->size());
}

void NearestNeighbor(int numNeighbors, const mxArray* queryPoint, mxArray* funcHandle, mxArray** plhs)
{
	// Create query point
	double* pQueryPoint = mxGetPr(queryPoint);
	int dim = mxGetM(queryPoint);
	Point pt(dim, 0, pQueryPoint);

	// .. and do search
	MatlabDistance distFunc(funcHandle);
	NeighborSearch search(*g_Tree, pt, numNeighbors, 0.0, true, distFunc);

	// Pull out results
	plhs[0] = mxCreateDoubleMatrix(dim, numNeighbors, mxREAL);
	plhs[1] = mxCreateDoubleMatrix(1, numNeighbors, mxREAL);
	plhs[2] = mxCreateDoubleMatrix(1, numNeighbors, mxREAL);
	double *pts = mxGetPr(plhs[0]), *distances = mxGetPr(plhs[1]), *ids = mxGetPr(plhs[2]);

	int n=0;
	for(NeighborSearch::iterator it = search.begin(); it != search.end(); it++){
		for (int k=0; k < dim; ++k) pts[k + (n * dim)] = (it->first).data[k];
		distances[n] = it->second;
		ids[n] = (it->first).id;
		n++;
	}
}

void MyExitFcn() 
{
	if (g_Tree) {
		delete g_Tree;
		g_Tree = NULL;
	}
	mexWarnMsgTxt("mexSpatialSearch library being unloaded.");
}

void my_failure_handler(const char *type, const char *expr, const char* file, int line, const char* msg)
{
    ostringstream out;
	out << "CGAL reported error!" << endl
		<< "Type: " << type << endl
		<< "Expr: " << expr << endl
		<< "File: " << file << endl
		<< "Line: " << line << endl
		<< "Msg: " << msg << endl;
	mexErrMsgIdAndTxt("SpatialSearch:CGAL",out.str().c_str());
}

