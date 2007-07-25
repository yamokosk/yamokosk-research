#ifndef DISTANCE_HEADER_FILE
#define DISTANCE_HEADER_FILE

#include "mex.h"

struct MatlabDistance
{
	typedef Point Query_item;

	MatlabDistance() : pdata(0) {};
	MatlabDistance(const mxArray* ptr) : pdata(ptr) {};

	double transformed_distance(const Point& p1, const Point& p2) const
	{
		// Create LHS and RHS arguments
		mxArray* prhs[3] = {0}; 
		mxArray* plhs[1] = {0}; 
		
		prhs[0] = mxGetField(pdata,0,"transformed_distance");
		prhs[1] = const_cast<Point&>(p1).CreateMArray();
		prhs[2] = const_cast<Point&>(p2).CreateMArray();

		// Call user matlab function
		mexCallMATLAB(1,plhs,3,prhs,"feval"); 
		if (plhs[0] == NULL) 
			mexErrMsgIdAndTxt("SpatialSearch:distanceCalc","User specified transformed_distance did not return a value.");
	        
		// Clean up and return the value
		for (int n=1; n<3; ++n) mxDestroyArray(prhs[n]);
		return (double)mxGetScalar(plhs[0]);
	}

	template <class TreeTraits>
	double min_distance_to_rectangle(const Point& p, const CGAL::Kd_tree_rectangle<TreeTraits>& b) const
	{
		return distance_to_rectangle("min_distance_to_rectangle", p, b);	
	}

	template <class TreeTraits>
	double max_distance_to_rectangle(const Point& p, const CGAL::Kd_tree_rectangle<TreeTraits>& b) const
	{
		return distance_to_rectangle("max_distance_to_rectangle", p, b);	
	}

	template <class TreeTraits>
	double distance_to_rectangle(const char* funcName, const Point& p, const CGAL::Kd_tree_rectangle<TreeTraits>& b) const
	{
		// Grab CGAL data and make it amenable to Matlab
		mxArray* lb = mxCreateDoubleMatrix(b.dimension(),1,mxREAL);
		double* plb = mxGetPr(lb);
		for (int n=0; n < b.dimension(); ++n) plb[n] = b.min_coord(n);

		mxArray* ub = mxCreateDoubleMatrix(b.dimension(),1,mxREAL);
		double* pub = mxGetPr(ub);
		for (int n=0; n < b.dimension(); ++n) pub[n] = b.max_coord(n);

		// Create LHS and RHS arguments
		mxArray* prhs[4] = {0}; 
		mxArray* plhs[1] = {0}; 
 
		prhs[0] = mxGetField(pdata,0,funcName);
		prhs[1] = const_cast<Point&>(p).CreateMArray();
		prhs[2] = lb;
		prhs[3] = ub;

		// Call user matlab function
		mexCallMATLAB(1,plhs,4,prhs,"feval"); 
		if (plhs[0] == NULL) 
			mexErrMsgIdAndTxt("SpatialSearch:distanceCalc","User specified %s did not return a value.", funcName);
	    
		for (int n=1; n<4; ++n) mxDestroyArray(prhs[n]);
		return (double)mxGetScalar(plhs[0]);
	}

	double new_distance(double& dist, double old_off, double new_off, int cutting_dimension) const
	{
		// Create LHS and RHS arguments
		mxArray* prhs[5] = {0}; 
		mxArray* plhs[1] = {0}; 
    
		prhs[0] = mxGetField(pdata,0,"new_distance");
		prhs[1] = mxCreateDoubleScalar(dist);
		prhs[2] = mxCreateDoubleScalar(old_off);
		prhs[3] = mxCreateDoubleScalar(new_off);
		prhs[4] = mxCreateDoubleScalar(cutting_dimension);

		// Call user matlab function
		mexCallMATLAB(1,plhs,5,prhs,"feval"); 
		if (plhs[0] == NULL)
			mexErrMsgIdAndTxt("SpatialSearch:distanceCalc","User specified new_distance did not return a value.");
	    
		for (int n=1; n<5; ++n) mxDestroyArray(prhs[n]);
		return (double)mxGetScalar(plhs[0]);
	}
	
	double transformed_distance(double d) const { return d*d; }
	double inverse_of_transformed_distance(double d) { return std::sqrt(d); }

	const mxArray* pdata;
};


#endif