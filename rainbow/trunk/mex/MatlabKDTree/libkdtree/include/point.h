#ifndef POINT_HEADER_FILE
#define POINT_HEADER_FILE

#include "mex.h"

struct Point
{ 
	Point(unsigned int n, unsigned int ID, const double* ptr) : N(n), id(ID)
	{
		data = new double[N];
		for (unsigned int ii=0; ii < N; ++ii) data[ii] = ptr[ii];
	}
	Point(const Point &p)
	{
		this->data = new double[p.N];
		this->N = p.N;
		this->id = p.id;
		for (unsigned int ii=0; ii < N; ++ii) data[ii] = p.data[ii];
	}
	~Point() {if (data) delete [] data;}

	Point& operator=(const Point& p)
	{
		if (this != &p) {
			delete [] data;
			data = new double[p.N];
			this->N = p.N;
			this->id = p.id;
			for (int ii=0; ii<p.N; ++ii)	data[ii] = p.data[ii];
		}
		return *this;
	}

	bool operator==(const Point& p) const
	{
		if (this->id == p.id) return true;
		if (this->N != p.N) return false;

		for (unsigned int ii=0; ii < N; ++ii) {
			if (data[ii] != p.data[ii]) return false;
		}
		return true;
	}
	bool operator!=(const Point& p) const
	{
		return !(*this == p);
	}

	mxArray* CreateMArray()
	{
		mxArray* marray = mxCreateDoubleMatrix(N, 1, mxREAL);
		double* ptr = mxGetPr(marray);
		for (int ii=0; ii<N; ++ii) ptr[ii] = data[ii];
		return marray;
	}

	double *data;
	unsigned int N;
	unsigned int id;
};

struct ConstructCoordIterator {
	const double* operator()(const Point& p) const 
	{ 
		return static_cast<const double*>(p.data); 
	}

	const double* operator()(const Point& p, int)  const
	{ 
		return static_cast<const double*>(p.data + p.N); 
	}
};

namespace CGAL {
	template <>
	struct Kernel_traits<Point> {
		struct Kernel {
			typedef double FT;
			typedef double RT;
		};
	};
}

#endif