#ifndef CONFIGURATION_HEADER_FILE
#define CONFIGURATION_HEADER_FILE

#include "mex.h"

struct Configuration
{ 
	Configuration(unsigned int n, unsigned int ID, const double* ptr) : N(n), id(ID)
	{
		data = new double[N];
		for (unsigned int ii=0; ii < N; ++ii) data[ii] = ptr[ii];
	}
	Configuration(const Configuration &p)
	{
		this->data = new double[p.N];
		this->N = p.N;
		this->id = p.id;
		for (unsigned int ii=0; ii < N; ++ii) data[ii] = p.data[ii];
	}
	~Configuration() {if (data) delete [] data;}

	Configuration& operator=(const Configuration& p)
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

	bool operator==(const Configuration& p) const
	{
		if (this->id == p.id) return true;
		if (this->N != p.N) return false;

		for (unsigned int ii=0; ii < N; ++ii) {
			if (data[ii] != p.data[ii]) return false;
		}
		return true;
	}
	bool operator!=(const Configuration& p) const
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
	const double* operator()(const Configuration& p) const 
	{ 
		return static_cast<const double*>(p.data); 
	}

	const double* operator()(const Configuration& p, int)  const
	{ 
		return static_cast<const double*>(p.data + p.N); 
	}
};

namespace CGAL {
	template <>
	struct Kernel_traits<Configuration> {
		struct Kernel {
			typedef double FT;
			typedef double RT;
		};
	};
}

#endif