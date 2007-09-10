#ifndef DISJOINTSET_HEADER_FILE
#define DISJOINTSET_HEADER_FILE

#include <boost/numeric/ublas/matrix_sparse.hpp>

struct disjointset
{
	disjointset(int n);
	bool same_connected_component(int a, int b) {return false;}
	void union_set(int a, int b) {return;}
};

#endif