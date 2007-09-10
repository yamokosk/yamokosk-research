#ifndef ROADMAP_HEADER_FILE
#define ROADMAP_HEADER_FILE

#include "planner.h"
#include <boost/graph/adjacency_list.hpp>

struct Roadmap
{
	typedef boost::adjacency_list<boost::listS, boost::listS, boost::bidirectionalS> Graph;
	// PRM construction algorithm
		/*for (int kk = 0; kk < numSamples; ++kk) {
			ii = indices(kk);
			c = S(:,ii);
			[nbhrs,dist,ind] = searchKDTree(c,numNhbrs);
			    
			for n = 1:length(ind) {
				if ( !ds.same_connected_component(ii,ind(n)) ) {
					if lpm(c,nbhrs(:,n),opts) {
						G(ii,ind(n)) = dist(n); // Record new edge
						ds.union_set(ii,ind(n)); // Update connect components
					}
				}
			}
		}*/
	void construct(const Configuration& q1, const Configuration& q2, int N);

	disjointset ds;
	Graph G;
};

#endif