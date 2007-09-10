#include "planner\roadmap.h"

void Roadmap::construct(const Configuration& q1, const Configuration& q2, int N)
{
	// PRM construction algorithm
	/*do {
		c = model.randomConfiguration();
		// Add c to V
		// G.addNode(c);
		Nc = neighbors(c,V);
		
		for (int n=0; n < Nc.size(); ++n) {
			if ( !ds.same_connected_component(c, Nc[n]) ) {
				if ( lpm(c, Nc[n]) )
					G.addEdge(c,Nc[n],weight);
			}
		}
	} while ( !ds.same_connected_component(q1,q2) || G.numNodes < (N + 2) )*/
	return;
}