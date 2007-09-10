#ifndef CGALPREQS_HEADER_FILE
#define CGALPREQS_HEADER_FILE

// CGAL includes
#include <CGAL/basic.h>
#include <CGAL/Search_traits.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/assertions.h>
#include "spatialsearch\configuration.h"
#include "spatialsearch\distance.h"
using namespace CGAL;

typedef CGAL::Search_traits<double, Configuration, const double*, ConstructCoordIterator> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits,MatlabDistance> NeighborSearch;
typedef NeighborSearch::Tree Tree;

#endif