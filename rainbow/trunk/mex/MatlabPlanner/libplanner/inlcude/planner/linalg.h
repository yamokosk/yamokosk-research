#ifndef LINEAR_ALGEBRA_UTILITY_HEADER_FILE
#define LINEAR_ALGEBRA_UTILITY_HEADER_FILE

//#define PI 3.14159265
#include "newmat.h"
#include <cmath>
#define NO_DATA 99999
#ifndef M_PI 
	#define M_PI 3.1415926535897932384626433832795
#endif

ReturnMatrix transl(double x, double y, double z);
ReturnMatrix transl(const ColumnVector& p);

//ReturnMatrix rotx(double t);
//ReturnMatrix roty(double t);
//ReturnMatrix rotz(double t);

ReturnMatrix inv_tmatrix(const Matrix& t);
void acos2(const double& x, double& ang1, double& ang2);
void asin2(const double& x, double& ang1, double& ang2);

#endif