#include "planner\linalg.h"


ReturnMatrix transl(double x, double y, double z)
{
	SquareMatrix T = IdentityMatrix(4);
	T(1,4) = x;	T(2,4) = y; T(3,4) = z;
	T.release();
	return T;
}

ReturnMatrix transl(const ColumnVector& p)
{
	SquareMatrix T = IdentityMatrix(4);
	T.submatrix(1,3,4,4) = p.submatrix(1,3,1,1);
	T.release();
	return T;
}

/*ReturnMatrix rotx(double t)
{
	double ct = cos(t);	double st = sin(t);
	SquareMatrix T = IdentityMatrix(4);
	T(2,2) = ct;
	T(3,3) = ct;
	T(2,3) = -st;
	T(3,2) = st;
	T.release();
	return T;
}

ReturnMatrix roty(double t)
{
	double ct = cos(t);	double st = sin(t);
	SquareMatrix T = IdentityMatrix(4);
	T(1,1) = ct;
	T(1,3) = st;
	T(3,1) = -st;
	T(3,3) = ct;
	T.release();
	return T;
}

ReturnMatrix rotz(double t)
{
	double ct = cos(t); double st = sin(t);
	SquareMatrix T = IdentityMatrix(4);
	T(1,1) = ct;
	T(1,2) = -st;
	T(2,1) = st;
	T(2,2) = ct;
	T.release();
	return T;
}*/



ReturnMatrix inv_tmatrix(const Matrix& t)
{
	SquareMatrix tinv = IdentityMatrix(4);
	tinv.submatrix(1,3,1,3) = t.submatrix(1,3,1,3).t();
	tinv.submatrix(1,3,4,4) = -tinv.submatrix(1,3,1,3) * t.submatrix(1,3,4,4);
	tinv.release();
	return tinv;
}

void acos2(const double& x, double& ang1, double& ang2)
{
	ang1 = acos(x);
	ang2 = -ang1;
}

void asin2(const double& x, double& ang1, double& ang2)
{
	ang1 = asin(x);
	if (x > 0) ang2 = M_PI - ang1;
	else ang2 = -M_PI - ang1;
}