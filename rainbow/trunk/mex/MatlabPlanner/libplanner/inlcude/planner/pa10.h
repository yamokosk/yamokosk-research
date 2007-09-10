#ifndef PA10_HEADER_FILE
#define PA10_HEADER_FILE

#include "robot.h"

struct ClosedLoopParams
{
	double a71;
	double S7;
	double S1;
	double alp71;
	double th7;
	double gam1;
};

class PA10 : public Robot
{
public:
	PA10(const std::string & filename, const std::string & robotName);
	virtual ~PA10();
	
	//ReturnMatrix kine_pa10(void) const;
	//ReturnMatrix kine_pa10(int n) const;
	virtual ReturnMatrix inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge);
	
private:
	ReturnMatrix inv_kin_pa10(const Matrix& T_1_tool, ColumnVector& good_soln, int* nGoodSolutions);
	void close_loop(const Matrix& T_1_tool, ClosedLoopParams& p);
	bool fix_angle(const int& jn, double& ang);
	bool solve_trig(const double& A, const double& B, const double& Din, double& ang1, double& ang2);
	bool solve_pair(const Matrix& A, const ColumnVector& b, ColumnVector& x);
	bool isabout(const double& var, const double& value, const double& tol);	
};

#endif