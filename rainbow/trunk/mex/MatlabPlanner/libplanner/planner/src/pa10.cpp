#include "planner\pa10.h"

#include "planner\linalg.h"
#include <cmath>

PA10::PA10(const std::string & filename, const std::string & robotName) : Robot(filename, robotName) 
{

};

PA10::~PA10() {}; 

/*ReturnMatrix PA10::kine_pa10(void) const
{
	Matrix t = T_f_base * this->kine() * T_6_tool;
	t.Release(); 
	return t;
}

ReturnMatrix PA10::kine_pa10(int n) const
{
	Matrix t = T_f_base * this->kine(n);
	t.release();
	return t;
}*/

/*ReturnMatrix PA10::inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
{
	//SquareMatrix T_1_tool = inv_tmatrix(this->T_f_base) * Tobj;
	//return inv_kin_pa10(T_1_tool, converge);
	//SquareMatrix T_1_6 = inv_tmatrix(this->T_f_base) * Tobj * inv_tmatrix(this->T_6_tool);
	//return Robot::inv_kin(T_1_6, mj, endlink, converge);
	ColumnVector q = Robot::inv_kin(Tobj, mj, endlink, converge);
	std::cout << "T_f_tool:" << std::endl << Tobj << std::endl;
	std::cout << "q" << std::endl << q.t() << std::endl;
	q.release();
	return q;
}*/

ReturnMatrix PA10::inv_kin(const Matrix & Tobj, const int mj, const int endlink, bool & converge)
{
	SquareMatrix T_1_tool = inv_tmatrix(this->T_f_base) * Tobj;
	ColumnVector good_soln(8);
	int numGoodSolutions = 0;

	// Analytic solution to inverse kinematics
	Matrix Q = inv_kin_pa10(T_1_tool, good_soln, &numGoodSolutions);
	
	if (numGoodSolutions > 0) {
		Matrix Qout(6,numGoodSolutions);
		int ptr = 1;
		for (int n=1; n <= 8; ++n) {
			if (good_soln(n)) {
				for (int dof=1; dof <= 6; ++dof) Qout(dof,ptr) = Q(dof,n);
				ptr++;
			}
		}
		Qout.release();
		converge = true;
		return Qout;
	} else {
		converge = false;
		return Matrix();
	}
}

ReturnMatrix PA10::inv_kin_pa10(const Matrix& T_1_tool, ColumnVector& good_soln, int* nGoodSolns)
{
/*	REVERSE_PA10     Reverse position analysis for the PA-10
 *       [ANGLES, GOOD_SOLN] = REVERSE_PA10(ROBOT,T_F_TOOL) performs 
 *       reverse position analysis for the Mitsubishi PA-10C using Crane's 
 *       method. The inputs are:
 *
 *       Robot - Robot object created using the Matlab Robot Toolbox
 *
 *       T_f_tool - Transformation matrix from fixed to the tool frame
 *
 *       REVERSE_PA10 will return a 4x6 matrix called ANGLES which will
 *       contain all possible solutions for the specified input parameters.
 *       Phi-1 is column 1, theta-2 is column 2, theta-3 is column 3, etc.
 *       GOOD_SOLN is a 4x1 vector. Each row of GOOD_SOLN corresponds to a
 *       solution row of ANGLES. If GOOD_SOLN[i] = 0, then the corresponding
 *       ANGLES[i] is not a valid solution. If GOOD_SOLN[i] = 1, then the
 *       corresponding ANGLES[i] is a valid solution.
 *
 *       Author: J.D. Yamokoski
 *       Modified: 5/4/2006
 *
 */
	Matrix angles(6,8);

	/* --------------------------------------------------------------------------
	 * Step 1. Extract DH parameters from robot object
	 * -------------------------------------------------------------------------*/
	double alp12 = this->links[1].get_alpha();
	double a23 = this->links[2].get_a();    
	double alp23 = this->links[2].get_alpha();      
	double alp34 = this->links[3].get_alpha();
	double alp45 = this->links[4].get_alpha();
	double S4 = this->links[4].get_d();
	double alp56 = this->links[5].get_alpha();
	double alp67 = M_PI/2; //?? I remember this being some sort of hack...
	double S6 = this->links[6].get_d();

	// Use the constant mechanism parameters to find cj, sj, cij, and sij.
	double c12 = cos(alp12); double s12 = sin(alp12);
	double c23 = cos(alp23); double s23 = sin(alp23);
	double c34 = cos(alp34); double s34 = sin(alp34);
	double c45 = cos(alp45); double s45 = sin(alp45);
	double c56 = cos(alp56); double s56 = sin(alp56);
	double c67 = cos(alp67); double s67 = sin(alp67);

	// Check passed in values... derivation of the analytic solution below
	// depeneds on a few assumptions!
	if ( s23 > 1e-9 ) throw out_of_range("Alpha_23 should be equal to 0 degrees.");
	if ( c34 > 1e-9 ) throw out_of_range("Alpha_34 should be equal to 90 degrees.");

	/* --------------------------------------------------------------------------
	 * Step 2. Create a virtual closed-loop mechanism so that we can write a
	 * vector loop equation.
	 *
	 *   Vector-loop equation for PA-10:
	 *   S1*S1_vec + a23*a23_vec + S4*S4_vec + S6*S6_vec + S7*S7_vec + a71*a71_vec = 0_vec
	 * ------------------------------------------------------------------------*/
	ClosedLoopParams p;
	close_loop(T_1_tool, p);

	/* --------------------------------------------------------------------------
	 * Step 3. Solve for theta_1 and phi_1
	 *
	 *   There should be two valid solutions for theta_1 (phi_1). Therefore this
	 *   creates the first branch in our tree of solutions.
	 * ------------------------------------------------------------------------*/
	double c7 = cos(p.th7); double s7 = sin(p.th7);
	double c71 = cos(p.alp71); double s71 = sin(p.alp71);
	double X7 = s67*s7;
	double Y7 = -(s71*c67 + c71*s67*c7);
	double Z7 = c71*c67 - s71*s67*c7;
	// Expand z-component of vector loop eqn and collect s1 and c1 terms to get
	//   A*c1 + B*s1 + D = 0
	double A = -S6*Y7 + p.S7*s71;
	double B = -S6*X7 - p.a71;
	double D = p.S1*c12; // Should be equal to zero
	double ang1 = 0, ang2 = 0;
	
	// If the does not work then there is no point in continuing.
	if (!solve_trig(A, B, D, ang1, ang2)) throw out_of_range("Encountered bad solutions at theta_1!");

	//PossibleNumSolns = 4 * numSolutions;
	//angles = zeros(PossibleNumSolns,6);
	//Matrix Q(6,4*2);
	//ColumnVector th1(8), ph1(8), th2(8), th3(8), th4(8), th5(8), th6(8), good_soln(8);
	//ColumnVector th1 << ang1 << ang1 << ang1 << ang1 << ang2 << ang2 << ang2 << ang2;
	ColumnVector th1(8);
	th1.submatrix(1,4,1,1) = ang1; th1.submatrix(5,8,1,1) = ang2;
	good_soln = 1;
	//good_soln = zeros(PossibleNumSolns,1);

	//if ( !fix_angle(1,a1) && !fix_angle(1,a2) ) throw out_of_range("Encountered bad solutions at theta_1!");
	//qout(1) = ( (fabs(a1 - q_actual(1)) < fabs(a2 - q_actual(1))) ? a1 : a2 ); // Choose one closest to q1_actual
	//double th1 = qout(1) + p.gam1;
	//if ( !fix_angle(1,a1) ) for (int n=0; n < 4; ++n) good_soln[0] = false;
	//if ( !fix_angle(1,a2) ) for (int n=5; n < 8; ++n) good_soln[0] = false;
	ang1 = ang1 - p.gam1; ang2 = ang2 - p.gam1;
	if ( !fix_angle(1,ang1) ) good_soln.submatrix(1,4,1,1) = 0;
	if ( !fix_angle(1,ang2) ) good_soln.submatrix(5,8,1,1) = 0;
	angles.submatrix(1,1,1,4) = ang1;
	angles.submatrix(1,1,5,8) = ang2;

	/* -------------------------------------------------------------------------
	 * Solutions for the rest of the joint angles
	 *   The rest of this program consists of a various number of nested FOR and
	 *   IF statements. The purpose of this code is to traverse the entire
	 *   solution tree.
	 *
	 *   The if statments are used to primarily set and check the status of the
	 *   good_soln array. If there is a problem calculating any of the joint
	 *   angles at any part of the tree the problematic joint angle will set the
	 *   appropriate sections of good_soln = 0 which will essentially cancel
	 *   calculation of the tree from that angle forward.
	 * ------------------------------------------------------------------------*/
	for (int th1index = 1; th1index <= 5; th1index=th1index+4)  //[1, 5]
	{
		/*--------------------------------------------------------------------------
		 * Step 4. Solve for theta_3
		 *
		 *   There are two possible solutions for theta_3. Thus creating the second
		 *   branch in the tree of solutions.
		 *------------------------------------------------------------------------*/
		double c1 = cos(th1(th1index)); double s1 = sin(th1(th1index));
		double X1 = s71*s1;
		double Y1 = -(s12*c71 + c12*s71*c1);
		double X71 = X7*c1 - Y7*s1;
		double Y71 = c12*(X7*s1 + Y7*c1) - s12*Z7;
		double Z71 = s12*(X7*s1 + Y7*c1) + c12*Z7;
		A = -S6*X71 - p.S7*X1 - p.a71*c1;
		B = -p.S1*s12 + S6*Y71 + p.S7*Y1 + p.a71*s1*c12;
		double s3 = (pow(A,2) + pow(B,2) - pow(a23,2) - pow(S4,2))/(2*S4*a23);
		
		if ( isabout(fabs(s3), 1, 1e-6) ) s3=1.0;

		if (fabs(s3) > 1.0) {
			//throw out_of_range("Encountered bad solutions at th3.");
			good_soln.submatrix(th1index,th1index+3,1,1) = 0;
			continue;
		}

		asin2(s3, ang1, ang2);
		if ( !fix_angle(3,ang1) ) good_soln.submatrix(th1index,th1index + 1,1,1) = 0;
		if ( !fix_angle(3,ang2) ) good_soln.submatrix(th1index+2,th1index+3,1,1) = 0;
	    //th3.submatrix(th1index,th1index + 1,1,1) = ang1;
	    //th3.submatrix(th1index+2,th1index+3,1,1) = ang2;
		angles.submatrix(3,3,th1index,th1index+1) = ang1;
		angles.submatrix(3,3,th1index+2,th1index+3) = ang2;

		for (int th3index = th1index; th3index <= th1index+2; th3index = th3index + 2) 
		{
			/*----------------------------------------------------------------------
			 * Step 5. Solve for theta_2
			 *
			 *   There is one unique solution for theta_2.
			 *--------------------------------------------------------------------*/
			double c3 = cos(angles(3,th3index));
			SquareMatrix M(2); ColumnVector b(2);
			M(1,1) = a23 + S4*s3;	M(1,2) = S4*c3;
			M(2,1) = -S4*c3;        M(2,2) = a23 + S4*s3;
			b(1) = -S6*X71 - p.S7*X1 - p.a71*c1;
			b(2) = -p.S1*s12 + S6*Y71 + p.S7*Y1 + p.a71*s1*c12;
			ColumnVector x;

			if ( !solve_pair(M,b,x) ) {
				//throw out_of_range("Encountered some bad solutions at th2."); 
				good_soln.submatrix(th3index,th3index+1,1,1) = 0;
				continue;
			}

			double c2 = x(1); double s2 = x(2);
			ang1 = atan2(s2,c2);
			if ( !fix_angle(2,ang1) ) good_soln.submatrix(th3index,th3index+1,1,1) = 0;
			//th2.submatrix(th3index,th3index+1,1,1) = ang1;
			angles.submatrix(2,2,th3index,th3index+1) = ang1;
			
			/*----------------------------------------------------------------------
			 * Step 6. Solve for theta_5
			 *
			 *   There are two possible solutions for theta_5 for each value of
			 *   theta_3. This creates the second branch in the tree of solutions.
			 *--------------------------------------------------------------------*/
			double X712 = X71*c2 - Y71*s2;
			double Y712 = c23*(X71*s2 + Y71*c2) - s23*Z71;
			double Z712 = s23*(X71*s2 + Y71*c2) + c23*Z71;
			double X7123 = X712*c3 - Y712*s3;
			double Y7123 = c34*(X712*s3 + Y712*c3) - s34*Z712;
			double Z7123 = s34*(X712*s3 + Y712*c3) + c34*Z712;
			double c5 = -(Z7123 - c45*c56)/(s45*s56);

			if ( isabout( fabs(c5), 1, 1e-6 ) ) c5 = 1.0;

			if (fabs(c5) > 1.0) {
				//throw out_of_range("Encountered some bad solutions at th5.");
				good_soln.submatrix(th3index,th3index+1,1,1) = 0;
				continue;
			}
			
			acos2(c5, ang1, ang2);
			if ( !fix_angle(5,ang1) ) good_soln(th3index) = 0;
			if ( !fix_angle(5,ang2) ) good_soln(th3index+1) = 0;
			//th5(th3index,1) = ang1;
			//th5(th3index+1,1) = ang2;
			angles(5,th3index) = ang1;
			angles(5,th3index+1) = ang2;
			
			for (int th5index = th3index; th5index <= th3index+1; th5index++) 
			{
				/*------------------------------------------------------------------
				 * Step 7. Solve for theta_4
				 *
				 *   There is one unique solution for theta_4.
				 *----------------------------------------------------------------*/
				double s5 = sin(angles(5,th5index));
				double X5_bar = s56*s5;
				double Y5_bar = -(s45*c56 + c45*s56*c5);
				M(1,1) = X5_bar;    M(1,2) = -Y5_bar;
				M(2,1) = Y5_bar;    M(2,2) = X5_bar;
				b(1) = X7123;
				b(2) = -Y7123;

				if ( !solve_pair(M,b,x) ) {
					//throw out_of_range("Encountered some bad solutions at th4.");
					good_soln(th5index) = 0;
					continue;
				}

				double c4 = x(1); double s4 = x(2);
				ang1 = atan2(s4,c4);
				if ( !fix_angle(4,ang1) ) good_soln(th5index) = 0;
				angles(4,th5index) = ang1;

				/*------------------------------------------------------------------
				 * Step 8. Solve for theta_6
				 *
				 *   There is one unique solution for theta_6.
				 *----------------------------------------------------------------*/
				double bX4 = s45*s4;
				double bY4 = -(s34*c45 + c34*s45*c4);
				double bZ4 = c34*c45 - s34*s45*c4;
				double X43 = bX4*c3 - bY4*s3;
				double Y43 = c23*(bX4*s3 + bY4*c3) - s23*bZ4;
				double Z43 = s23*(bX4*s3 + bY4*c3) + c23*bZ4;
				double X432 = X43*c2 - Y43*s2;
				double Y432 = c12*(X43*s2 + Y43*c2) - s12*Z43;
				double Z432 = s12*(X43*s2 + Y43*c2) + c12*Z43;
				double X4321 = X432*c1 - Y432*s1;
				double Y4321 = c71*(X432*s1 + Y432*c1) - s71*Z432;
				double Z4321 = s71*(X432*s1 + Y432*c1) + c71*Z432;
				double X43217 = X4321*c7 - Y4321*s7;
				double Y43217 = c67*(X4321*s7 + Y4321*c7) - s67*Z4321;
				double c6 = Y43217/s56;
				double s6 = X43217/s56;

				ang1 = atan2(s6,c6);
				if ( !fix_angle(6,ang1) ) good_soln(th5index) = 0;
				angles(6,th5index) = ang1;
			} // END THETA 5 BRANCH
		} // END THETA 3 BRANCH
	} // END THETA 1 BRANCH
	
	// Add in joint offsets
	angles.row(2) = angles.row(2) - this->links[2].get_joint_offset();
	angles.row(3) = angles.row(3) - this->links[3].get_joint_offset();
	for (int n=1; n <= 8; ++n) if(good_soln(n)) (*nGoodSolns)++;
	
	angles.release();
	return angles;
}

// [a71, S7, S1, alp71, th7, gam1]
void PA10::close_loop(const Matrix& T_1_tool, ClosedLoopParams& p)
{
/*
 * CLOSE_LOOP    Determines the closed-loop parameters for an open-loop
 *               chain.
 *
 *   USAGE:
 *   [a71, S7, S1, alp71, th71, gam1] = CLOSE_LOOP(ROBOT,T_F_TOOL)
 *   
 *   INPUTS:
 *       Robot - Robot object created using the Matlab Robot Toolbox
 *
 *       T_f_tool - Transformation matrix from fixed to the tool frame
 *
 *   OUTPUTS:
 *       a71 and S7 are all scalar values for the distances to travel 
 *       along the imaginary close-loop 7th frame x and z axes respectively.
 *
 *       S1 is the distance to travel from a71 to a12.
 *
 *       alp71, th7, gam1 are angles use to specify directions for the new
 *       7th frame.
 *
 *   REFERENCE:
 *       See Crane & Duffy's book for a more detailed explanation of these
 *       parameters.
 *
 *       Author: J.D. Yamokoski
 *       Modified: 5/4/2006
 */
	double S1_Offset = this->links[1].get_d();

	SquareMatrix T_1_6 = T_1_tool * inv_tmatrix(this->T_6_tool);

	ColumnVector S6_f = T_1_6.submatrix(1,3,3,3);
	ColumnVector a67_f = T_1_6.submatrix(1,3,1,1);
	ColumnVector P6_f = T_1_6.submatrix(1,3,4,4);

	// Now we can also define the z-axis unit vectors for the first and 7th
	// frames.
	ColumnVector S7_f = crossproduct(a67_f, S6_f);
	S7_f = S7_f * (1/norm_Frobenius(S7_f));
	ColumnVector S1_f = this->T_f_base.submatrix(1,3,3,3);

	// At this point we can go no further without doing some checks. We will use
	// cos(alpz1) [c71] to test if S1_f and S7_f are parrallel
	double c71 = dotproduct(S7_f,S1_f);
	ColumnVector a71_f(3);

	// Check the first special case:
	if ( isabout(fabs(c71),1,1e-6) ) {// S1 and S7 parrallel?
		// If S1_f and S7_f are parrallel, then we will just set S7 to zero and
		// continue on with calculations.
		double s71 = 0;
		p.alp71 = atan2(s71,c71);
	    
		// Calculate the scalar distances for the parrallel case
		p.S7 = 0;
		p.S1 = -1*dotproduct(P6_f,S1_f) + S1_Offset; // Add any existing S1
		p.a71 = norm_Frobenius(-1*(P6_f + p.S1 * S1_f));
	    
		// Here we have our second and final special case. If the scalar
		// distance a71 is zero, then that implies that not only are S1_f and 
		// S7_f parrallel (condition required to get into this part of the if
		// statement) but they are also collinear! So let's now handle that 
		// case:
		if (isabout(fabs(p.a71),0,1e-6)) {
			//disp('S1_f and S7_f are collinear!');
			p.a71 = 0;
			p.th7 = 0;
			a71_f = a67_f; // Since th7 = 0, these are now parrallel.
		} else {
			// Finish calculations for just parrallel case
			//disp('S1_f and S7_f are parrallel!'); // Can make this statement with confidence now.
			a71_f = (-1*(P6_f + p.S1 * S1_f)) * (1/p.a71);

			// With a71_f we can now get theta7 (th7)
			double c7 = dotproduct(a67_f,a71_f);
			double s7 = dotproduct(crossproduct(a67_f,a71_f),S7_f);
			p.th7 = atan2(s7,c7);
		}
	} else {// Normal case
		a71_f = crossproduct(S7_f,S1_f) * ( 1 / (crossproduct(S7_f,S1_f).norm_Frobenius() ) );
	    
		// With a71_f we can now get theta7 (th7)
		double c7 = dotproduct(a67_f,a71_f);
		double s7 = dotproduct(crossproduct(a67_f,a71_f),S7_f);
		p.th7 = atan2(s7,c7);
	    
		// Now we have checked special cases, and by this time we should have the
		// correct a71_f vector so we can finish calculation of alp71.
		double s71 = dotproduct(crossproduct(S7_f,S1_f),a71_f);
		p.alp71 = atan2(s71,c71);
	    
		// Finally get the three scalar distances for the normal case.
		p.S7 = dotproduct(crossproduct(S1_f,P6_f),a71_f)/s71;
		p.a71 = dotproduct(crossproduct(P6_f,S1_f),S7_f)/s71;
		p.S1 = (dotproduct(crossproduct(P6_f,S7_f),a71_f)/s71) + S1_Offset; // Add any existing S1
	}

	// Calculation of gam1 the same for all cases.
	ColumnVector xaxis(3);
	xaxis << 1.0 << 0.0 << 0.0;
	double cgam1 = dotproduct(a71_f,xaxis);
	double sgam1 = dotproduct(crossproduct(a71_f,xaxis),S1_f);
	p.gam1 = atan2(sgam1,cgam1);
}

bool PA10::isabout(const double& var, const double& value, const double& tol) {
	if (fabs(var - value) < tol) return true;
	else return false;
}

bool PA10::solve_trig(const double& A, const double& B, const double& Din, double& ang1, double& ang2) 
{
/*   SOLVE_TRIG     Determines coefficients of trigonometric equation
 *       SOLVE_TRIG(A, B, D) returns the two solutions (ANG_A
 *       and ANG_B) of the equation:
 *
 *           A*cos(x) + B*sin(x) + D = 0
 *
 *       SOLVE_TRIG will also return a Boolean 1 or 0 depending on the
 *       success of the calculation. If it returns a 0, then no solution was
 *       found and ANG_A and ANG_B = [0]
 */
	double D = Din;
	if ( fabs(D) < 1e-9 ) D=0;

	if ( D == 0 ) { // Special case
		ang1 = atan2(-A,B);
		ang2 = atan2(A,-B);
		return true;
	} else {
		double sgam = B/sqrt(pow(A,2) + pow(B,2));
		double cgam = A/sqrt(pow(A,2) + pow(B,2));
	    
		double gam = atan2(sgam,cgam);
	    
		double RHS = -D/sqrt(pow(A,2) + pow(B,2));
	    
		if (fabs(RHS) <= 1) {
			//double LHS1 = acos(RHS);
			//double LHS2 = -acos(RHS);
			double LHS1 = 0, LHS2 = 0;
			acos2(RHS, LHS1, LHS2);
	    
			ang1 = LHS1 + gam;
			ang2 = LHS2 + gam;
			return true;
		} else {
			ang1 = NO_DATA;
			ang2 = NO_DATA;
			return false;
		}
	}
}

//function [x,y,int] = 
bool PA10::solve_pair(const Matrix& A, const ColumnVector& b, ColumnVector& x)
{
	// if linearly dependent then we are unable to solve the eqns.
	// otherwise, use linear algebra to solve for x,y
	double test = determinant(A);
	if ( test < 1e-12 ) {
		return false;
	} else {
		x = A.i()*b;
		return true;
	}
}

bool PA10::fix_angle(const int& jn, double& ang)
{
	double ang_copy = ang;

	// must convert to actual ja before checking against limits
	ang = ang - this->links[jn].get_joint_offset();

	if ( ang > this->links[jn].get_theta_max() ) {
		// Flip angle around unit circle. Check if this new angle
		// exceeds the lower range of the joint.
		ang = ang - 2*M_PI;
		if ( ang < this->links[jn].get_theta_min() ) {
			ang = ang_copy;
			return false; 
		} else {
			ang = ang + this->links[jn].get_joint_offset();
			return true;
		}
	}

	// Check angle against lower range of joint
	if ( ang < this->links[jn].get_theta_min() ) {
		// Flip angle around unit circle. Check if this new angle
		// exceeds the upper range of the joint.
		ang = ang + 2*M_PI;
		if ( ang > this->links[jn].get_theta_max() ) {
			ang = ang_copy;
			return false;
		} else {
			ang = ang + this->links[jn].get_joint_offset();
			return true;
		}
	}

	ang = ang + this->links[jn].get_joint_offset();
	return true;
}
