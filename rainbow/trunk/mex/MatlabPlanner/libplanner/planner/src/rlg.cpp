#include "planner\rlg.h"
#include "planner\linalg.h"

RandomLoopGenerator::RandomLoopGenerator(PA10* src, PA10* sen) :
//RandomLoopGenerator::RandomLoopGenerator(Robot* src, Robot* sen) :
	del(0.0), sigma_theta(0.0), dbar(0.1), sigma_d(0.01), Lbar(1.0), sigma_L(0.3),
	max_tries(10), failed_sensor_int(0), failed_source_int(0), failed_sensor_ikine(0), failed_source_ikine(0), num_unique_samples(0), num_total_samples(0), num_attempts(0),
	source(src), sensor(sen),
	rand(rng1, unidist(0,1)),
	randn(rng2, normdist(0,1))
{

}

RandomLoopGenerator::~RandomLoopGenerator() {}

void RandomLoopGenerator::operator()(int N, Matrix* S)
{
	reset_stats();

	//Matrix out(N,12);
	
	int row = 1;
	while ((num_unique_samples < N) && (num_attempts < max_tries) ) {
		bool bSuccess = false;
		Matrix Qr = randomConfiguration(bSuccess);
		if (bSuccess) {
			//int stride = (Qr.nrows() > (N - row)) ? (N - row) : Qr.nrows();
			//out.submatrix(row,row+stride-1,1,12) = Qr.submatrix(1,stride,1,12);
			//row = row + stride;
			S[num_unique_samples] = Qr;
			num_total_samples = Qr.nrows() + num_total_samples;
			num_unique_samples++;
		}

		// Increment attempts and check to see if we have tried too many times.
		num_attempts++;
		//if ( num_attempts > max_tries ) {
		//	std::cout << "Reached max number of attempts" << std::endl;	
		//	break;
		//}
	}

	//out.release();
	//return out;
}

//bool RandomLoopGenerator::randomConfiguration(ColumnVector& qr)
ReturnMatrix RandomLoopGenerator::randomConfiguration(bool &success)
{
	Matrix Qr;
	success = true;

	/* Step 1. Generate perturbed desired target t-matrix
	 *	1a. Random walk in target's X-Y plane: T_d_dp = transl(del*cos(alpha), del*sin(alpha), 0);
	 *	1b. Random rotation of target's X-Z plane: T_dp_dpp = rotz(phi);
	 *	1c. View orientation error: T_dpp_dhat = roty(sigma_theta * randn());
	 *	1d. Concatenation: SquareMatrix T_dp_dhat = T_dp_dpp * T_dpp_dhat;
	 *	1e. Concatenation: T_wcs_dhat = T_wcs_d * T_d_dp * T_dp_dhat; */
	double alpha = rand() * 2 * M_PI - M_PI;
	double phi = rand() * 2 * M_PI - M_PI; // Random rotation of the local X-Z plane
	SquareMatrix T_wcs_dhat = T_wcs_d * transl(del*cos(alpha), del*sin(alpha), 0) * rotz(phi) * roty(sigma_theta * randn());
	
	/* Step 2. Shoot a ray aligned with the z-axis of T_wcs_dhat and
	 * calculate where (and if) it intersects the reachable workspace of the robots.
	 * If the ray fails to intersect both robot workspaces, return false. */
	double t1=0.0, t2=0.0, t3=0.0, t4=0.0;
	ColumnVector P_wcs_dhat0 = T_wcs_dhat.submatrix(1,3,4,4);
	ColumnVector P_wcs_dhat_Z(3), P_wcs_S1(3); Matrix T_wcs_S1(4,4);
	
	//std::cout << "Sensor: " << std::endl;
	P_wcs_dhat_Z = P_wcs_dhat0 - T_wcs_dhat.submatrix(1,3,3,3); // Point on the dhat, minus Z-axis
	T_wcs_S1 = sensor->kine(1); P_wcs_S1 = T_wcs_S1.submatrix(1,3,4,4);
	bool sen_int_success = ray_sphere_int(P_wcs_dhat0, P_wcs_dhat_Z, P_wcs_S1, .855, &t1, &t2);
	if (!sen_int_success) failed_sensor_int++;

	//std::cout << "Source: " << std::endl;
	P_wcs_dhat_Z = P_wcs_dhat0 + T_wcs_dhat.submatrix(1,3,3,3); // Point on the dhat, positive Z-axis
	T_wcs_S1 = source->kine(1); P_wcs_S1 = T_wcs_S1.submatrix(1,3,4,4);
	bool src_int_success = ray_sphere_int(P_wcs_dhat0, P_wcs_dhat_Z, P_wcs_S1, .855, &t3, &t4);
	if (!src_int_success) failed_source_int++; 
	
	if (!sen_int_success || !src_int_success) {success=false; Qr.release(); return Qr;} // One or both intersection test failed.. return

	// Deal with sen then src
	double d = dbar + sigma_d * randn();
	if (d < t1) d = t1; // Clamp d so that point lies within the sensor workspace
	else if (d > t2) d = t2; 
	SquareMatrix T_wcs_sen = T_wcs_dhat * transl(0.0, 0.0, -d); // Sensor frame in the WCS

	double L = Lbar + sigma_L * randn();
	if ( (L-d) < t3 ) L = t3 + d; // Clamp L so that point lies within the source workspace
	else if ( (L-d) > t4 ) L = t4 + d;
	SquareMatrix T_wcs_src = T_wcs_dhat * transl(0.0, 0.0, L) * rotz(phi);// * rotx(M_PI); // Source tool frame in the WCS
	
	/* Step 3. Use inverse kinematics to compute the robot joint angles */
	bool sen_ikine_success = false, src_ikine_success = false;

	Matrix Qsrc = source->inv_kin(T_wcs_src, 0, 6, src_ikine_success);
	if (!src_ikine_success) failed_source_ikine++;

	Matrix Qsen = sensor->inv_kin(T_wcs_sen, 0, 6, sen_ikine_success);
	if (!sen_ikine_success) failed_sensor_ikine++;

	// One or both robots failed ika.. return
	if (!src_ikine_success || !sen_ikine_success) {success=false; Qr.release(); return Qr;};
	
	int nsrc = Qsrc.ncols(), nsen = Qsen.ncols();
	Qr.resize(nsrc*nsen,12); int ptr = 1;
	for (int m=1; m <= nsrc; ++m) {
		for (int n=1; n <= nsen; ++n) {
			Qr.submatrix(ptr,ptr,1,6) = (Qsrc.column(m)).t();
			Qr.submatrix(ptr,ptr,7,12) = (Qsen.column(n)).t();
			ptr++;
		}
	}

	Qr.release();
	return Qr;
}

bool RandomLoopGenerator::ray_sphere_int(const ColumnVector& P0,const ColumnVector& Pd,const ColumnVector& Sc, const double& Sr, double* t1, double* t2) const
{
	double A = dotproduct(Pd,Pd);
	double B = 2 * dotproduct(Pd,(P0 - Sc));
	double C = dotproduct((P0 - Sc),(P0 - Sc)) - pow(Sr,2);

	double discriminant = pow(B,2) - 4*A*C;
	if (discriminant >= 0) {
		*t1 = ( -B - sqrt(discriminant) )/(2*A);
		*t2 = ( -B + sqrt(discriminant) )/(2*A);
	} else {
		return false;
	}
	return true;
}

void RandomLoopGenerator::reset_stats()
{
	failed_source_int = 0;
	failed_sensor_int = 0;
	failed_source_ikine = 0;
	failed_sensor_ikine = 0;
	num_unique_samples = 0;
	num_attempts = 0;
}