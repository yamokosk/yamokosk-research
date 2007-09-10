#ifndef RLG_HEADER_FILE
#define RLG_HEADER_FILE

#include "pa10.h"
#include <boost/random.hpp>

class RandomLoopGenerator
{
	typedef boost::mt19937 mt19937;
	typedef boost::uniform_real<double> unidist;
	typedef boost::normal_distribution<double> normdist;
	
public:
	RandomLoopGenerator(PA10* src, PA10* sen);
	//RandomLoopGenerator(Robot* src, Robot* sen);
	virtual ~RandomLoopGenerator();

	void operator()(int N, Matrix* S);
	
	void set_del(double x) {del = x;};
	void set_sigma_theta(double x) {sigma_theta=x;};
	void set_dbar(double x) {dbar = x;};
	void set_sigma_d(double x) {sigma_d = x;};
	void set_Lbar(double x) {Lbar = x;};
	void set_sigma_L(double x) {sigma_L = x;};
	void set_max_tries(int n) {max_tries = n;};
	void set_T_wcs_d(const SquareMatrix& m) {T_wcs_d = m;};

	int get_failed_sensor_int() {return failed_sensor_int;};
	int get_failed_source_int() {return failed_source_int;};
	int get_failed_sensor_ikine() {return failed_sensor_ikine;};
	int get_failed_source_ikine() {return failed_source_ikine;};
	int get_num_unique_samples() {return num_unique_samples;};
	int get_num_total_samples() {return num_total_samples;};
	int get_num_attempts() {return num_attempts;};

	void reset_stats();
private:
	ReturnMatrix randomConfiguration(bool& bSuccess);
	bool ray_sphere_int(const ColumnVector& P0,const ColumnVector& Pd,const ColumnVector& Sc, const double& Sr, double* t1, double* t2) const;

	double del, sigma_theta, dbar, sigma_d, Lbar, sigma_L;
	int max_tries, failed_sensor_int, failed_source_int, failed_sensor_ikine, failed_source_ikine, num_unique_samples, num_total_samples, num_attempts;
	SquareMatrix T_wcs_d;
	PA10 *source, *sensor;

	mt19937 rng1, rng2; 
	boost::variate_generator<mt19937&, unidist> rand;
	boost::variate_generator<mt19937&, normdist> randn;
};

#endif