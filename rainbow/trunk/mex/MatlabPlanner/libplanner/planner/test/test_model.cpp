#include "planner\rlg.h"
#include "planner\linalg.h"

#include <stdio.h>
#include <time.h>

int main(void)
{
	PA10 source("pa10_dh.conf","PA10_DH");
	source.T_f_base=transl(1.0, 0.0, 0.5843) * rotz(M_PI);
	source.T_6_tool=rotx(M_PI);

	PA10 sensor("pa10_dh.conf","PA10_DH");
	sensor.T_f_base=transl(-1.0, 0.0, 0.5843);

	SquareMatrix m = transl(0,0,.9031) * roty(M_PI/2);

	RandomLoopGenerator rlg(&source, &sensor);
	rlg.set_del(.01);
	rlg.set_sigma_theta(M_PI/(6));
	rlg.set_dbar(.1);
	rlg.set_sigma_d(.05);
	rlg.set_Lbar(1.0);
	rlg.set_sigma_L(.3);
	rlg.set_max_tries(100000);
	rlg.set_T_wcs_d(m);

	Matrix Qr;
	clock_t start = clock();
	try {
		Matrix S[5000];
		rlg(5000, S);
	} catch (int n) {
		std::cout << "Failed to generate 10000 states 100000 attempts!" << std::endl;
	}
	double elapsed = ((double)(clock() - start))/CLOCKS_PER_SEC;
	double times_failed = -rlg.get_num_unique_samples() + rlg.get_num_attempts();

	std::cout << "Mean run-time per good sample: " << elapsed/(double)rlg.get_num_unique_samples() << std::endl;
	std::cout << "Total run-time: " << elapsed << std::endl;
	std::cout << "	Number unique samples: " << rlg.get_num_unique_samples() << std::endl;
	std::cout << "	Number total samples: " << rlg.get_num_total_samples() << std::endl;
	std::cout << "	Number of attempts: " << rlg.get_num_attempts() << std::endl;
	std::cout << "	Failed sensor intersections: " << 100*((double)rlg.get_failed_sensor_int()/times_failed) << "%" << std::endl;
	std::cout << "	Failed source intersections: " << 100*((double)rlg.get_failed_source_int()/times_failed) << "%" << std::endl;
	std::cout << "	Failed sensor ikine: " << 100*((double)rlg.get_failed_sensor_ikine()/times_failed) << "%" << std::endl;
	std::cout << "	Failed source ikine: " << 100*((double)rlg.get_failed_source_ikine()/times_failed) << "%" << std::endl;
}