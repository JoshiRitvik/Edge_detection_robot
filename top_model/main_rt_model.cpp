#include <cadmium/core/simulation/root_coordinator.hpp>
#include "include/Edge_robot.hpp"
#ifndef NO_LOGGING
	#ifdef RT_ARM_MBED
		#include <cadmium/core/logger/rt.hpp>
	#else
		#include <cadmium/core/logger/csv.hpp>
	#endif
#endif

#include <limits>
#include "Edge_robot.hpp"

#ifdef RT_ARM_MBED
	#include "../mbed.h"
#endif

using namespace cadmium::example::Edge_robot;

int main(int argc, char *argv[]) {
	double DrivePeriod = 0.001, samplingTime = 1, cmdTime = 0.01;
	auto model = std::make_shared<Edge_detect_robot>("Edge_detect_robot", DrivePeriod, samplingTime, cmdTime);
	auto rootCoordinator = cadmium::RootCoordinator(model);

#ifndef NO_LOGGING

	#ifdef RT_ARM_MBED
		auto logger = std::make_shared<cadmium::RTLogger>(";");
	# else
		auto logger = std::make_shared<cadmium::CSVLogger>("Edge_robot_Log.csv",";"); // new
	#endif

	rootCoordinator.setLogger(logger);
#endif

	rootCoordinator.start();
// 	rootCoordinator.simulate(std::numeric_limits<double>::infinity());
	rootCoordinator.simulate(1000.0);
	rootCoordinator.stop();
	return 0;
}
