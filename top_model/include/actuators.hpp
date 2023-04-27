#ifndef CADMIUM_ACTUATORS_HPP_
#define CADMIUM_ACTUATORS_HPP_

#include <cadmium/core/modeling/coupled.hpp>
#include "wheel.hpp"
#include "motion_controller.hpp"

#ifdef RT_ARM_MBED
	#include <cadmium/core/real_time/arm_mbed/io/digitalOutput.hpp>
	#include <cadmium/core/real_time/arm_mbed/io/pwmOutput.hpp>
	#include <cadmium/core/real_time/arm_mbed/io/digitalInput.hpp>
	#include "../mbed.h"
	#include "PinNames.h"
#endif

namespace cadmium::example::Edge_robot {
	//! Coupled DEVS model of the experimental frame-processor.
	struct Actuators : public Coupled {
		Port<int> wheel_speed;
		/**
		 * Constructor function for the GPT model.
		 * @param id ID of the gpt model.
		 * @param jobPeriod Job generation period for the Generator model.
		 * @param processingTime Job processing time for the Processor model.
		 * @param obsTime time to wait by the Transducer before asking the Generator to stop creating Job objects.
		 */
		Actuators(const std::string& id, double DrivePeriod) : Coupled(id) {
			auto wheel = addComponent<Wheel>("wheel", DrivePeriod);
			auto wheel2 = addComponent<Wheel>("wheel2", DrivePeriod);
			wheel_speed = addInPort<int>("wheel_start_stop");

			addCoupling(wheel_speed, wheel->In1); // motion_controller->
			addCoupling(wheel_speed, wheel2->In1);
#ifdef RT_ARM_MBED		
			// NUCLEO F411RE
			auto PWMOutput1 = addComponent<PWMOutput>("PWMOutput1", PB_0);
			auto PWMOutput2  = addComponent<PWMOutput>("PWMOutput2", PC_10);
			// BLUE PILL
			// auto digitalOutput = addComponent<DigitalOutput>("digitalOuput", PC_13);
			// auto digitalInput  = addComponent<DigitalInput>("digitalInput", PB_14);
			addCoupling(wheel2->Out, PWMOutput2->in);
			addCoupling(wheel->Out, PWMOutput1->in);
#endif

		}
	};
}  //namespace cadmium::example::cleaning_robot

#endif //CADMIUM_ACTUATORS_HPP_
