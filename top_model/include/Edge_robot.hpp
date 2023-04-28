#ifndef CADMIUM_EDGE_ROBOT_HPP_
#define CADMIUM_EDGE_ROBOT_HPP_

#include <cadmium/core/modeling/coupled.hpp>
#include "motion_controller.hpp"
#include "wheel.hpp"
#include "generator.hpp"
#ifdef RT_ARM_MBED
	#include <cadmium/core/real_time/arm_mbed/io/digitalOutput.hpp>
	#include <cadmium/core/real_time/arm_mbed/io/digitalInput.hpp>
	#include <cadmium/core/real_time/arm_mbed/io/pwmOutput.hpp>
	#include "../mbed.h"
	#include "PinNames.h"
#endif
namespace cadmium::example::Edge_robot {
	//! Coupled DEVS model of the generator-processor-transducer.
	struct Edge_detect_robot : public Coupled {
		/**
		 * Constructor function for the Edge_robot model.
		 * @param id ID of the gpt model.
		 * @param DrivePeriod driving motor period of robot model.
		 * @param samplingTime sensor sampling time of robot model.
		 * @param cmdTime time for data processing and sending motion command.
		 */
		Edge_detect_robot(const std::string& id, double DrivePeriod, double samplingTime, double cmdTime): Coupled(id) {
//			std::cout << " Edge robot " << std::endl;
//			auto actuators = addComponent<Actuators>("actuators", DrivePeriod);
//			auto sensor_block = addComponent<Sensor_block>("sensor_block", samplingTime);
			auto motion_controller = addComponent<Motion_controller>("motion_controller", cmdTime);
			auto wheel_drive = addComponent<Wheel>("wheel_drive", DrivePeriod);
			auto generator = addComponent<Generator>("generator");
#ifdef RT_ARM_MBED		
			// NUCLEO F411RE
			auto digitalSenseInput1  = addComponent<DigitalInput>("digitalSenseInput1", PA_4); //PC_13
			auto digitalSenseInput2  = addComponent<DigitalInput>("digitalSenseInput2", PA_1);
			auto digitalSenseInput3  = addComponent<DigitalInput>("digitalSenseInput3", PB_0);
			auto PWMOutput1 = addComponent<PWMOutput>("PWMOutput1", PA_7); //PB_0 LED1
			auto PWMOutput2 = addComponent<PWMOutput>("PWMOutput2", PA_9);
			auto PWMOutput3 = addComponent<PWMOutput>("PWMOutput3", PA_5);
			auto PWMOutput4 = addComponent<PWMOutput>("PWMOutput4", PA_6);
			auto MotorEnableLeft  = addComponent<DigitalOutput>("MotorEnableLeft", PC_7);
			auto MotorEnableRight  = addComponent<DigitalOutput>("MotorEnableRight", PB_6);

			addCoupling(digitalSenseInput1->out, motion_controller->sensor_input);
			addCoupling(digitalSenseInput2->out, motion_controller->sensor_input2);
			addCoupling(digitalSenseInput3->out, motion_controller->sensor_input3);
			addCoupling(wheel_drive->wheel_left_fw, PWMOutput1->in);
			addCoupling(wheel_drive->wheel_left_bw, PWMOutput2->in);
			addCoupling(wheel_drive->wheel_right_fw, PWMOutput3->in);
			addCoupling(wheel_drive->wheel_right_bw, PWMOutput4->in);

			addCoupling(wheel_drive->wheel_left_en, MotorEnableLeft->in);
			addCoupling(wheel_drive->wheel_right_en, MotorEnableRight->in);
#else
			// auto digitalSenseInput1  = addInPort<bool>("digitalSenseInput1"); //PC_13
			// auto digitalSenseInput2  = addInPort<bool>("digitalSenseInput2");
			// auto digitalSenseInput3  = addInPort<bool>("digitalSenseInput3");

			addCoupling(generator->out1, motion_controller->sensor_input);
			addCoupling(generator->out2, motion_controller->sensor_input2);
			addCoupling(generator->out3, motion_controller->sensor_input3);

#endif 
			addCoupling(motion_controller->Out_flag, wheel_drive->In_Flag);
		}
	};
}  //namespace cadmium::example::Edge_robot

#endif //CADMIUM_EDGE_ROBOT_HPP_
