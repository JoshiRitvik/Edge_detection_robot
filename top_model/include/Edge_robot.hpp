#ifndef CADMIUM_EDGE_ROBOT_HPP_
#define CADMIUM_EDGE_ROBOT_HPP_

#include <cadmium/core/modeling/coupled.hpp>
#include "actuators.hpp"
#include "Sensor_block.hpp"
#include "motion_controller.hpp"
#include "remote_input.hpp"

namespace cadmium::example::Edge_robot {
	//! Coupled DEVS model of the generator-processor-transducer.
	struct Edge_detect_robot : public Coupled {
		//Port<bool> Ext_Remote_control;
		/**
		 * Constructor function for the GPT model.
		 * @param id ID of the gpt model.
		 * @param jobPeriod Job generation period for the Generator model.
		 * @param processingTime Job processing time for the Processor model.
		 * @param cmdTime time to wait by the Transducer before asking the Generator to stop creating Job objects.
		 */
		Edge_detect_robot(const std::string& id, double DrivePeriod, double samplingTime, double cmdTime): Coupled(id) {
			std::cout << " Edge robot " << std::endl;
			auto actuators = addComponent<Actuators>("actuators", DrivePeriod);
			auto sensor_block = addComponent<Sensor_block>("sensor_block", samplingTime);
			auto motion_controller = addComponent<Motion_controller>("motion_controller", cmdTime);
			auto remote_control_In = addComponent<Remote_control_In>("remote_control_In", cmdTime);
			//Ext_Remote_control = addInPort<bool>("Ext_Remote_control");

			addCoupling(remote_control_In->Ext_Remote_control, motion_controller->Remote_control); //just for test
			addCoupling(motion_controller->wheel_speed, actuators->wheel_speed);
			addCoupling(sensor_block->Proximity_data, motion_controller->Proximity_data);
			//addCoupling(remote_control_In->Ext_Remote_control, Ext_Remote_control);  

			std::cout << " Edge robot bottom " << std::endl;
		}
	};
}  //namespace cadmium::example::cleaning_robot

#endif //CADMIUM_EDGE_ROBOT_HPP_
