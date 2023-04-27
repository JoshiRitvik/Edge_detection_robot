#ifndef CADMIUM_SENSOR_BLOCK_HPP_
#define CADMIUM_SENSOR_BLOCK_HPP_

#include <cadmium/core/modeling/coupled.hpp>
#include "proximity_sense.hpp"
#ifdef RT_ARM_MBED
	#include <cadmium/core/real_time/arm_mbed/io/digitalOutput.hpp>
	#include <cadmium/core/real_time/arm_mbed/io/digitalInput.hpp>
	#include "../mbed.h"
	#include "PinNames.h"
#endif
namespace cadmium::example::Edge_robot {
	//! Coupled DEVS model of the experimental frame.
	struct Sensor_block: public Coupled {
//		BigPort<Job> inProcessed;   //!< Input Port for processed Job objects.
		Port<int> Proximity_data;  //!< Output Port for sending new Job objects to be processed.

		/**
		 * Constructor function for the experimental frame model.
		 * @param id ID of the experimental frame model.
		 * @param jobPeriod Job generation period for the Generator model.
		 * @param obsTime time to wait by the Transducer before asking the Generator to stop creating Job objects.
		 */
		Sensor_block(const std::string& id, double samplingTime): Coupled(id) {
			std::cout << " Sensor_block " << std::endl;
			//inProcessed = addInPort<Job>("inProcessed");
			Proximity_data = addOutPort<int>("Proximity_data");

			auto proximity_sensor = addComponent<Proximity_sensor>("proximity_sensor", samplingTime);
#ifdef RT_ARM_MBED		
			// NUCLEO F411RE
			auto digitalSenseInput1  = addComponent<DigitalInput>("digitalInput", PA_4);
			auto digitalSenseInput2  = addComponent<DigitalInput>("digitalInput", PA_1);
//			auto digitalSenseInput3  = addComponent<DigitalInput>("digitalInput", PB_0);
			// BLUE PILL
			// auto digitalOutput = addComponent<DigitalOutput>("digitalOuput", PC_13);
			// auto digitalInput  = addComponent<DigitalInput>("digitalInput", PB_14);
			addCoupling(digitalSenseInput1->out, proximity_sensor->sensor_input);
//			addCoupling(digitalSenseInput2->out, proximity_sensor->sensor_input);
//			addCoupling(digitalSenseInput3->out, proximity_sensor->sensor_input);
#endif

			addCoupling(proximity_sensor->Proximity_data, Proximity_data);
		}
	};
}  //namespace cadmium::example::gpt

#endif //CADMIUM_SENSOR_BLOCK_HPP_
