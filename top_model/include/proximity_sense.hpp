#ifndef CADMIUM_PROXIMITY_SENSOR_HPP_
#define CADMIUM_PROXIMITY_SENSOR_HPP_

#include <cadmium/core/modeling/atomic.hpp>
#include <iostream>
//#include "job.hpp"

namespace cadmium::example::Edge_robot {
	//! Class for representing the Generator DEVS model state.
	struct Proximity_Sense_State {
		double clock;  //!< Current simulation time.
		double sigma;  //!< Time to wait before triggering the next internal transition function.
		int sensor_value;
		//! Constructor function. It sets all the attributes to 0.
		Proximity_Sense_State(): clock(), sigma(), sensor_value() {} 
	};

	/**
	 * Insertion operator for GeneratorState objects. It only displays the value of jobCount.
	 * @param out output stream.
	 * @param s state to be represented in the output stream.
	 * @return output stream with jobCount already inserted.
	 */
	std::ostream& operator<<(std::ostream& out, const Proximity_Sense_State& s) {
		out <<"sampling rate"<< s.sigma << "sensor value"<< s.sensor_value;
		return out;
	}

	//! Atomic DEVS model of a Job generator.
	class Proximity_sensor : public Atomic<Proximity_Sense_State> {
	 private:
		double samplingTime;                            //!< Time to wait between Job generations.
	 public:
		//Port<bool> inStop;          //!< Input Port for receiving stop generating Job objects.
		Port<int> Proximity_data;  //!< Output Port for sending new Job objects to be processed.
		Port<bool>sensor_input;

		/**
		 * Constructor function for Generator DEVS model.
		 * @param id model ID.
		 * @param jobPeriod Job generation period.
		 */
		Proximity_sensor(const std::string& id, double samplingTime): Atomic<Proximity_Sense_State>(id, Proximity_Sense_State()), samplingTime(samplingTime)  {
			Proximity_data = addOutPort<int>("Proximity_data");
			sensor_input = addInPort<bool>("Sensor_input");
		}

		/**
		 * Updates GeneratorState::clock and GeneratorState::sigma an
		 * d increments GeneratorState::jobCount by one.
		 * @param s reference to the current generator model state.
		 */
		void internalTransition(Proximity_Sense_State& s) const override {
			s.clock += s.sigma;
			s.sigma = samplingTime;
			//s.sensor_value = int(1 + (rand() % 100));
		}

		/**
		 * Updates GeneratorState::clock and GeneratorState::sigma.
		 * If it receives a true message via the Generator::inStop port, it passivates and stops generating Job objects.
		 * @param s reference to the current generator model state.
		 * @param e time elapsed since the last state transition function was triggered.
		 * @param x reference to the atomic model input port set.
		 */
		void externalTransition(Proximity_Sense_State& s, double e) const override {
			s.clock += e;
			s.sigma = std::max(s.sigma - e, 0.);
			if(!sensor_input->empty()){
				for( const auto x : sensor_input->getBag()){
					if (x==0)
						s.sensor_value = int(100);
					else
						s.sensor_value = int(1);
				}
			
		}
		}
		/**
		 * Sends a new Job that needs to be processed via the Generator::outGenerated port.
		 * @param s reference to the current generator model state.
		 * @param y reference to the atomic model output port set.
		 */
		void output(const Proximity_Sense_State& s) const override {
			Proximity_data->addMessage(s.sensor_value);
			// outGenerated->addMessage(Job(s.jobCount, s.clock + s.sigma)); // TODO we could also do this
		}

		/**
		 * It returns the value of GeneratorState::sigma.
		 * @param s reference to the current generator model state.
		 * @return the sigma value.
		 */
		[[nodiscard]] double timeAdvance(const Proximity_Sense_State& s) const override {
			return s.sigma;
		}
	};
}  //namespace cadmium::example::Edge_robot

#endif //
