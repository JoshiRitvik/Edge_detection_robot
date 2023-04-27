#ifndef CADMIUM_MOTION_CONTROLLER_HPP_
#define CADMIUM_MOTION_CONTROLLER_HPP_

#include <cadmium/core/modeling/atomic.hpp>
#include <iostream>
#include <optional>

namespace cadmium::example::Edge_robot {
	//! Class for representing the Processor DEVS model state.
	struct motion_controller_State {
		double clock;                   //!< Current simulation time.
		double sigma;                   //!< Time to wait before triggering the next internal transition function.
	 	bool Remote_flag;
		int Obstacle_distance;
		int start_stop_flag;
		//! Processor state constructor. By default, the processor is idling.
		explicit motion_controller_State(double cmdTime): clock(), sigma(cmdTime), Remote_flag(), Obstacle_distance(), start_stop_flag() {} 
	};

	/**
	 * Insertion operator for ProcessorState objects. It only displays the value of sigma.
	 * @param out output stream.
	 * @param s state to be represented in the output stream.
	 * @return output stream with sigma already inserted.
	 */
	std::ostream& operator<<(std::ostream &out, const motion_controller_State& s) {
		out  << "{" << s.Remote_flag << " " << s.Obstacle_distance << " " << s.start_stop_flag << "}";
		return out;
	}

	//! Atomic DEVS model of a Job processor.
	class Motion_controller : public Atomic<motion_controller_State> {
	 private:
		double cmdTime;  //!< Time required by the Processor model to process one Job.
	 public:
		Port<bool> Remote_control;  //!< Input Port for receiving new Job objects.
		Port<int> Proximity_data;  //!< Output Port for sending processed Job objects.
		Port<int> wheel_speed;

		/**
		 * Constructor function.
		 * @param id ID of the new Processor model object.
		 * @param cmdTime time it takes the Processor to process a Job.
		 */
		Motion_controller(const std::string& id, double cmdTime): Atomic<motion_controller_State>(id, motion_controller_State(cmdTime)) {
			Remote_control = addInPort<bool>("Remote_control");
			Proximity_data = addInPort<int>("Proximity_data");
			wheel_speed = addOutPort<int>("wheel_speed");
			std::cout << "motion controller  " << std::endl;

		}

		/**
		 * It updates the ProcessorState::clock, clears the ProcessorState::Job being processed, and passivates.
		 * @param s reference to the current state of the model.
		 */
		void internalTransition(motion_controller_State& s) const override {
			std::cout << "motion controller internal" << std::endl;
			s.clock += s.sigma;
			//s.currentJob.reset();
			s.sigma = 100;//std::numeric_limits<double>::infinity();
		}

		/**
		 * Updates ProcessorState::clock and ProcessorState::sigma.
		 * If it is idling and gets a new Job via the Processor::inGenerated port, it starts processing it.
		 * @param s reference to the current model state.
		 * @param e time elapsed since the last state transition function was triggered.
		 * @param x reference to the model input port set.
		 */
		void externalTransition(motion_controller_State& s, double e) const override {
			s.clock += e;
			s.sigma -= e;
//			 std::cout << "motion controller external " << std::endl;
/*			if(!Remote_control->empty()){
				for( const auto x : Remote_control->getBag()){
					s.Remote_flag = Remote_control->getBag().back();
					std::cout << "remote flag " << s.start_stop_flag << std::endl;
					// if (s.start_stop_flag ==0)
					// 	s.Remote_flag = !s.Remote_flag;
				}
			}*/
				if(!Proximity_data->empty()) { 
					s.Obstacle_distance = Proximity_data->getBag().back();
					std::cout << s.Obstacle_distance << std::endl;						
					if (s.Obstacle_distance > int(10)){
						s.start_stop_flag = 100;
						std::cout << " start command " << s.start_stop_flag << std::endl;
					}
					else
					{
						s.start_stop_flag = 0;
						std::cout << " stop command " << std::endl;
					}
				}
			s.sigma = cmdTime;
		}

		/**
		 * It outputs the already processed ProcessorState::Job via the Processor::outProcessed port.
		 * @param s reference to the current model state.
		 * @param y reference to the atomic model output port set.
		 */
		void output(const motion_controller_State& s) const override {
			wheel_speed->addMessage(s.start_stop_flag);
			std::cout << " start stop command " << std::endl;
		}

		/**
		 * It returns the value of ProcessorState::sigma.
		 * @param s reference to the current model state.
		 * @return the sigma value.
		 */
		[[nodiscard]] double timeAdvance(const motion_controller_State& s) const override {

			return s.sigma;
		}
	};
}  //namespace cadmium::example::cleaning_robot

#endif //CADMIUM_MOTION_CONTROLLER_HPP_
