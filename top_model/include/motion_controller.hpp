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
	 	bool Edge_flag;
		int Obstacle_distance;
		int fw_bw_flag;
		//! Processor state constructor. By default, the processor is idling.
		explicit motion_controller_State(double cmdTime): clock(), sigma(cmdTime), Edge_flag(), Obstacle_distance(), fw_bw_flag() {} 
	};

	/**
	 * Insertion operator for ProcessorState objects. It only displays the value of sigma.
	 * @param out output stream.
	 * @param s state to be represented in the output stream.
	 * @return output stream with sigma already inserted.
	 */
	std::ostream& operator<<(std::ostream &out, const motion_controller_State& s) {
//		out  << "{" << s.Remote_flag << " " << s.Obstacle_distance << " " << s.start_stop_flag << "}";
		return out;
	}

	//! Atomic DEVS model of a Job processor.
	class Motion_controller : public Atomic<motion_controller_State> {
	 private:
		double cmdTime;  //!< Time required by the Processor model to process one Job.
	 public:
		Port<bool> sensor_input;  //!< Input Port for receiving new Job objects.
		Port<bool> sensor_input2;  //!< Output Port for sending processed Job objects.
		Port<bool> sensor_input3;

		Port<bool> Out_flag;

		/**
		 * Constructor function.
		 * @param id ID of the new Processor model object.
		 * @param cmdTime time it takes the Processor to process a Job.
		 */
		Motion_controller(const std::string& id, double cmdTime): Atomic<motion_controller_State>(id, motion_controller_State(cmdTime)) {
			sensor_input = addInPort<bool>("sensor_input");
			sensor_input2 = addInPort<bool>("sensor_input2");
			sensor_input3 = addInPort<bool>("sensor_input3");

			Out_flag = addOutPort<bool>("Out_flag");
		}

		/**
		 * It updates the ProcessorState::clock, clears the ProcessorState::Job being processed, and passivates.
		 * @param s reference to the current state of the model.
		 */
		void internalTransition(motion_controller_State& s) const override {
			s.clock += s.sigma;
			s.sigma = std::numeric_limits<double>::infinity(); //100;
		}

		/**
		 * Updates ProcessorState::clock and ProcessorState::sigma.
		 * If it is idling and gets a new Job via the Processor::inGenerated port, it starts processing it.
		 * @param s reference to the current model state.
		 * @param e time elapsed since the last state transition function was triggered.
		 * @param x reference to the model input port set.
		 */
		void externalTransition(motion_controller_State& s, double e) const override {
			bool x,y,z;
			s.clock += e;
			s.sigma -= e;
//			 std::cout << "motion controller external " << std::endl;
			if(!sensor_input->empty()) //for(const auto y : sensor_input->getBag())
			{
				x = sensor_input->getBag().back(); // const auto x : {
				if (x==0)
					s.Edge_flag = 1;
				else
					s.Edge_flag = 0;
			}
			else if(!sensor_input2->empty())
			{
				x = sensor_input2->getBag().back(); // const auto x : {
				if (x==0)
					s.Edge_flag = 1;
				else
					s.Edge_flag = 0;
			}
			else if(!sensor_input2->empty())
			{
				x = sensor_input2->getBag().back(); // const auto x : {
				if (x==0)
					s.Edge_flag = 1;
				else
					s.Edge_flag = 0;
			}
			s.sigma = cmdTime;
		}

		/**
		 * It outputs the already processed ProcessorState::Job via the Processor::outProcessed port.
		 * @param s reference to the current model state.
		 * @param y reference to the atomic model output port set.
		 */
		void output(const motion_controller_State& s) const override {

			Out_flag ->addMessage (s.Edge_flag);

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
}  //namespace cadmium::example::Edge_robot

#endif //CADMIUM_MOTION_CONTROLLER_HPP_
