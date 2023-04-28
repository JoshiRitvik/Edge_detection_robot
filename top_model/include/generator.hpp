#ifndef _GENERATOR_HPP__
#define _GENERATOR_HPP__

#include <cadmium/core/modeling/atomic.hpp>

#ifndef NO_LOGGING
	#include <iostream>
#endif


#include <cstdlib>

namespace cadmium::example::Edge_robot {
	//! Class for representing the Generator DEVS model state.struct GeneratorState {
	struct GeneratorState {
		double sigma;
		bool val1,val2,val3;
		//! Generator state constructor.
		GeneratorState(): sigma(0), val1(0),val2(0),val3(0)  {}
	};
#ifndef NO_LOGGING
		/**
		 * Insertion operator for GeneratorState objects. It only displays the value of sigma.
		 * @param out output stream.
		 * @param s state to be represented in the output stream.
		 * @return output stream with sigma already inserted.
		 */
		std::ostream& operator<<(std::ostream &out, const GeneratorState& state) {
			out << "Status:, " << state.val1 ; // state to string
			return out;
		}
#endif

	//! Atomic DEVS model of a Generator.
	class Generator : public Atomic<GeneratorState> {
	 private:
		
	 public:
		Port<bool> out1;  
		Port<bool> out2;  
		Port<bool> out3;  
		float a, b;
		int sense1,sense2,sense3;

		/**
		 * Constructor function.
		 * @param id ID of the new Generator model object.
		 */
		Generator(const std::string& id): Atomic<GeneratorState>(id, GeneratorState()) {
			out1 = addOutPort<bool>("out1");
			out2 = addOutPort<bool>("out2");
			out3 = addOutPort<bool>("out3");
			a = 10; b = 20; 
			srand((unsigned) time(NULL));
			sense1 = a + (float)rand()/RAND_MAX * (b-a); // sigma takes random values between 1 and 20
			sense2 = a + (float)rand()/RAND_MAX * (b-a); // sigma takes random values between 1 and 20
			sense3 = a + (float)rand()/RAND_MAX * (b-a); // sigma takes random values between 1 and 20
			if(sense1 > 18)
			 state.val1 = 1;
			 else
			 state.val1 = 0;
			 
			 if(sense2 > 15)
			 state.val2 = 1;
			 else
			 state.val2 = 0;
			 
			 if(sense3 > 10)
			 state.val3 = 1;
			 else
			 state.val3 = 0;

//			printf("[generator] init function\n");
		}

		/**
		 * It updates the GeneratorState::sigma.
		 * @param state reference to the current state of the model.
		 */
		void internalTransition(GeneratorState& state) const override {
			state.sigma = 0.1; // sigma takes random values between 1 and 20 
//			printf("[generator] internal transition function\n");
		}

		/**
		 * Updates GeneratorState::state.
		 * @param state reference to the current model state.
		 * @param e time elapsed since the last state transition function was triggered.
		 * @param x reference to the model input port set.
		 */
		void externalTransition(GeneratorState& state, double e) const override {
			state.sigma = 0.1;
//			printf("[generator] external transition function\n");
		}

		/**
		 * It outputs a 0 value to the out port.
		 * @param state reference to the current model state.
		 * @param y reference to the atomic model output port set.
		 */
		void output(const GeneratorState& state) const override {
			out1->addMessage(state.val1);
			out1->addMessage(state.val2);
			out1->addMessage(state.val3);
			std::cout << " data out" << std::endl;
//			printf("[generator] output function\n");
		}

		/**
		 * It returns the value of GeneratorState::sigma.
		 * @param state reference to the current model state.
		 * @return the sigma value.
		 */
		[[nodiscard]] double timeAdvance(const GeneratorState& state) const override {
			return state.sigma;
		}
	};
}  //namespace cadmium::blinkySystem

#endif //_GENERATOR_HPP__
