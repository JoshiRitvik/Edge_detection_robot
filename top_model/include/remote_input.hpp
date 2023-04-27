#ifndef CADMIUM_REMOTE_INPUT_HPP_
#define CADMIUM_REMOTE_INPUT_HPP_

#include <cadmium/core/modeling/atomic.hpp>
#include <iostream>

namespace cadmium::example::Edge_robot {
	struct Remote_control {
		double clock;  //!< Current simulation time.
		double sigma;  //!< Time to wait before triggering the next internal transition function.
        bool remote_flag;
		//! Constructor function. It sets all the attributes to 0.
		Remote_control(): clock(), sigma(),remote_flag() {} 

	};

    std::ostream& operator<<(std::ostream& out, const Remote_control& s) {
		out << s.remote_flag;
		return out;
    }

    class Remote_control_In : public Atomic <Remote_control> {
     private:
	 	double toggleTime;
	 public:
		Port<bool> Ext_Remote_control;  //!< Output Port for sending new Job objects to be processed.
		Remote_control_In(const std::string& id, double toggleTime): Atomic<Remote_control>(id, Remote_control()) {
            Ext_Remote_control = addOutPort<bool>("Ext_Remote_control");
		}
		void internalTransition(Remote_control& s) const override {
			s.clock += s.sigma;
			s.sigma = double(100);
            s.remote_flag = !s.remote_flag;;//!s.remote_flag;
		}

		void externalTransition(Remote_control& s, double e) const override {
			s.clock += e;
			s.sigma = std::max(s.sigma - e, 0.);
		}
        void output(const Remote_control& s) const override {
 			Ext_Remote_control->addMessage(s.remote_flag);
            std::cout << " Remote_control_In " << std::endl;
		}
        [[nodiscard]] double timeAdvance(const Remote_control& s) const override {
			return s.sigma;
		}

    };
}
#endif
