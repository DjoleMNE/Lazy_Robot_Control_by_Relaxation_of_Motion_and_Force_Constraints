/**
 * This is the ROS node class for the Hybrid Dynamics Solver.
 * It provides the connection between the ROS environment
 * and the Solver_Vereshchagin class.
 */

#ifndef MCR_HYBRID_DYNAMICS_SOLVER_HPP_
#define MCR_HYBRID_DYNAMICS_SOLVER_HPP_
#include <solver_vereshchagin.hpp>
#include <motion_specification.hpp>


class dynamics_controller
{

    public:
        // KDL::Solver_Vereshchagin &hd_solver_;
        // KDL::Chain arm_chain_;
        // motion_specification motion_;
        // const int NUMBER_OF_CONSTRAINTS = 6;
        std::vector<double> joint_limits_;

        dynamics_controller();
        ~dynamics_controller();
        // void init_motion(const KDL::Chain &arm_chain_, motion_specification &motion_, unsigned int &NUMBER_OF_CONSTRAINTS);
        
    private:

};

#endif /* MCR_HYBRID_DYNAMICS_SOLVER_HPP_*/
