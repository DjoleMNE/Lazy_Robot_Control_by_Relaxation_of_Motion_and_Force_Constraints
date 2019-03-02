/*
Author: Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Copyright (c) [2019]
*/

#ifndef YOUBOT_CUSTOM_MODEL_HPP
#define YOUBOT_CUSTOM_MODEL_HPP
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

class youbot_custom_model
{
	public:
		youbot_custom_model(KDL::Chain &arm_chain);
		~youbot_custom_model(){}

		void createModel(KDL::Chain &arm_chain);

	private:
		// Number of bodies and joints in the manipulator
		const int NUMBER_OF_JOINTS_;
        const int NUMBER_OF_BODIES_;
        const int ID_BODY_BASE;
        const int ID_BODY_LINK_1;
        const int ID_BODY_LINK_2;
        const int ID_BODY_LINK_3;
        const int ID_BODY_LINK_4;
        const int ID_BODY_LINK_5;
        const int ID_BODY_GRIPPER_BASE;
        
		KDL::Frame createTransform(double origin_x, double origin_y, 
                                   double origin_z, double gamma, 
                                   double beta, double alpha);
};
#endif /* YOUBOT_CUSTOM_MODEL_HPP */
