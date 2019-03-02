/*
Author: Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Copyright (c) [2019]
*/

#ifndef LWR_KDL_MODEL_HPP
#define LWR_KDL_MODEL_HPP
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>

class lwr_kdl_model
{
	public:
		lwr_kdl_model(KDL::Chain &robot_chain);
		~lwr_kdl_model(){}


	private:
		void createModel(KDL::Chain &robot_chain);
};
#endif /* LWR_KDL_MODEL_HPP */
