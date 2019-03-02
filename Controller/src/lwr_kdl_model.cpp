#include <lwr_kdl_model.hpp>

lwr_kdl_model::lwr_kdl_model(KDL::Chain &robot_chain)
{
    createModel(robot_chain);
}

void lwr_kdl_model::createModel(KDL::Chain &robot_chain)
{
	//KDL::Joint 0
	// robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
    //                                KDL::Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0) //KDL::Frame 1
    //                               ));

	//KDL::Joint 1
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                   KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse() * KDL::RigidBodyInertia(2,
								                       KDL::Vector::Zero(),
												       KDL::RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0)))); //KDL::Frame 2
				   
	//KDL::Joint 2 
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                                   KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse() * KDL::RigidBodyInertia(2,
											     	   KDL::Vector(0.0,-0.3120511,-0.0038871),
											     	   KDL::RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828)))); //KDL::Frame 3
				  
	//KDL::Joint 3
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                   KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse() * KDL::RigidBodyInertia(2,
												       KDL::Vector(0.0,-0.0015515,0.0),
												       KDL::RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147)))); //KDL::Frame 4
				  
	//KDL::Joint 4
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                                   KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse() * KDL::RigidBodyInertia(2,
								     				   KDL::Vector(0.0,0.5216809,0.0),
									    			   KDL::RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324)))); //KDL::Frame 5
				  
	//KDL::Joint 5
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                                   KDL::Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse() * KDL::RigidBodyInertia(2,
									      			   KDL::Vector(0.0,0.0119891,0.0),
										     		   KDL::RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226)))); //KDL::Frame 6
				  
	//KDL::Joint 6
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                                   KDL::Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse() * KDL::RigidBodyInertia(2,
												       KDL::Vector(0.0,0.0080787,0.0),
												       KDL::RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101)))); //KDL::Frame 7
	//KDL::Joint 7
	robot_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                   KDL::Frame::Identity(),
                                   KDL::RigidBodyInertia(2,
												    KDL::Vector::Zero(),
												    KDL::RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0)))); //KDL::Frame 8
}