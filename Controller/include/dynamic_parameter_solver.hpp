// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>

// Version: 1.0
// Author: Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef SOLVER_DYNAMIC_PARAMETER_HPP
#define SOLVER_DYNAMIC_PARAMETER_HPP

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/articulatedbodyinertia.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <Eigen/StdVector>

namespace KDL {

    /**
     * Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) 
     * for the calculation torques out of the pose and derivatives.
     * (inverse dynamics)
     *
     * The algorithm implementation for H is based on the book "Rigid Body
     * Dynamics Algorithms" of Roy Featherstone, 2008
     * (ISBN:978-0-387-74314-1) See page 107 for the pseudo-code.
     * This algorithm is extended for the use of fixed joints
     *
     * It calculates the joint-space inertia matrix, given the motion of
     * the joints (q,qdot,qdotdot), external forces on the segments
     * (expressed in the segments reference frame) and the dynamical
     * parameters of the segments.
     */
    class Solver_Dynamic_Parameter : public SolverI
    {
        public:
        Solver_Dynamic_Parameter(const KDL::Chain& chain, const KDL::Vector _grav, const std::vector<double> _joint_inertia);
        virtual ~Solver_Dynamic_Parameter();

        virtual int JntToCoriolis(const KDL::JntArray &q, const KDL::JntArray &q_dot, KDL::JntArray &coriolis);
        virtual int JntToMass(const KDL::JntArray &q, KDL::JntSpaceInertiaMatrix& H);
        virtual int JntToGravity(const KDL::JntArray &q, KDL::JntArray &gravity);

        /// @copydoc KDL::SolverI::updateInternalDataStructures()
        virtual void updateInternalDataStructures();

        private:
        const KDL::Chain& chain;
        int nr;  // unused, remove in a future version
        unsigned int nj;
        unsigned int ns;	
        KDL::Vector grav;
        KDL::Vector vectornull;
        KDL::JntArray jntarraynull;
        KDL::ChainIdSolver_RNE chainidsolver_coriolis;
        KDL::ChainIdSolver_RNE chainidsolver_gravity;
        const std::vector<double> joint_inertia_;
        std::vector<KDL::Wrench> wrenchnull;
        std::vector<KDL::Frame> X;
        std::vector<KDL::Twist> S;
        //std::vector<RigidBodyInertia> I;
        std::vector<KDL::ArticulatedBodyInertia, Eigen::aligned_allocator<KDL::ArticulatedBodyInertia> > Ic;
        KDL::Wrench F;
        KDL::Twist ag;
    };
}

#endif
