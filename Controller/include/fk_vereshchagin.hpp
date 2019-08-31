// Copyright  (C)  2009, 2011

// Version: 1.0
// Author: Ruben Smits, Herman Bruyninckx, Azamat Shakhimardanov
// Maintainer: Ruben Smits, Azamat Shakhimardanov
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

#ifndef KDL_FK_VERESHCHAGIN_HPP
#define KDL_FK_VERESHCHAGIN_HPP

#include "kdl/chainidsolver.hpp"
#include "kdl/solveri.hpp"
#include "kdl/frames.hpp"
#include "kdl/articulatedbodyinertia.hpp"

#include<Eigen/StdVector>

namespace KDL
{
    /**
     * \brief Dynamics calculations by constraints based on Vereshchagin 1989.
     * for a chain. This class creates instance of hybrid dynamics solver.
     * The solver calculates total joint space accelerations in a chain when a constraint force(s) is applied
     * to the chain's end-effector (task space/cartesian space).
     */

    class FK_Vereshchagin: public KDL::SolverI
    {
        typedef std::vector<Twist> Twists;
        typedef std::vector<Frame> Frames;

    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param chain The kinematic chain to calculate the inverse dynamics for, an internal copy will be made.
         * \param root_acc The acceleration vector of the root to use during the calculation.(most likely contains gravity)
         *
         */
        FK_Vereshchagin(const Chain& chain_);

        ~FK_Vereshchagin()
        {
        };

        /**
         * This method does an upward recursion for position(X) and velocities(X_dot)
         * This method returns 0 when it succeeds,
         * otherwise -1 or -2 for unmatching matrix and array sizes.
         * Input parameters;
         * \param q The current joint positions
         * \param q_dot The current joint velocities
         * @return error/success code
         */
        int JntToCart(const JntArray &q, const JntArray &q_dot,
                      Frames& cart_pose, Twists& cart_vel);

        /// @copydoc KDL::SolverI::updateInternalDataStructures
        virtual void updateInternalDataStructures() {};

    private:
        /**
         *  This method calculates all cartesian space poses, twists.
         */
        void upwards_sweep(const JntArray &q, const JntArray &q_dot, 
                           Frames& cart_pose, Twists& cart_vel);

    private:
        const Chain& chain;
        const unsigned int nj;
        const unsigned int ns;
        const unsigned int nc;

        Frame F_total;

        struct segment_info
        {
            Frame F; //local pose with respect to previous link in segments coordinates
            Frame F_base; // pose of a segment in root coordinates
            Twist Z; //Unit twist
            Twist v; //twist

            segment_info(unsigned int nc)
            {
            };
        };

        std::vector<segment_info, Eigen::aligned_allocator<segment_info> > results;

    };
} //namespace
#endif
