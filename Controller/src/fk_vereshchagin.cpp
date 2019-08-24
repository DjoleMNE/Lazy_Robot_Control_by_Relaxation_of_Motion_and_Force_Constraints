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

#include "fk_vereshchagin.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/utilities/svd_eigen_HH.hpp"
/**
 * \brief Dynamics calculations by constraints based on Vereshchagin 1989.
 * for a chain. This class creates instance of hybrid dynamics solver.
 * The solver calculates total joint space accelerations in a chain when a constraint force(s) is applied
 * to the chain's end-effector (task space/cartesian space).
 * This implementation includes only end-effector constraints, but PhD thesis containts solution for all segments.
 */

using namespace Eigen;

/**
 * Constructor for the solver, it will allocate all the necessary memory
 * \param chain The kinematic chain to calculate the inverse dynamics for, an internal copy will be made.
 */

KDL::FK_Vereshchagin::FK_Vereshchagin(const KDL::Chain& chain_) :
    chain(chain_), nj(chain.getNrOfJoints()), 
    ns(chain.getNrOfSegments()), nc(0),
    results(ns + 1, segment_info(nc))
    //nc -> number of constraints
{

}

/**
 * This method does an upward recursion for position(X) and velocities(X_dot)
 * This method returns 0 when it succeeds,
 * otherwise -1 or -2 for unmatching matrix and array sizes.
 * Input parameters;
 * \param q The current joint positions
 * \param q_dot The current joint velocities
 * @return error/success code
 */

int KDL::FK_Vereshchagin::JntToCart(const KDL::JntArray &q, const KDL::JntArray &q_dot, 
                                    Frames& cart_pose, Twists& cart_vel)
{
    if (nj != chain.getNrOfJoints()) return (error = -3);
    if (ns != chain.getNrOfSegments()) return (error = -3);

    //Check sizes always
    if (q.rows() != nj || q_dot.rows() != nj) return (error = -4);
    if (cart_pose.size() != ns || cart_vel.size() != ns) return (error = -4);

    //do an upward recursion for position(X) and velocities(X_dot)
    this->upwards_sweep(q, q_dot, cart_pose, cart_vel);
    return (error = E_NOERROR);
}

/**
 *  This method calculates all cartesian space poses and twists
 */
void KDL::FK_Vereshchagin::upwards_sweep(const KDL::JntArray &q, const KDL::JntArray &qdot, 
                                         Frames& cart_pose, Twists& cart_vel)
{
    unsigned int j = 0;
    F_total = KDL::Frame::Identity();

    for (unsigned int i = 0; i < ns; i++)
    {
        //Express everything in the segments reference frame (body coordinates)
        //which is at the segments tip, i.e. where the next joint is attached.

        //Calculate segment properties: X,S,vj
        const KDL::Segment& segment = chain.getSegment(i);
        segment_info& s = results[i + 1];

        //The pose between the joint root and the segment tip (tip expressed in joint root coordinates)
        s.F = segment.pose(q(j)); //X pose of each link in link coord system

        F_total = F_total * s.F; //X pose of the each link in root coord system
        s.F_base = F_total; //X pose of the each link in root coord system for getter functions

        //Saves cartesian pose of links in robot base coordinates
        cart_pose[i] = s.F_base; // Save link pose for the output 

        //The velocity due to the joint motion of the segment expressed in the segments reference frame (tip)
        KDL::Twist vj = s.F.M.Inverse(segment.twist(q(j), qdot(j))); //XDot of each link

        //The total velocity of the segment expressed in the the segments reference frame (tip)
        if (i != 0)
        {
            s.v = s.F.Inverse(results[i].v) + vj; // recursive velocity of each link in segment frame
        }
        else
        {
            s.v = vj;
        }

        //Saves cartesian velocity of links.
        
        // Pose twist?
        cart_vel[i] = s.F_base.M * s.v; // Save link vel for the output 

        // Screw Twist?
        // cart_vel[i] = s.F_base * s.v; // Save link vel for the output 
 
        // Body-Fixed Twist?
        // cart_vel[i] = s.v; // Save link vel for the output 

        if (segment.getJoint().getType() != KDL::Joint::None)
            j++;
    }
}

