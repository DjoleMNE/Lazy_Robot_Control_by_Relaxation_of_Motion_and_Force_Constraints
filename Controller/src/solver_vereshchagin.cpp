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

#include "solver_vereshchagin.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/utilities/svd_eigen_HH.hpp"
/**
 * \brief Dynamics calculations by constraints based on Vereshchagin 1989.
 * This class creates instance of hybrid dynamics solver.
 * The solver calculates total joint space torques and accelerations in a chain 
 * when a constraint force(s) is applied to the chain's 
 * end-effector (task space/cartesian space).
 * This implementation includes only end-effector constraints, but PhD thesis containts solution for all segments.
 */

namespace KDL
{
using namespace Eigen;

/**
 * Constructor for the solver, it will allocate all the necessary memory
 * \param chain The kinematic chain to calculate the inverse dynamics for, an internal copy will be made.
 * \param root_acc The acceleration vector of the root to use during the calculation.(most likely contains gravity)
 */

Solver_Vereshchagin::Solver_Vereshchagin(const Chain& chain_,
                                         const std::vector<double> joint_inertia_, 
                                         const Twist root_acc, 
                                         const unsigned int _nc) :
    chain(chain_), nj(chain.getNrOfJoints()), ns(chain.getNrOfSegments()), nc(_nc),
    results(ns + 1, segment_info(nc))
    //nc -> number of constraints
{
    // Set vector of joint (rotor + gear) inertia: "d" in the algorithm
    assert(joint_inertia_.size() == nj);
    d = Eigen::VectorXd::Map(joint_inertia_.data(), joint_inertia_.size());
    
    //Set root acceleration
    acc_root = root_acc;

    //Provide the necessary memory for computing the inverse of M0
    //It should be made  with constant number of rows/columns..6x6
    //A user can change task in run-time...
    //In this way it would require to call Constructor each time!
    nu_sum.resize(nc);
    ext_torque.resize(nj);
    controlTorque.resize(nj);
    constraintTorque.resize(nj);
    M_0_inverse.resize(nc, nc);
    Um = MatrixXd::Identity(nc, nc);
    Vm = MatrixXd::Identity(nc, nc);
    Sm = VectorXd::Ones(nc);
    tmpm = VectorXd::Ones(nc);
}

/**
 * This method calculates joint space constraint torques and total joint space acceleration.
 * It returns 0 when it succeeds, otherwise -1 or -2 for unmatching matrix and array sizes.
 * Input parameters;
 * \param q The current joint positions
 * \param q_dot The current joint velocities
 * \param f_ext The external forces (no gravity, it is given in root frame) on the segments (from environment!).
 * Output parameters:
 * \param q_dotdot The joint accelerations
 * \param torques the resulting constraint torques for the joints
 *
 * @return error/success code
 */

int Solver_Vereshchagin::CartToJnt(const JntArray &q, const JntArray &q_dot, 
                                   JntArray &q_dotdot, const Jacobian& alfa, 
                                   const JntArray& beta, 
                                   const Wrenches& f_ext_natural, 
                                   const Wrenches& f_ext_virtual, 
                                   JntArray &torques)
{
    if (nj != chain.getNrOfJoints())
        return (error = -3);

    if (ns != chain.getNrOfSegments())
        return (error = -3);

    //Check sizes always
    if (q.rows() != nj || q_dot.rows() != nj || q_dotdot.rows() != nj || torques.rows() != nj || f_ext_natural.size() != ns || f_ext_virtual.size() != ns){
        return (error = -4);
    }

    if (alfa.columns() != nc || beta.rows() != nc){
        return (error = -4);
    }

    //do an upward recursion for position(X) and velocities(X_dot)
    this->initial_upwards_sweep(q, q_dot, q_dotdot, f_ext_natural, f_ext_virtual);

    //do an inward recursion for inertia(H), forces(F) and constraints (U and L)
    this->downwards_sweep(alfa, torques);

    //Solve for the constraint forces(b_N-> beta = ...)
    //equation f) in paper: nu = M_0_inverse*(beta_N - E0_tilde`*acc0 - G0)
    this->constraint_calculation(beta);

    //do an upward recursion to propagate the result
    this->final_upwards_sweep(q_dotdot, torques);
    return (error = E_NOERROR);
}

/**
 *  This method calculates all cartesian space poses, twists, bias accelerations.
 *  External forces are also taken into account in this outward sweep.
 */
void Solver_Vereshchagin::initial_upwards_sweep(const JntArray &q, 
                                                const JntArray &qdot, 
                                                const JntArray &qdotdot, 
                                                const Wrenches& f_ext_natural, 
                                                const Wrenches& f_ext_virtual)
{
    //if (q.rows() != nj || qdot.rows() != nj || qdotdot.rows() != nj || f_ext.size() != ns)
    //        return -1;
    unsigned int j = 0;
    F_total = Frame::Identity();
    for (unsigned int i = 0; i < ns; i++)
    {
        //Express everything in the segments reference frame (body coordinates)
        //which is at the segments tip, i.e. where the next joint is attached.

        //Calculate segment properties: X,S,vj,cj
        const Segment& segment = chain.getSegment(i);
        segment_info& s = results[i + 1];
        //The pose between the joint root and the segment tip (tip expressed in joint root coordinates)
        s.F = segment.pose(q(j)); //X pose of each link in link coord system

        F_total = F_total * s.F; //X pose of the each link in root coord system
        s.F_base = F_total; //X pose of the each link in root coord system for getter functions

        //The velocity due to the joint motion of the segment expressed in the segments reference frame (tip)
        Twist vj = s.F.M.Inverse(segment.twist(q(j), qdot(j))); //XDot of each link
        //Twist aj = s.F.M.Inverse(segment.twist(q(j), qdotdot(j))); //XDotDot of each link

        //The unit velocity due to the joint motion of the segment expressed in the segments reference frame (tip)
        s.Z = s.F.M.Inverse(segment.twist(q(j), 1.0));
        //Put Z in the joint root reference frame:
        s.Z = s.F * s.Z;

        //The total velocity of the segment expressed in the the segments reference frame (tip)
        if (i != 0)
        {
            s.v = s.F.Inverse(results[i].v) + vj; // recursive velocity of each link in segment frame
            //s.A=s.F.Inverse(results[i].A)+aj;
            s.A = s.F.M.Inverse(results[i].A);
        }
        else
        {
            s.v = vj;
            s.A = s.F.M.Inverse(acc_root);
        }
        //c[i] = cj + v[i]xvj (remark: cj=0, since our S is not time dependent in local coordinates)
        //The velocity product acceleration
        //std::cout << i << " Initial upward" << s.v << std::endl;
        s.C = s.v * vj; //This is a cross product: cartesian space BIAS acceleration in local link coord.
        //Put C in the joint root reference frame
        s.C = s.F * s.C; //+F_total.M.Inverse(acc_root));
        //The rigid body inertia of the segment, expressed in the segments reference frame (tip)
        //It is variable type of ArticulatedBodyInertia!!!
        s.H = segment.getInertia();

        //wrench of the rigid body bias forces and external forces on the segment (in body coordinates, tip)
        //external forces are taken into account through s.U.
        Wrench FextLocal = F_total.M.Inverse() * (f_ext_natural[i] + f_ext_virtual[i]);
        s.U = s.v * (s.H * s.v) - FextLocal; //f_ext[i];

        // Save external forces on end_eff for next sweep. Expressed in end_eff tip frame.
        if(i == (int)ns - 1) f_ext_ee = -(F_total.M.Inverse() * f_ext_virtual[i]);

        if (segment.getJoint().getType() != Joint::None)
            j++;
    }

}


/**
 *  F_bias in upward sweep??? Can be computed in first but also in second sweep check explanation in Azamat's thesis!
 *  This method is a force balance sweep. It calculates articulated body inertias and bias forces.
 *  Additionally, acceleration energies generated by bias forces and unit forces are calculated here (U and L). Unit==constraint!
 */
void Solver_Vereshchagin::downwards_sweep(const Jacobian& alfa, const JntArray &torques)
{
    int j = nj - 1;
    for (int i = ns; i >= 0; i--)
    {
        //Get a handle for the segment we are working on.
        segment_info& s = results[i];
        //For segment N,
        //tilde is in the segment refframe (tip, not joint root)
        //without tilde is at the joint root (the childs tip!!!)
        //P_tilde is the articulated body inertia
        //R_tilde is the sum of external and coriolis/centrifugal forces
        //F_ext_ee_tilde are external forces acting on the end-effector segment
        //M is the (unit) acceleration energy already generated at link i (L)
        //G is the (unit) magnitude of the constraint forces at link i (U)
        //E are the (unit) constraint forces due to the constraints (A or alpha here)

        if (i == (int)ns)
        {
            s.P_tilde = s.H;
            s.R_tilde = s.U;
            s.F_ext_ee_tilde = f_ext_ee;
            s.M.setZero();
            s.G.setZero();

            //changeBase(alfa_N,F_total.M.Inverse(),alfa_N2);
            for (unsigned int r = 0; r < 3; r++)
                for (unsigned int c = 0; c < nc; c++)
                {
                    //copy alfa constrain force matrix in E~
                    s.E_tilde(r, c) = alfa(r + 3, c);
                    s.E_tilde(r + 3, c) = alfa(r, c);
                }
        
            //Change the reference frame of alfa to the segmentN tip frame
            //F_Total holds end effector frame, if done per segment bases then constraints could be extended to all segments
            Rotation base_to_end = F_total.M.Inverse();
            for (unsigned int c = 0; c < nc; c++)
            {
                Wrench col(Vector(s.E_tilde(3, c), s.E_tilde(4, c), s.E_tilde(5, c)),
                           Vector(s.E_tilde(0, c), s.E_tilde(1, c), s.E_tilde(2, c)));
                col = base_to_end*col;
                s.E_tilde.col(c) << Vector3d::Map(col.torque.data), Vector3d::Map(col.force.data);
            }
            
            // Gravity acceleration expressed in base frame. First angular than linear part.
            // Vector6d gravity_acc_swap;
            // gravity_acc_swap << Vector3d::Map(acc_root.rot.data), Vector3d::Map(acc_root.vel.data);

            // Djordje: Initializing G matrix with gravitational effects. Compensation for gravity forces on end-effector
            // Djordje: See Popov and Vereshchagin book from 1978, Moscow 
            // s.G.noalias() = s.E_tilde * gravity_acc_swap;
        }
        else
        {
            //For all others:
            //Everything should expressed in the body coordinates of segment i
            segment_info& child = results[i + 1];
            //Copy PZ into a vector so we can do matrix manipulations, put torques above forces
            Vector6d vPZ;
            vPZ << Vector3d::Map(child.PZ.torque.data), Vector3d::Map(child.PZ.force.data);
            Matrix6d PZDPZt;
            PZDPZt.noalias() = vPZ * vPZ.transpose();
            PZDPZt /= child.D;

            //equation a) (see Vereshchagin89) PZDPZt=[I,H;H',M]
            //Azamat:articulated body inertia as in Featherstone (7.19)
            s.P_tilde = s.H + child.P - ArticulatedBodyInertia(PZDPZt.bottomRightCorner<3,3>(), PZDPZt.topRightCorner<3,3>(), PZDPZt.topLeftCorner<3,3>());
            //equation b) (see Vereshchagin89)
            //Azamat: bias force as in Featherstone (7.20)
            s.R_tilde = s.U + child.R + child.PC + (child.PZ / child.D) * child.u;
            
            //Djordje: External force on end-effector. Required for control torque output
            s.F_ext_ee_tilde = child.F_ext_ee - (child.PZ / child.D) * dot(s.Z, child.F_ext_ee);

            //equation c) (see Vereshchagin89)
            s.E_tilde = child.E;

            //Azamat: equation (c) right side term
            s.E_tilde.noalias() -= (vPZ * child.EZ.transpose()) / child.D;

            //equation d) (see Vereshchagin89)
            s.M = child.M;
            //Azamat: equation (d) right side term
            s.M.noalias() -= (child.EZ * child.EZ.transpose()) / child.D;

            //equation e) (see Vereshchagin89)
            s.G = child.G;
            Twist CiZDu = child.C + (child.Z / child.D) * child.u;
            Vector6d vCiZDu;
            vCiZDu << Vector3d::Map(CiZDu.rot.data), Vector3d::Map(CiZDu.vel.data);
            s.G.noalias() += child.E.transpose() * vCiZDu;
        }

        if (i != 0)
        {
            //Transform all results to joint root coordinates of segment i (== body coordinates segment i-1)
            //equation a)
            s.P = s.F * s.P_tilde;
            //equation b)
            s.R = s.F * s.R_tilde;
            // Djordje: Tranformation of the external force acting on end-effector 
            s.F_ext_ee = s.F * s.F_ext_ee_tilde;

            //equation c), in matrix: torques above forces, so switch and switch back
            for (unsigned int c = 0; c < nc; c++)
            {
                Wrench col(Vector(s.E_tilde(3, c), s.E_tilde(4, c), s.E_tilde(5, c)),
                           Vector(s.E_tilde(0, c), s.E_tilde(1, c), s.E_tilde(2, c)));
                col = s.F*col;
                s.E.col(c) << Vector3d::Map(col.torque.data), Vector3d::Map(col.force.data);
            }

            //needed for next recursion
            s.PZ = s.P * s.Z;

            // Djordje: Additionally adding joint (rotor + gear) inertia: equation a) (see Vereshchagin89)
            s.D = d(j) + dot(s.Z, s.PZ);
            // s.D = dot(s.Z, s.PZ);

            s.PC = s.P * s.C;

            //u=(Q-Z(R+PC)=sum of external forces along the joint axes,
            //R are the forces comming from the children,
            //Q is taken zero (do we need to take the previous calculated torques?

            //projection of coriolis and centrepital forces into joint subspace (0 0 Z)
            s.totalBias = -dot(s.Z, s.R + s.PC);
            s.u = torques(j) + s.totalBias;

            //Matrix form of Z
            Vector6d vZ;
            vZ << Vector3d::Map(s.Z.rot.data), Vector3d::Map(s.Z.vel.data);
            s.EZ.noalias() = s.E.transpose() * vZ;

            // const IOFormat fmt(1, DontAlignCols, "\t", " ", "", "", "[", "]");
            // std::cout << "\n ZtE[" << j << "]=" << s.EZ.transpose().format(fmt) << std::endl;

            if (chain.getSegment(i - 1).getJoint().getType() != Joint::None)
                j--;
        }
    }
}

/**
 *  This method calculates constraint force magnitudes.
 *
 */
void Solver_Vereshchagin::constraint_calculation(const JntArray& beta)
{
    JntArray total_beta(6);
    for (int i = 0; i < 6; i++)
        total_beta(i) = beta(i);// + acc_root(i);     
    
    //equation f) nu = M_0_inverse*(beta_N - E0_tilde`*acc0 - G0)
    //M_0_inverse, always nc*nc symmetric matrix
    //std::cout<<"M0: "<<results[0].M<<std::endl;
    //results[0].M-=MatrixXd::Identity(nc,nc);
    //std::cout<<"augmented M0: "<<results[0].M<<std::endl;

    //ToDo: Need to check ill conditions!!!
    //IMPORTANT!!! If M looses its rank, this SVD will not return error.
    //But not full rank of M can mean that given task is not feasible
    //Or initial configuration of the robot is singular!!!!
    //Overall -> even if everthing is ok with initial configuration and
    //task specification, matrix can be ill conditioned
    //which can result in not completely correct results

    //M_0_inverse=results[0].M.inverse();
    // std::cout <<"\n Constraint Coupling Matrix: \n" << results[0].M << '\n';

    int result  = svd_eigen_HH(results[0].M, Um, Sm, Vm, tmpm);
    assert(result == 0);
    // std::cout << "Constraint::SVD: " << result << '\n';

    bool use_dls = false;
    double sigma_min = 0.0;
    double lambda_scaled_square = 0.01;
    // double lambda_scaled_square = 0.0;
    double eps = 1e-6;
    double lambda_max = 1.010;
    // if (use_dls)
    // {
    //     // std::cout << SM.transpose() << std::endl;
    //     // Minimal of six largest singular values is Sm(5), if number of joints >=6, else it's  0
    //     if (nj >= 6 ) sigma_min = Sm(2);
    //     printf("SIMGA min: %.12g \n", sigma_min);

    //     // If sigmaMin > eps, then dls is not active and lambda_scaled = 0 (default value)
    //     // If sigmaMin < eps, then dls is active and lambda_scaled is scaled from 0 to lambda
    //     // Note:  singular values are always positive so sigmaMin >=0
    //     if ( sigma_min < eps )
    //     {
    //         lambda_scaled_square = (1.0 - (sigma_min / eps) * (sigma_min / eps)) * lambda_max * lambda_max;
    //         // lambda_scaled_square = sqrt(1.0 - (sigma_min / eps )*(sigma_min / eps)) * lambda_max;
    //         // lambda_scaled_square = lambda_scaled_square * lambda_scaled_square;
    //     }
    //     printf("Lambda: %.12g", lambda_scaled_square);
    // }

    //truncated svd, what would sdls, dls physically mean?
    // printf("\n Singular: ");
    for (unsigned int i = 0; i < nc; i++)
    {   
        if(use_dls && nj >= 6)
        {
            Sm(i) = Sm(i) / (Sm(i) * Sm(i) + lambda_scaled_square);
            // printf("%d: %.10g,  ", i, Sm(i));
        }
        else
        {
            if (Sm(i) < 1e-8)
            {
                // printf("%d: %f,  ", i, Sm(i));
                Sm(i) = 0.0;
            } 
            else Sm(i) = 1 / Sm(i);
        }
    }
    // printf("\n \n");

    results[0].M.noalias() = Vm * Sm.asDiagonal();
    M_0_inverse.noalias() = results[0].M * Um.transpose();
    //results[0].M.ldlt().solve(MatrixXd::Identity(nc,nc),&M_0_inverse);
    //results[0].M.computeInverse(&M_0_inverse);
    Vector6d acc;
    acc << Vector3d::Map(acc_root.rot.data), Vector3d::Map(acc_root.vel.data);

    nu_sum.noalias() = -(results[0].E_tilde.transpose() * acc);
    //nu_sum.setZero();
    nu_sum += total_beta.data;
    nu_sum -= results[0].G;

    //equation f) nu = M_0_inverse*(beta_N - E0_tilde`*acc0 - G0)
    nu.noalias() = M_0_inverse * nu_sum;
}

/**
 *  This method puts all acceleration contributions (constraint, bias, nullspace and parent accelerations) together.
 *
 */
void Solver_Vereshchagin::final_upwards_sweep(JntArray &q_dotdot, JntArray &torques)
{
    unsigned int j = 0;

    for (unsigned int i = 1; i <= ns; i++)
    {
        segment_info& s = results[i];
        //Calculation of joint and segment accelerations
        //equation g) qdotdot[i] = D^-1*(Q - Z'(R + P(C + acc[i-1]) + E*nu))
        // = D^-1(u - Z'(P*acc[i-1] + E*nu)
        Twist a_g;
        Twist a_p;

        if (i == 1)
        {
            a_p = acc_root;
        }
        else
        {
            a_p = results[i - 1].acc;
        }

        //The contribution of the constraint forces at segment i
        Vector6d tmp = s.E * nu;
        Wrench constraint_force = Wrench(Vector(tmp(3), tmp(4), tmp(5)),
                                         Vector(tmp(0), tmp(1), tmp(2)));

        //acceleration components are also computed
        //Contribution of the acceleration of the parent (i-1)
        Wrench parent_force = s.P * a_p;
        double parent_forceProjection = -dot(s.Z, parent_force);
        double parentAccComp = parent_forceProjection / s.D;

        // Compute torques felt in joints due to ext force acting on end effector
        // I.e. projection of ext force into joint subspace (0 0 Z)
        ext_torque(j) = -dot(s.Z, s.F_ext_ee);

        //The constraint force and acceleration force projected on the joint axes -> axis torque/force
        double constraint_torque = -dot(s.Z, constraint_force);
        //The result should be the torque at this joint.
        constraintTorque(j) = constraint_torque;

        //Summing 2 contributions for true control torque:
        controlTorque(j) = constraintTorque(j) + ext_torque(j);
        // controlTorque(j) = ext_torque(j);

        s.constAccComp = constraint_torque / s.D;
        s.nullspaceAccComp = s.u / s.D;

        //total joint space acceleration resulting from accelerations of parent joints, constraint forces and
        // nullspace forces.
        //equation g) qdotdot[i] = D^-1(u - Z'(P*acc[i-1] + E*nu) Vereshchagin89'
        q_dotdot(j) = (s.nullspaceAccComp + parentAccComp + s.constAccComp);

        //returns acceleration in link distal tip coordinates.
        //For use needs to be transformed to root(base) reference frame
        //Check two getters for this
        s.acc = s.F.Inverse(a_p + s.Z * q_dotdot(j) + s.C);

        if (chain.getSegment(i - 1).getJoint().getType() != Joint::None)
            j++;
    }
}

// Returns Cartesian pose of links in robot base coordinates
void Solver_Vereshchagin::get_transformed_link_pose(Frames& x)
{
    assert(x.size() == ns);

    for (int i = 0; i < ns; i++) {
        x[i] = results[i + 1].F_base;
    }
}

// Returns Cartesian velocity of links in robot base coordinates and base reference point 
// Tranformation from base-fixed twist to screw twist?
void Solver_Vereshchagin::get_screw_twist(Twists& xDot)
{
    assert(xDot.size() == ns);

    for (int i = 0; i < ns; i++) {
        xDot[i] = results[i + 1].F_base * results[i + 1].v;
    }
}

// Returns Cartesian velocity of links in robot base coordinates. 
// Returns Pose twist?
void Solver_Vereshchagin::get_transformed_link_velocity(Twists& xDot)
{
    assert(xDot.size() == ns);

    for (int i = 0; i < ns; i++) {
        xDot[i] = results[i + 1].F_base.M * results[i + 1].v;
    }
}

// Returns Cartesian acceleration of links in link's tip coordinates
void Solver_Vereshchagin::get_link_acceleration(Twists& xDotdot)
{
    assert(xDotdot.size() == ns + 1);
    xDotdot[0] = acc_root;
    for (int i = 1; i < ns + 1; i++) {
        xDotdot[i] = results[i].acc;
    }
}

// Returns Cartesian acceleration of links in robot base coordinates
void Solver_Vereshchagin::get_transformed_link_acceleration(Twists& xDotdot)
{
    assert(xDotdot.size() == ns + 1);
    xDotdot[0] = acc_root;
    for (int i = 1; i < ns + 1; i++) {
        xDotdot[i] = results[i].F_base.M * results[i].acc;
    }
}

//H -> Rigid Body Inertia of the segment!!
//expressed in the segments reference frame (tip)
//but variable Type is ArticulatedBodyInertia!
void Solver_Vereshchagin::get_link_inertias(Inertias &h)
{
    assert(h.size() == ns + 1);

    for (int i = 0; i < ns + 1; i++) {
        h[i] = results[i].H;
    }
}

void Solver_Vereshchagin::get_bias_force(Wrenches &bias)
{
    assert(bias.size() == ns + 1);

    for (int i = 0; i < ns + 1; i++) {
        bias[i] = results[i].U;
    }
}

void Solver_Vereshchagin::get_control_torque(JntArray &tau_control)
{
    assert(tau_control.rows() == controlTorque.rows());
    tau_control = controlTorque;
}

void Solver_Vereshchagin::get_constraint_torque(JntArray &tau_constraint)
{
    assert(tau_constraint.rows() == constraintTorque.rows());
    tau_constraint = constraintTorque;
}

void Solver_Vereshchagin::get_constraint_magnitude(Eigen::VectorXd &nu_)
{
    assert(nu_.rows() == nc);
    nu_ = nu;
}

}//namespace
