// Copyright  (C)  2020  Ruben Smits <ruben dot smits at intermodalics dot eu>

// Version: 1.0
// Author: Dominick Vanthienen <dominick dot vanthienen at mech dot kuleuven dot be>
// Author: Ruben Smits <ruben dot smits at intermodalics dot eu>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
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

#include "dynamic_parameter_solver.hpp"
#include "kdl/frames_io.hpp"
#include <iostream>

namespace KDL {

    Solver_Dynamic_Parameter::Solver_Dynamic_Parameter(const KDL::Chain& _chain, const KDL::Vector _grav, const std::vector<double> _joint_inertia):
            chain(_chain),
            nr(0),
            nj(chain.getNrOfJoints()),
            ns(chain.getNrOfSegments()),
            grav(_grav),
            jntarraynull(nj),
            chainidsolver_coriolis( chain, Vector::Zero()),
            chainidsolver_gravity( chain, grav),
            joint_inertia_(_joint_inertia),
            wrenchnull(ns,Wrench::Zero()),
            X(ns),
            S(ns),
            Ic(ns)
    {
        ag=-Twist(grav,Vector::Zero());
    }

    void Solver_Dynamic_Parameter::updateInternalDataStructures() {
        nj = chain.getNrOfJoints();
        ns = chain.getNrOfSegments();
        jntarraynull.resize(nj);
        chainidsolver_coriolis.updateInternalDataStructures();
        chainidsolver_gravity.updateInternalDataStructures();
        wrenchnull.resize(ns, KDL::Wrench::Zero());
        X.resize(ns);
        S.resize(ns);
        Ic.resize(ns);
    }


    //calculate inertia matrix H
    int Solver_Dynamic_Parameter::JntToMass(const KDL::JntArray &q, KDL::JntSpaceInertiaMatrix& H)
    {
        if(nj != chain.getNrOfJoints() || ns != chain.getNrOfSegments())
            return (error = E_NOT_UP_TO_DATE);
	//Check sizes when in debug mode
        if(q.rows()!=nj || H.rows()!=nj || H.columns()!=nj )
            return (error = E_SIZE_MISMATCH);
        unsigned int k=0;
	double q_;

	//Sweep from root to leaf
        for(unsigned int i=0;i<ns;i++)
	{
	  //Collect RigidBodyInertia
          Ic[i]=chain.getSegment(i).getInertia();
          if(chain.getSegment(i).getJoint().getType()!= KDL::Joint::None)
	  {
	      q_=q(k);
	      k++;
	  }
	  else
	  {
	    q_=0.0;
	  }
	  X[i]=chain.getSegment(i).pose(q_);//Remark this is the inverse of the frame for transformations from the parent to the current coord frame
	  S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));
        }
	//Sweep from leaf to root
    int j, l;
	k = nj - 1; //reset k

    for(int i=ns-1;i>=0;i--)
	{

	  if(i!=0)
	    {
	      //assumption that previous segment is parent
	      Ic[i-1]=Ic[i-1]+X[i]*Ic[i];
	    }

	  F=Ic[i]*S[i];
      if(chain.getSegment(i).getJoint().getType()!=KDL::Joint::None)
	  {
          H(k, k) = dot(S[i], F);
          H(k, k) += joint_inertia_[k];  // add joint inertia
	      j=k; //countervariable for the joints
	      l=i; //countervariable for the segments
	    while(l!=0) //go from leaf to root starting at i
		{
		  //assumption that previous segment is parent
		  F=X[l]*F; //calculate the unit force (cfr S) for every segment: F[l-1]=X[l]*F[l]
		  l--; //go down a segment

          if(chain.getSegment(l).getJoint().getType()!=KDL::Joint::None) //if the joint connected to segment is not a fixed joint
		  {
		    j--;
		    H(k,j)=dot(F,S[l]); //here you actually match a certain not fixed joint with a segment
		    H(j,k)=H(k,j);
		  }
		}
	      k--; //this if-loop should be repeated nj times (k=nj-1 to k=0)
	  }

	}
	return (error = E_NOERROR);
    }

    //calculate coriolis matrix C
    int Solver_Dynamic_Parameter::JntToCoriolis(const KDL::JntArray &q, const KDL::JntArray &q_dot, KDL::JntArray &coriolis)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        SetToZero(jntarraynull);

        //the calculation of coriolis matrix C
        return chainidsolver_coriolis.CartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis);
    }

    //calculate gravity matrix G
    int Solver_Dynamic_Parameter::JntToGravity(const KDL::JntArray &q, KDL::JntArray &gravity)
    {
        //make a null matrix with the size of q_dotdot and a null wrench
        KDL::SetToZero(jntarraynull);
        return chainidsolver_gravity.CartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity);
    }

    Solver_Dynamic_Parameter::~Solver_Dynamic_Parameter(){}
}