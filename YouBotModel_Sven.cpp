#include <YouBotModel.hpp>
#include <iostream>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>


#define KILOGRAM(x) (x)
#define KILOGRAM_SQUARE_METRE(x) (x)
#define DEGREE(x) (x)
#define METRE(x) (x)
#define NEWTON_METRE(x) (x)
#define NEWTON_METRE_PER_RADIAN(x) (x)
#define NEWTON_METRE_SECOND_PER_RADIAN(x) (x)
#define DEGREE_PER_SECOND(x) (x)
#define SECOND_PER_RADIAN(x) (x)

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0


/*
#include <Eigen/Core>

using namespace Eigen;
KDL::RigidBodyInertia transform_inertia(
        const KDL::Frame &T, const KDL::RigidBodyInertia &I)
{
    // From (Featherstone2008):
    // mb = ma
    // hb = R*(h-m*r)
    // Ib = R(Ia+r x h x + (h-m*r) x r x)R'
    //
    // But consider for two coordinate systems call A and B the conventions used
    // in
    // 1. Featherstone (for the above formula and its implementation below):
    //    a) The translation vector "goes" from A's origin to B's origin and is
    //       expressed in A's coordinates
    //    b) The orientation expresses how 3D vectors are transformed from A to
    //       B
    // vs.
    // 2. Craig (KDL's frame representation):
    //    a) The translation vector "goes" from A's origin to B's origin and is
    //       expressed in A's coordinates
    //    b) The orientation expresses how 3D vectors are transformed from B to
    //       A

    KDL::Frame X = T.Inverse();
    KDL::Vector hmr = (I.h - I.m * X.p);
    Vector3d r_eig = Map<Vector3d>(X.p.data);
    Vector3d h_eig = Map<const Vector3d>(I.h.data);
    Vector3d hmr_eig = Map<Vector3d>(hmr.data);
    Matrix3d rcrosshcross = h_eig * r_eig.transpose() - r_eig.dot(h_eig) * Matrix3d::Identity();
    Matrix3d hmrcrossrcross = r_eig * hmr_eig.transpose() - hmr_eig.dot(r_eig) * Matrix3d::Identity();
    Matrix3d R = Map<Matrix3d>(T.M.Inverse().data);
    KDL::RotationalInertia Ib;
    Map<Matrix3d>(Ib.data) = R*((Map<const Matrix3d>(I.I.data) + rcrosshcross + hmrcrossrcross) * R.transpose());
    
    return KDL::RigidBodyInertia(I.m, T.M * hmr, Ib, mhi);
}
*/


YouBotModel::YouBotModel() : NUMBER_OF_BODIES(7), NUMBER_OF_JOINTS(5)
{
    createModel();
}


YouBotModel::~YouBotModel()
{
}


KDL::Frame YouBotModel::createTransform(
        double origin_x, double origin_y, double origin_z,
        double gamma, double beta, double alpha)
{
    // The youBot specification uses extrinsic Tait-Bryan angles with the ZYX
    // sequence
    double ca = cos(alpha);
    double sa = sin(alpha);
    double cb = cos(beta);
    double sb = sin(beta);
    double cc = cos(gamma);
    double sc = sin(gamma);

    return KDL::Frame(
            KDL::Rotation(
                            cb * cc    ,           -cb * sc     ,     sb  ,
                 sa * sb * cc + ca * sc, -sa * sb * sc + ca * cc, -sa * cb,
                -ca * sb * cc + sa * sc,  ca * sb * sc + sa * cc,  ca * cb),
            KDL::Vector(origin_x, origin_y, origin_z));
}


void YouBotModel::createModel()
{
    //
    // Body parameters
    //
    std::vector<KDL::Frame> joint_to_tip(NUMBER_OF_BODIES);
    joint_to_tip[ID_BODY_BASE        ] = createTransform(METRE(0.024), METRE(0.0),    METRE(0.115),   DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE( 180.0)));
    joint_to_tip[ID_BODY_LINK_1      ] = createTransform(METRE(0.033), METRE(0.0),    METRE(0.0),     DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(  90.0)));
    joint_to_tip[ID_BODY_LINK_2      ] = createTransform(METRE(0.155), METRE(0.0),    METRE(0.0),     DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    joint_to_tip[ID_BODY_LINK_3      ] = createTransform(METRE(0.0),   METRE(0.135),  METRE(0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    joint_to_tip[ID_BODY_LINK_4      ] = createTransform(METRE(0.0),   METRE(0.1136), METRE(0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(- 90.0)));
    joint_to_tip[ID_BODY_LINK_5      ] = createTransform(METRE(0.0),   METRE(0.0),    METRE(0.05716), DEG_TO_RAD(DEGREE( 180.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));
    joint_to_tip[ID_BODY_GRIPPER_BASE] = createTransform(METRE(0.0),   METRE(0.0),    METRE(0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(0.0)), DEG_TO_RAD(DEGREE(   0.0)));

    // The pose of the "inertia frame". The inertia is measured about the origin of this frame and expressed in the frame's axes.
    // See: https://simtk.org/api_docs/molmodel/api_docs22/Simbody/html/classSimTK_1_1Inertia__.html
    // For the youBot store specification it holds that:
    // - The frame's origin is the centre of mass (i.e. the coordinates describe the "central inertia")
    // - The frame's axes are the principal axes (i.e. the coordinates describe the "principal moments of inertia")
    std::vector<KDL::Frame> joint_to_inertia(NUMBER_OF_BODIES);
    joint_to_inertia[ID_BODY_BASE        ] = createTransform(METRE(0.0),     METRE(0.0),     METRE( 0.0),     DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE(  0.0)));
    joint_to_inertia[ID_BODY_LINK_1      ] = createTransform(METRE(0.01516), METRE(0.00359), METRE( 0.03105), DEG_TO_RAD(DEGREE( 180.0)), DEG_TO_RAD(DEGREE( 20.0)), DEG_TO_RAD(DEGREE(  0.0)));
    joint_to_inertia[ID_BODY_LINK_2      ] = createTransform(METRE(0.11397), METRE(0.015),   METRE(-0.01903), DEG_TO_RAD(DEGREE(- 90.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE(-90.0)));
    joint_to_inertia[ID_BODY_LINK_3      ] = createTransform(METRE(0.00013), METRE(0.10441), METRE( 0.02022), DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE( 90.0)));
    joint_to_inertia[ID_BODY_LINK_4      ] = createTransform(METRE(0.00015), METRE(0.05353), METRE(-0.02464), DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE(180.0)), DEG_TO_RAD(DEGREE(  0.0)));
    joint_to_inertia[ID_BODY_LINK_5      ] = createTransform(METRE(0.0),     METRE(0.0012),  METRE(-0.01648), DEG_TO_RAD(DEGREE(   0.0)), DEG_TO_RAD(DEGREE( 90.0)), DEG_TO_RAD(DEGREE(  0.0)));
    joint_to_inertia[ID_BODY_GRIPPER_BASE] = createTransform(METRE(0.0),     METRE(0.0),     METRE( 0.0289),  DEG_TO_RAD(DEGREE( 180.0)), DEG_TO_RAD(DEGREE(  0.0)), DEG_TO_RAD(DEGREE( 90.0)));

    // The rotational inertia's coordinate representation:
    // - Central: the point about which the inertia is measured is the centre of mass
    // - Principal moments of inertia: the inertia is expressed about the principal axes (i.e. it is represented by a diagonal matrix)
    std::vector<KDL::RotationalInertia> central_principal_moments_of_inertia(NUMBER_OF_BODIES);
    central_principal_moments_of_inertia[ID_BODY_BASE        ] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001));
    central_principal_moments_of_inertia[ID_BODY_LINK_1      ] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.0029525),  KILOGRAM_SQUARE_METRE(0.0060091),  KILOGRAM_SQUARE_METRE(0.0058821));
    central_principal_moments_of_inertia[ID_BODY_LINK_2      ] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.0031145),  KILOGRAM_SQUARE_METRE(0.0005843),  KILOGRAM_SQUARE_METRE(0.0031631));
    central_principal_moments_of_inertia[ID_BODY_LINK_3      ] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.00172767), KILOGRAM_SQUARE_METRE(0.00041967), KILOGRAM_SQUARE_METRE(0.00184680));
    central_principal_moments_of_inertia[ID_BODY_LINK_4      ] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.0006764),  KILOGRAM_SQUARE_METRE(0.0010573),  KILOGRAM_SQUARE_METRE(0.0006610));
    central_principal_moments_of_inertia[ID_BODY_LINK_5      ] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.0001934),  KILOGRAM_SQUARE_METRE(0.0001602),  KILOGRAM_SQUARE_METRE(0.0000689));
    central_principal_moments_of_inertia[ID_BODY_GRIPPER_BASE] = KDL::RotationalInertia(KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001),    KILOGRAM_SQUARE_METRE(0.00001));

    std::vector<double> mass(NUMBER_OF_BODIES); // [kg]
    mass[ID_BODY_BASE        ] = KILOGRAM(0.961);
    mass[ID_BODY_LINK_1      ] = KILOGRAM(1.390);
    mass[ID_BODY_LINK_2      ] = KILOGRAM(1.318);
    mass[ID_BODY_LINK_3      ] = KILOGRAM(0.821);
    mass[ID_BODY_LINK_4      ] = KILOGRAM(0.769);
    mass[ID_BODY_LINK_5      ] = KILOGRAM(0.687);
    mass[ID_BODY_GRIPPER_BASE] = KILOGRAM(0.199);


    // Create segments
    for (int i = 0; i < NUMBER_OF_BODIES - 1; i++) {
        // KDL vs. youBot inertia specification
        // ------------------------------------
        // In KDL the inertia is provided w.r.t. the segment's reference
        // frame. The joint frame is also provided w.r.t. this reference
        // frame. Thus, a joint only "feels" the inertia of the more distal
        // _segments_ (not of the "current" segment).
        //
        // Update: this is actually not true, as one can see in a 1-DOF example.
        //         Indeed the inertia is attached w.r.t. the current segment's
        //         tip frame!
        //
        // In contrast, the youBot specification provides the inertia in the
        // (moving) joint frame.
        // Additionally, KDL expects the rotational inertia, that is provided to
        // the rigid body inertia, to be expressed in (the orientation of) the
        // segment's reference frame. In general, this is different from the
        // principal axes.
        //
        // To accomodate for those differences, the KDL segment is created
        // with the inertia values of the "predecessor" segment which must be
        // appropriately transformed to from the current segment's base frame.

        KDL::RigidBodyInertia inertia;

        // Only mass and centre of mass
        inertia = KDL::RigidBodyInertia(mass[i], joint_to_tip[i].Inverse() * joint_to_inertia[i].p);

/*
        // Rotational inertia (i) measured in centre of mass point; and (ii) expressed in inertial frame
        KDL::RigidBodyInertia i_com_in_inertia(0.0, KDL::Vector(0, 0, 0), central_principal_moments_of_inertia[i]);
        // Rotational inertia (i) measured in centre of mass point; and (ii) expressed in segment's joint frame
        KDL::RigidBodyInertia i_com_in_joint = joint_to_inertia[i].M * i_com_in_inertia;  // No inverse required here (according to KDL documentation for Rotation*Inertia)!!!

        // Position vector (i) from joint frame's origin to centre of mass point; and (ii) expressed in segment's joint frame
        KDL::Vector joint_to_com_in_joint = joint_to_inertia[i].p;

        // Inertia (i) measured in joint frame's origin; and (ii) expressed in segment's joint frame
        KDL::RigidBodyInertia i_joint_in_joint = KDL::RigidBodyInertia(mass[i], joint_to_com_in_joint, i_com_in_joint.getRotationalInertia());

        inertia = joint_to_tip[i].Inverse() * i_joint_in_joint; // transform to joint frame
*/
        

        std::cout << joint_to_inertia[i].p << std::endl;
        std::cout << inertia.getCOG() << std::endl;
        //for (int x = 0; x < 9; x++) std::cout << inertia.getRotationalInertia().data[x] << ", ";
        //std::cout << std::endl;

        /*
        inertia = KDL::RigidBodyInertia(mass[i], KDL::Vector(0, 0, 0), central_principal_moments_of_inertia[i]);
        inertia = joint_to_inertia[i].Inverse() * inertia;
        inertia = joint_to_tip[i].Inverse() * inertia;
        */

        /*
        // Start with pure "rotational inertia" about principal axes (and measured in centre of mass)
        KDL::RigidBodyInertia inertia_pa(0, KDL::Vector(0, 0, 0), central_principal_moments_of_inertia[i]);
        // Re-express in joint frame
        KDL::RigidBodyInertia inertia_jf = joint_to_inertia[i].M.Inverse() * inertia_pa;
        // Build complete rigid body inertia (in predecessor's joint frame)
        KDL::RigidBodyInertia inertia_in_predecessor_joint_frame(mass[i], joint_to_inertia[i].p, inertia_jf.getRotationalInertia());
        // Transform to the current segment's reference frame
        KDL::RigidBodyInertia inertia2 = joint_to_tip[i].Inverse() * inertia_in_predecessor_joint_frame;


        /*
        // KDL expects the rotational inertia to be expressed in the inertia
        // frame. Internally (in the rigid body inertia) the inertia is then
        // shifted to the base frame using the CoM vector.
        //
        // Note: This operation does not work as expected. The rotational
        // inertia is expressed in the orientation frame of the principal axes.
        // However, KDL expects it to be expressed in the orientation frame of
        // the reference frame.
        KDL::RigidBodyInertia inertia_pa(mass[i], joint_to_inertia[i].p, central_principal_moments_of_inertia[i]);

        // The operator* has the following effect:
        // * First, the _rotational inertia_ of the rigid body inertia is
        //   re-expressed with the orientation of the frame.
        // * Second, the inertia is shifted to the origin of the frame (i.e.
        //   it's reference point is changed) using the parallel axis
        //   theorem.
        KDL::RigidBodyInertia inertia = joint_to_tip[i].Inverse() * inertia_pa;
        */


        KDL::Joint::JointType type;
        if ((i == 0) || (i == NUMBER_OF_BODIES - 1)) type = KDL::Joint::None;
        else type = KDL::Joint::RotZ;

        chain.addSegment(KDL::Segment(KDL::Joint(type), joint_to_tip[i], inertia));
    }
}

