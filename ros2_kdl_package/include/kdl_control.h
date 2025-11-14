#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);

    Eigen::VectorXd velocity_ctrl_null(const Eigen::Vector3d& pos_err,
                                     const Eigen::Vector3d& ori_err,
                                     const Eigen::Vector3d& des_lin_vel,
                                     double Kp,
                                     const KDL::JntArray& q_min,
                                     const KDL::JntArray& q_max,
                                     double lambda);

    Eigen::VectorXd vision_ctrl(const Eigen::Vector3d& p_cO,   // posizione marker nel frame camera
                            const KDL::Frame&     F_ec,    // pose camera wrt EE (fissa)
                            const Eigen::Vector3d& s_des,  // tipico [0,0,1]
                            double Kv,                     // guadagno (diag = Kv*I)
                            const KDL::JntArray& q_min,
                            const KDL::JntArray& q_max,
                            double lambda);



private:

    KDLRobot* robot_;

};

#endif
