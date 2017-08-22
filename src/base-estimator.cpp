/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <sot/torque_control/base-estimator.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>

#include "pinocchio/algorithm/frames.hpp"

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace se3;
      using boost::math::normal; // typedef provides default type is double.

      void rpyToMatrix(double roll, double pitch, double yaw, Eigen::Matrix3d & R)
      {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
        R = q.matrix();
      }

      void rpyToMatrix(const Eigen::Vector3d & rpy, Eigen::Matrix3d & R)
      {
        return rpyToMatrix(rpy[0], rpy[1], rpy[2], R);
      }

      void matrixToRpy(const Eigen::Matrix3d & M, Eigen::Vector3d & rpy)
      {
        double m = sqrt(M(2,1)*M(2,1) + M(2,2)*M(2,2));
        rpy(1) = atan2(-M(2,0), m);
        if( fabs(fabs(rpy(1)) - 0.5*M_PI) < 0.001 )
        {
          rpy(0) = 0.0;
          rpy(2) = -atan2(M(0,1), M(1,1));
        }
        else
        {
          rpy(2) = atan2(M(1,0), M(0,0));  // alpha
          rpy(0) = atan2(M(2,1), M(2,2));  // gamma
        }
      }

#define PROFILE_BASE_POSITION_ESTIMATION    "base-est position estimation"
#define PROFILE_BASE_VELOCITY_ESTIMATION    "base-est velocity estimation"
#define PROFILE_BASE_KINEMATICS_COMPUTATION "base-est kinematics computation"

#define IMU_JOINT_NAME "CHEST_JOINT1"

#define INPUT_SIGNALS     m_joint_positionsSIN << m_joint_velocitiesSIN << \
                          m_imu_quaternionSIN << m_forceLLEGSIN << m_forceRLEGSIN << \
                          m_w_lf_inSIN << m_w_rf_inSIN
#define OUTPUT_SIGNALS    m_qSOUT << m_vSOUT << m_q_lfSOUT << m_q_rfSOUT << m_q_imuSOUT << \
                          m_w_lfSOUT << m_w_rfSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef BaseEstimator EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(BaseEstimator,
                                         "BaseEstimator");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      BaseEstimator::
      BaseEstimator(const std::string& name)
        : Entity(name)
        ,CONSTRUCT_SIGNAL_IN( joint_positions,            dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( joint_velocities,           dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( imu_quaternion,             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( forceLLEG,                  dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( forceRLEG,                  dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( w_lf_in,                    double)
        ,CONSTRUCT_SIGNAL_IN( w_rf_in,                    double)
        ,CONSTRUCT_SIGNAL_INNER(kinematics_computations,  dynamicgraph::Vector, m_joint_positionsSIN
                                                                              <<m_joint_velocitiesSIN)
        ,CONSTRUCT_SIGNAL_OUT(q,                          dynamicgraph::Vector, m_kinematics_computationsSINNER
                                                                              <<m_joint_positionsSIN
                                                                              <<m_imu_quaternionSIN
                                                                              <<m_forceLLEGSIN
                                                                              <<m_forceRLEGSIN
                                                                              <<m_w_lf_inSIN
                                                                              <<m_w_rf_inSIN
                                                                              <<m_w_lfSOUT
                                                                              <<m_w_rfSOUT)
        ,CONSTRUCT_SIGNAL_OUT(v,                          dynamicgraph::Vector, m_kinematics_computationsSINNER)
        ,CONSTRUCT_SIGNAL_OUT(q_lf,                       dynamicgraph::Vector, m_qSOUT)
        ,CONSTRUCT_SIGNAL_OUT(q_rf,                       dynamicgraph::Vector, m_qSOUT)
        ,CONSTRUCT_SIGNAL_OUT(q_imu,                      dynamicgraph::Vector, m_qSOUT<<m_imu_quaternionSIN)
        ,CONSTRUCT_SIGNAL_OUT(w_lf,                       double, m_forceLLEGSIN)
        ,CONSTRUCT_SIGNAL_OUT(w_rf,                       double, m_forceRLEGSIN)
        ,m_initSucceeded(false)
        ,m_reset_foot_pos(true)
        ,m_w_imu(0.0)
        ,m_zmp_std_dev_lf(1.0)
        ,m_zmp_std_dev_rf(1.0)
        ,m_fz_std_dev_lf(1.0)
        ,m_fz_std_dev_rf(1.0)
        ,m_zmp_margin_lf(0.0)
        ,m_zmp_margin_rf(0.0)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        m_K_rf << 4034,23770,239018,707,502,936;
        m_K_lf << 4034,23770,239018,707,502,936;
        m_left_foot_sizes  << 0.130, -0.100,  0.075, -0.056;
        m_right_foot_sizes << 0.130, -0.100,  0.056, -0.075;

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &BaseEstimator::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "time step (double)",
                                                    "URDF file path (string)")));
        addCommand("reset_foot_positions",
                   makeCommandVoid0(*this, &BaseEstimator::reset_foot_positions,
                                    docCommandVoid0("Reset the position of the feet.")));
        addCommand("get_imu_weight",
                   makeDirectGetter(*this,&m_w_imu,
                                    docDirectGetter("Weight of imu-based orientation in sensor fusion",
                                                    "double")));
        addCommand("set_imu_weight",
                   makeCommandVoid1(*this, &BaseEstimator::set_imu_weight,
                                    docCommandVoid1("Set the weight of imu-based orientation in sensor fusion",
                                                    "double")));
        addCommand("set_zmp_std_dev_right_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_zmp_std_dev_right_foot,
                                    docCommandVoid1("Set the standard deviation of the ZMP measurement errors for the right foot",
                                                    "double")));
        addCommand("set_zmp_std_dev_left_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_zmp_std_dev_left_foot,
                                    docCommandVoid1("Set the standard deviation of the ZMP measurement errors for the left foot",
                                                    "double")));
        addCommand("set_normal_force_std_dev_right_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_normal_force_std_dev_right_foot,
                                    docCommandVoid1("Set the standard deviation of the normal force measurement errors for the right foot",
                                                    "double")));
        addCommand("set_normal_force_std_dev_left_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_normal_force_std_dev_left_foot,
                                    docCommandVoid1("Set the standard deviation of the normal force measurement errors for the left foot",
                                                    "double")));
        addCommand("set_stiffness_right_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_stiffness_right_foot,
                                    docCommandVoid1("Set the 6d stiffness of the spring at the right foot",
                                                    "vector")));
        addCommand("set_stiffness_left_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_stiffness_left_foot,
                                    docCommandVoid1("Set the 6d stiffness of the spring at the left foot",
                                                    "vector")));
        addCommand("set_right_foot_sizes",
                   makeCommandVoid1(*this, &BaseEstimator::set_right_foot_sizes,
                                    docCommandVoid1("Set the size of the right foot (pos x, neg x, pos y, neg y)",
                                                    "4d vector")));
        addCommand("set_left_foot_sizes",
                   makeCommandVoid1(*this, &BaseEstimator::set_left_foot_sizes,
                                    docCommandVoid1("Set the size of the left foot (pos x, neg x, pos y, neg y)",
                                                    "4d vector")));
        addCommand("set_zmp_margin_right_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_zmp_margin_right_foot,
                                    docCommandVoid1("Set the ZMP margin for the right foot",
                                                    "double")));
        addCommand("set_zmp_margin_left_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_zmp_margin_left_foot,
                                    docCommandVoid1("Set the ZMP margin for the left foot",
                                                    "double")));
        addCommand("set_normal_force_margin_right_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_normal_force_margin_right_foot,
                                    docCommandVoid1("Set the normal force margin for the right foot",
                                                    "double")));
        addCommand("set_normal_force_margin_left_foot",
                   makeCommandVoid1(*this, &BaseEstimator::set_normal_force_margin_left_foot,
                                    docCommandVoid1("Set the normal force margin for the left foot",
                                                    "double")));
      }

      void BaseEstimator::init(const double & dt, const std::string& urdfFile)
      {
        m_dt = dt;
        try
        {
          se3::urdf::buildModel(urdfFile,se3::JointModelFreeFlyer(),m_model);
          assert(m_model.nq == N_JOINTS+7);
          assert(m_model.nv == N_JOINTS+6);
          assert(m_model.existFrame(LEFT_FOOT_FRAME_NAME));
          assert(m_model.existFrame(RIGHT_FOOT_FRAME_NAME));
          m_left_foot_id = m_model.getFrameId(LEFT_FOOT_FRAME_NAME);
          m_right_foot_id = m_model.getFrameId(RIGHT_FOOT_FRAME_NAME);
          m_q_pin.setZero(m_model.nq);
          m_q_pin[6]= 1.; // for quaternion
          m_q_sot.setZero(N_JOINTS+6);
          m_v_pin.setZero(N_JOINTS+6);
          m_v_sot.setZero(N_JOINTS+6);

          m_sole_M_ftSens = SE3(Matrix3::Identity(),
                                -Eigen::Map<const Vector3>(RIGHT_FOOT_FORCE_SENSOR_XYZ));
        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
        }
        m_data = new se3::Data(m_model);
        m_initSucceeded = true;
      }

      void BaseEstimator::reset_foot_positions()
      {
        m_reset_foot_pos = true;
      }

      void BaseEstimator::set_imu_weight(const double& w)
      {
        if(w<0.0)
          return SEND_MSG("Imu weight must be nonnegative", MSG_TYPE_ERROR);
        m_w_imu = w;
      }

      void BaseEstimator::set_stiffness_right_foot(const dynamicgraph::Vector & k)
      {
        if(k.size()!=6)
          return SEND_MSG("Stiffness vector should have size 6, not "+toString(k.size()), MSG_TYPE_ERROR);
        m_K_rf = k;
      }

      void BaseEstimator::set_stiffness_left_foot(const dynamicgraph::Vector & k)
      {
        if(k.size()!=6)
          return SEND_MSG("Stiffness vector should have size 6, not "+toString(k.size()), MSG_TYPE_ERROR);
        m_K_lf = k;
      }

      void BaseEstimator::set_zmp_std_dev_right_foot(const double & std_dev)
      {
        if(std_dev<=0.0)
          return SEND_MSG("Standard deviation must be a positive number", MSG_TYPE_ERROR);
        m_zmp_std_dev_rf = std_dev;
      }

      void BaseEstimator::set_zmp_std_dev_left_foot(const double & std_dev)
      {
        if(std_dev<=0.0)
          return SEND_MSG("Standard deviation must be a positive number", MSG_TYPE_ERROR);
        m_zmp_std_dev_lf = std_dev;
      }

      void BaseEstimator::set_normal_force_std_dev_right_foot(const double & std_dev)
      {
        if(std_dev<=0.0)
          return SEND_MSG("Standard deviation must be a positive number", MSG_TYPE_ERROR);
        m_fz_std_dev_rf = std_dev;
      }

      void BaseEstimator::set_normal_force_std_dev_left_foot(const double & std_dev)
      {
        if(std_dev<=0.0)
          return SEND_MSG("Standard deviation must be a positive number", MSG_TYPE_ERROR);
        m_fz_std_dev_lf = std_dev;
      }

      void BaseEstimator::set_right_foot_sizes(const dynamicgraph::Vector & s)
      {
        if(s.size()!=4)
          return SEND_MSG("Foot size vector should have size 4, not "+toString(s.size()), MSG_TYPE_ERROR);
        m_right_foot_sizes = s;
      }

      void BaseEstimator::set_left_foot_sizes(const dynamicgraph::Vector & s)
      {
        if(s.size()!=4)
          return SEND_MSG("Foot size vector should have size 4, not "+toString(s.size()), MSG_TYPE_ERROR);
        m_left_foot_sizes = s;
      }

      void BaseEstimator::set_zmp_margin_right_foot(const double & margin)
      {
        m_zmp_margin_rf = margin;
      }

      void BaseEstimator::set_zmp_margin_left_foot(const double & margin)
      {
        m_zmp_margin_lf = margin;
      }

      void BaseEstimator::set_normal_force_margin_right_foot(const double & margin)
      {
        m_fz_margin_rf = margin;
      }

      void BaseEstimator::set_normal_force_margin_left_foot(const double & margin)
      {
        m_fz_margin_lf = margin;
      }

      void BaseEstimator::compute_zmp(const Vector6 & w, Vector2 & zmp)
      {
        se3::Force f(w);
        f = m_sole_M_ftSens.act(f);
        if(f.linear()[2]==0.0)
          return;
        zmp[0] = -f.angular()[1] / f.linear()[2];
        zmp[1] =  f.angular()[0] / f.linear()[2];
      }

      double BaseEstimator::compute_zmp_weight(const Vector2 & zmp, const Vector4 & foot_sizes,
                                               double std_dev, double margin)
      {
        double fs0=foot_sizes[0] - margin;
        double fs1=foot_sizes[1] + margin;
        double fs2=foot_sizes[2] - margin;
        double fs3=foot_sizes[3] + margin;

        if(zmp[0]>fs0 || zmp[0]<fs1 || zmp[1]>fs2 || zmp[1]<fs3)
          return 0;

        double cdx = ((cdf(m_normal, (fs0-zmp[0])/std_dev) -
                       cdf(m_normal, (fs1-zmp[0])/std_dev))-0.5 )*2.0;
        double cdy = ((cdf(m_normal, (fs2-zmp[1])/std_dev) -
                       cdf(m_normal, (fs3-zmp[1])/std_dev))-0.5 )*2.0;
        return cdx*cdy;
      }

      double BaseEstimator::compute_force_weight(double fz, double std_dev, double margin)
      {
        if (fz<margin)
          return 0.0;
        return (cdf(m_normal, (fz-margin)/std_dev)-0.5)*2.0;
      }

      void BaseEstimator::reset_foot_positions_impl(const Vector6 & ftlf, const Vector6 & ftrf)
      {
        // compute the base position wrt each foot
        SE3 dummy, lfMff, rfMff;
        m_oMrfs = SE3::Identity();
        m_oMlfs = SE3::Identity();
        kinematics_estimation(ftrf, m_K_rf, m_oMrfs, m_right_foot_id, rfMff, dummy);
        kinematics_estimation(ftlf, m_K_lf, m_oMlfs, m_left_foot_id,  lfMff, dummy);

        // distance from ankle to ground
        Eigen::Map<const Eigen::Vector3d> ankle_2_sole_xyz(&RIGHT_FOOT_SOLE_XYZ[0]);
        const SE3 groundMfoot(Matrix3::Identity(), -1.0*ankle_2_sole_xyz);
        lfMff = groundMfoot * lfMff;
        rfMff = groundMfoot * rfMff;

        // set the world frame in between the feet
        const Vector3 w( 0.5*(se3::log3(lfMff.rotation())+se3::log3(rfMff.rotation())) );
        const SE3 oMff = SE3(se3::exp3(w), 0.5 * (lfMff.translation()+rfMff.translation()) );

        m_oMlfs = oMff * lfMff.inverse() * groundMfoot;
        m_oMrfs = oMff * rfMff.inverse() * groundMfoot;

        sendMsg("Reference pos of left foot:\n"+toString(m_oMlfs), MSG_TYPE_INFO);
        sendMsg("Reference pos of right foot:\n"+toString(m_oMrfs), MSG_TYPE_INFO);

//        kinematics_estimation(ftrf, m_K_rf, m_oMrfs, m_right_foot_id, m_oMff_rf, dummy);
//        kinematics_estimation(ftlf, m_K_lf, m_oMlfs, m_left_foot_id,  m_oMff_lf, dummy);
//        sendMsg("Base estimation left foot:\n"+toString(m_oMff_lf), MSG_TYPE_DEBUG);
//        sendMsg("Base estimation right foot:\n"+toString(m_oMff_rf), MSG_TYPE_DEBUG);
//        sendMsg("Difference base estimation left-right foot:\n"+toString(m_oMff_rf.inverse()*m_oMff_lf), MSG_TYPE_DEBUG);

        m_reset_foot_pos = false;
      }

      void BaseEstimator::kinematics_estimation(const Vector6 & ft, const Vector6 & K,
                                                const SE3 & oMfs, const int foot_id,
                                                SE3 & oMff, SE3 & oMfa)
      {
        Vector3 xyz;
        xyz << -ft[0]/K(0), -ft[1]/K(1), -ft[2]/K(2);
        Matrix3 R;
        rpyToMatrix(-ft[3]/K(3), -ft[4]/K(4), -ft[5]/K(5), R);
        const SE3 fsMfa(R, xyz);                          // foot sole to foot ankle
        oMfa = oMfs*fsMfa;                                // world to foot ankle
        const SE3 faMff(m_data->oMf[foot_id].inverse());  // foot ankle to free flyer
        oMff = oMfa*faMff;                                // world to free flyer
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_INNER_FUNCTION(kinematics_computations, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal kinematics_computations before initialization!");
          return s;
        }

        const Eigen::VectorXd& qj= m_joint_positionsSIN(iter);     //n+6
        const Eigen::VectorXd& dq= m_joint_velocitiesSIN(iter);
        assert(qj.size()==N_JOINTS     && "Unexpected size of signal joint_positions");
        assert(dq.size()==N_JOINTS     && "Unexpected size of signal joint_velocities");

        /* convert sot to pinocchio joint order */
        joints_sot_to_urdf(qj, m_q_pin.tail<N_JOINTS>());
        joints_sot_to_urdf(dq, m_v_pin.tail<N_JOINTS>());

        getProfiler().start(PROFILE_BASE_KINEMATICS_COMPUTATION);

        /* Compute kinematics assuming world is at free-flyer frame */
        m_q_pin.head<6>().setZero();
        m_q_pin(6) = 1.0;
        se3::forwardKinematics(m_model, *m_data, m_q_pin, m_v_pin);
        se3::framesForwardKinematics(m_model, *m_data);

        getProfiler().stop(PROFILE_BASE_KINEMATICS_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(q, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal q before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);
        
        const Eigen::VectorXd & qj          = m_joint_positionsSIN(iter);     //n+6
        const Eigen::Vector4d & quatIMU_vec = m_imu_quaternionSIN(iter);
        const Vector6 & ftrf                = m_forceRLEGSIN(iter);
        const Vector6 & ftlf                = m_forceLLEGSIN(iter);

        // if the weights are not specified by the user through the input signals w_lf, w_rf
        // then compute them
        double wL, wR;
        if(m_w_lf_inSIN.isPlugged())
          wL = m_w_lf_inSIN(iter);
        else
          wL = m_w_lfSOUT(iter);

        if(m_w_rf_inSIN.isPlugged())
          wR = m_w_rf_inSIN(iter);
        else
          wR = m_w_rfSOUT(iter);

        assert(qj.size()==N_JOINTS     && "Unexpected size of signal joint_positions");

        m_kinematics_computationsSINNER(iter);

        if(m_reset_foot_pos)
          reset_foot_positions_impl(ftlf, ftrf);

        getProfiler().start(PROFILE_BASE_POSITION_ESTIMATION);
        {
          SE3 oMlfa, oMrfa;
          kinematics_estimation(ftrf, m_K_rf, m_oMrfs, m_right_foot_id, m_oMff_rf, oMrfa);
          kinematics_estimation(ftlf, m_K_lf, m_oMlfs, m_left_foot_id,  m_oMff_lf, oMlfa);

          // get rpy
          Vector3 rpy_ff, rpy_ff_lf, rpy_ff_rf, rpy_ff_imu;
          matrixToRpy(m_oMff_lf.rotation(), rpy_ff_lf);
          matrixToRpy(m_oMff_rf.rotation(), rpy_ff_rf);
          Eigen::Quaternion<double> quatIMU(quatIMU_vec[0], quatIMU_vec[1], quatIMU_vec[2], quatIMU_vec[3]);
          matrixToRpy(quatIMU.toRotationMatrix(), rpy_ff_imu); // THIS IS FALSE ! IMU IS NOT LOCATED ON FREEFLYER

          // average (we do not take into account the IMU yaw)
          double wSum = wL + wR + m_w_imu;
          rpy_ff(0) = (rpy_ff_lf[0]*wL + rpy_ff_rf[0]*wR + rpy_ff_imu[0]*m_w_imu) / wSum;
          rpy_ff(1) = (rpy_ff_lf[1]*wL + rpy_ff_rf[1]*wR + rpy_ff_imu[1]*m_w_imu) / wSum;
          rpy_ff(2) = (rpy_ff_lf[2]*wL + rpy_ff_rf[2]*wR )                 / (wL+wR);
          Matrix3 R;
          rpyToMatrix(rpy_ff, R);

          // translation to get foot at the right position
          // evaluate Mrl Mlf for q with the good orientation and zero translation for freeflyer
          const Vector3 pos_lf_ff = R * m_data->oMf[m_left_foot_id].translation();
          const Vector3 pos_rf_ff = R * m_data->oMf[m_right_foot_id].translation();
          // get average translation
          m_q_pin.head<3>() = ((oMlfa.translation() - pos_lf_ff)*wL +
                               (oMrfa.translation() - pos_rf_ff)*wR) / (wL+wR);

          m_q_sot.tail<N_JOINTS>() = qj;
          base_se3_to_sot(m_q_pin.head<3>(), R, m_q_sot.head<6>());

          s = m_q_sot;
        }
        getProfiler().stop(PROFILE_BASE_POSITION_ESTIMATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(q_lf, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal q_lf before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);

        const Eigen::VectorXd & q = m_qSOUT(iter);
        s.tail(N_JOINTS) = q.tail(N_JOINTS);
        base_se3_to_sot(m_oMff_lf.translation(), m_oMff_lf.rotation(), s.head<6>());

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(q_rf, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal q_rf before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);

        const Eigen::VectorXd & q = m_qSOUT(iter);
        s.tail(N_JOINTS) = q.tail(N_JOINTS);
        base_se3_to_sot(m_oMff_rf.translation(), m_oMff_rf.rotation(), s.head<6>());

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(q_imu, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal q_imu before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);

        const Eigen::VectorXd & q = m_qSOUT(iter);
        s.tail(N_JOINTS) = q.tail(N_JOINTS);

        const Eigen::Vector4d & quatIMU_vec = m_imu_quaternionSIN(iter);
        Eigen::Quaternion<double> quatIMU(quatIMU_vec);
        base_se3_to_sot(q.head<3>(), quatIMU.toRotationMatrix(), s.head<6>());

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(w_lf, double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal w_lf before initialization!");
          return s;
        }

        const Vector6 & wrench                = m_forceLLEGSIN(iter);
        Vector2 zmp;
        compute_zmp(wrench, zmp);
        double w_zmp = compute_zmp_weight(zmp, m_left_foot_sizes,
                                          m_zmp_std_dev_lf, m_zmp_margin_lf);
        double w_fz = compute_force_weight(wrench(2), m_fz_std_dev_lf, m_fz_margin_lf);
        s = w_zmp*w_fz;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(w_rf, double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal w_rf before initialization!");
          return s;
        }

        const Vector6 & wrench                = m_forceRLEGSIN(iter);
        Vector2 zmp;
        compute_zmp(wrench, zmp);
        double w_zmp = compute_zmp_weight(zmp, m_right_foot_sizes,
                                          m_zmp_std_dev_rf, m_zmp_margin_rf);
        double w_fz = compute_force_weight(wrench(2), m_fz_std_dev_rf, m_fz_margin_rf);
        s = w_zmp*w_fz;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(v,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);

        m_kinematics_computationsSINNER(iter);

        getProfiler().start(PROFILE_BASE_VELOCITY_ESTIMATION);
        {
          const Eigen::VectorXd& dq= m_joint_velocitiesSIN(iter);
          assert(dq.size()==N_JOINTS     && "Unexpected size of signal joint_velocities");

          /* Compute foot velocities */
          const Frame & f_lf = m_model.frames[m_left_foot_id];
          const Motion v_lf_local = f_lf.placement.actInv(m_data->v[f_lf.parent]);
          const Vector6 v_lf = m_data->oMf[m_left_foot_id].act(v_lf_local).toVector();

          const Frame & f_rf = m_model.frames[m_right_foot_id];
          const Motion v_rf_local = f_rf.placement.actInv(m_data->v[f_rf.parent]);
          const Vector6 v_rf = m_data->oMf[m_right_foot_id].act(v_rf_local).toVector();

          m_v_sot.head<6>() = - 0.5*(v_lf + v_rf);
          m_v_sot.tail<N_JOINTS>() = dq;

          s = m_v_sot;
        }
        getProfiler().stop(PROFILE_BASE_VELOCITY_ESTIMATION);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void BaseEstimator::display(std::ostream& os) const
      {
        os << "BaseEstimator "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

      void BaseEstimator::commandLine(const std::string& cmdLine,
                                      std::istringstream& cmdArgs,
                                      std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "BaseEstimator:\n"
             << "\t -." << std::endl;
          Entity::commandLine(cmdLine, cmdArgs, os);
        }
        else
        {
          Entity::commandLine(cmdLine,cmdArgs,os);
        }
      }
      
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

