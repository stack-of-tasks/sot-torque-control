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

      void se3Interp(const se3::SE3 & s1, const se3::SE3 & s2, const double alpha, se3::SE3 & s12)
      {
        const Eigen::Vector3d t_( s1.translation() * alpha+
                                  s2.translation() * (1-alpha));

        const Eigen::Vector3d w( se3::log3(s1.rotation()) * alpha +
                                 se3::log3(s2.rotation()) * (1-alpha) );

        s12 =  se3::SE3(se3::exp3(w),t_);
      }
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
      
      void quanternionMult(const Eigen::Vector4d & q1, const Eigen::Vector4d & q2,  Eigen::Vector4d & q12)
      {
        q12(0) = q2(0)*q1(0)-q2(1)*q1(1)-q2(2)*q1(2)-q2(3)*q1(3);
        q12(1) = q2(0)*q1(1)+q2(1)*q1(0)-q2(2)*q1(3)+q2(3)*q1(2);
        q12(2) = q2(0)*q1(2)+q2(1)*q1(3)+q2(2)*q1(0)-q2(3)*q1(1);
        q12(3) = q2(0)*q1(3)-q2(1)*q1(2)+q2(2)*q1(1)+q2(3)*q1(0);
      }

      void pointRotationByQuaternion(const Eigen::Vector3d & point,const Eigen::Vector4d & quat, Eigen::Vector3d & rotatedPoint)
      {
            const Eigen::Vector4d p4(0.0, point(0),point(1),point(2));
            const Eigen::Vector4d quat_conj(quat(0),-quat(1),-quat(2),-quat(3));
            Eigen::Vector4d q_tmp1,q_tmp2;
            quanternionMult(quat,p4,q_tmp1);
            quanternionMult(q_tmp1,quat_conj,q_tmp2);
            rotatedPoint(0) = q_tmp2(1);
            rotatedPoint(1) = q_tmp2(2);
            rotatedPoint(2) = q_tmp2(3);
      }

#define PROFILE_BASE_POSITION_ESTIMATION    "base-est position estimation"
#define PROFILE_BASE_VELOCITY_ESTIMATION    "base-est velocity estimation"
#define PROFILE_BASE_KINEMATICS_COMPUTATION "base-est kinematics computation"


#define INPUT_SIGNALS     m_joint_positionsSIN << m_joint_velocitiesSIN << \
                          m_imu_quaternionSIN << m_forceLLEGSIN << m_forceRLEGSIN <<  m_dforceLLEGSIN << m_dforceRLEGSIN << \
                          m_w_lf_inSIN << m_w_rf_inSIN << m_K_fb_feet_posesSIN << m_lf_ref_xyzquatSIN << m_rf_ref_xyzquatSIN << m_accelerometerSIN << m_gyroscopeSIN
#define OUTPUT_SIGNALS    m_qSOUT << m_vSOUT << m_q_lfSOUT << m_q_rfSOUT << m_q_imuSOUT << \
                          m_w_lfSOUT << m_w_rfSOUT << m_w_lf_filteredSOUT << m_w_rf_filteredSOUT << m_lf_xyzquatSOUT << m_rf_xyzquatSOUT << \ 
                          m_v_acSOUT << m_a_acSOUT << m_v_kinSOUT << m_v_imuSOUT << m_v_gyrSOUT << m_v_flexSOUT

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
        ,CONSTRUCT_SIGNAL_IN( dforceLLEG,                 dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( dforceRLEG,                 dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( w_lf_in,                    double)
        ,CONSTRUCT_SIGNAL_IN( w_rf_in,                    double)
        ,CONSTRUCT_SIGNAL_IN( K_fb_feet_poses,            double)
        ,CONSTRUCT_SIGNAL_IN( lf_ref_xyzquat,             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( rf_ref_xyzquat,             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( accelerometer,              dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN( gyroscope,                  dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_INNER(kinematics_computations,  dynamicgraph::Vector, m_joint_positionsSIN
                                                                              <<m_joint_velocitiesSIN)
        ,CONSTRUCT_SIGNAL_OUT(q,                          dynamicgraph::Vector, m_kinematics_computationsSINNER
                                                                              <<m_joint_positionsSIN
                                                                              <<m_imu_quaternionSIN
                                                                              <<m_forceLLEGSIN
                                                                              <<m_forceRLEGSIN
                                                                              <<m_w_lf_inSIN
                                                                              <<m_w_rf_inSIN
                                                                              <<m_K_fb_feet_posesSIN
                                                                              <<m_w_lf_filteredSOUT
                                                                              <<m_w_rf_filteredSOUT
                                                                              <<m_lf_ref_xyzquatSIN
                                                                              <<m_rf_ref_xyzquatSIN)
        ,CONSTRUCT_SIGNAL_OUT(v,                          dynamicgraph::Vector, m_kinematics_computationsSINNER << m_accelerometerSIN << m_gyroscopeSIN << m_qSOUT << m_dforceLLEGSIN << m_dforceRLEGSIN)
        ,CONSTRUCT_SIGNAL_OUT(v_ac,                       dynamicgraph::Vector, m_vSOUT)
        ,CONSTRUCT_SIGNAL_OUT(a_ac,                       dynamicgraph::Vector, m_vSOUT)
        ,CONSTRUCT_SIGNAL_OUT(v_flex,                     dynamicgraph::Vector, m_vSOUT)
        ,CONSTRUCT_SIGNAL_OUT(v_imu,                      dynamicgraph::Vector, m_vSOUT)
        ,CONSTRUCT_SIGNAL_OUT(v_gyr,                      dynamicgraph::Vector, m_vSOUT)
        ,CONSTRUCT_SIGNAL_OUT(v_kin,                      dynamicgraph::Vector, m_vSOUT)
        ,CONSTRUCT_SIGNAL_OUT(lf_xyzquat,                 dynamicgraph::Vector, m_qSOUT)
        ,CONSTRUCT_SIGNAL_OUT(rf_xyzquat,                 dynamicgraph::Vector, m_qSOUT)
        ,CONSTRUCT_SIGNAL_OUT(q_lf,                       dynamicgraph::Vector, m_qSOUT)
        ,CONSTRUCT_SIGNAL_OUT(q_rf,                       dynamicgraph::Vector, m_qSOUT)
        ,CONSTRUCT_SIGNAL_OUT(q_imu,                      dynamicgraph::Vector, m_qSOUT<<m_imu_quaternionSIN)
        ,CONSTRUCT_SIGNAL_OUT(w_lf,                       double, m_forceLLEGSIN)
        ,CONSTRUCT_SIGNAL_OUT(w_rf,                       double, m_forceRLEGSIN)
        ,CONSTRUCT_SIGNAL_OUT(w_lf_filtered,              double, m_w_lfSOUT)
        ,CONSTRUCT_SIGNAL_OUT(w_rf_filtered,              double, m_w_rfSOUT)
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


        addCommand("set_fz_stable_windows_size",
                           makeCommandVoid1(*this, &BaseEstimator::set_fz_stable_windows_size,
                                            docCommandVoid1("Set the windows size used to detect that feet is stable",
                                                            "int")));
        addCommand("set_alpha_w_filter",
                   makeCommandVoid1(*this, &BaseEstimator::set_alpha_w_filter,
                                    docCommandVoid1("Set the filter parameter to filter weights",
                                                    "double")));                                            
        addCommand("set_alpha_DC_acc",
                   makeCommandVoid1(*this, &BaseEstimator::set_alpha_DC_acc,
                                    docCommandVoid1("Set the filter parameter for removing DC from accelerometer data",
                                                    "double")));                                            
        addCommand("set_alpha_DC_vel",
                   makeCommandVoid1(*this, &BaseEstimator::set_alpha_DC_vel,
                                    docCommandVoid1("Set the filter parameter for removing DC from velocity integrated from acceleration",
                                                    "double")));                                            
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

      void BaseEstimator::init(const double & dt, const std::string& robotRef)
      {
        m_dt = dt;
        try
        {
          /* Retrieve m_robot_util informations */
          std::string localName(robotRef);
          if (isNameInRobotUtil(localName))
          {
            m_robot_util = getRobotUtil(localName);
            std::cerr << "m_robot_util:" << m_robot_util << std::endl;
          }
          else
          {
            SEND_MSG("You should have an entity controller manager initialized before",MSG_TYPE_ERROR);
            return;
          }

          se3::urdf::buildModel(m_robot_util->m_urdf_filename,
                                se3::JointModelFreeFlyer(), m_model);

          assert(m_model.existFrame(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name));
          assert(m_model.existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
          assert(m_model.existFrame(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name));
          m_left_foot_id  = m_model.getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);
          m_right_foot_id = m_model.getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
          m_IMU_body_id   = m_model.getFrameId(m_robot_util->m_imu_joint_name);

          m_q_pin.setZero(m_model.nq);
          m_q_pin[6]= 1.; // for quaternion
          m_q_sot.setZero(m_robot_util->m_nbJoints+6);
          m_v_pin.setZero(m_robot_util->m_nbJoints+6);
          m_v_sot.setZero(m_robot_util->m_nbJoints+6);
          m_v_kin.setZero(m_robot_util->m_nbJoints+6);
          m_v_flex.setZero(m_robot_util->m_nbJoints+6);
          m_v_imu.setZero(m_robot_util->m_nbJoints+6);
          m_v_gyr.setZero(m_robot_util->m_nbJoints+6);
          m_sole_M_ftSens = SE3(Matrix3::Identity(),
                                -Eigen::Map<const Vector3>(&m_robot_util->m_foot_util.m_Right_Foot_Force_Sensor_XYZ(0)));
        
          m_last_vel.setZero();
          m_last_DCvel.setZero();
          m_last_DCacc.setZero(); //this is to replace by acceleration at 1st iteration
          
          m_alpha_DC_acc = 0.9995;
          m_alpha_DC_vel = 0.9995;
          m_alpha_w_filter = 1.0;
          m_left_foot_is_stable  = true;
          m_right_foot_is_stable = true;
          m_fz_stable_windows_size = 10;
          m_lf_fz_stable_cpt = m_fz_stable_windows_size;
          m_rf_fz_stable_cpt = m_fz_stable_windows_size;
          m_isFirstIterFlag = true;
        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
          return;
        }
        m_data = new se3::Data(m_model);
        m_initSucceeded = true;
      }

      void BaseEstimator::set_fz_stable_windows_size(const int& ws)
      {
          if(ws<0.0)
          return SEND_MSG("windows_size should be a positive integer", MSG_TYPE_ERROR);
        m_fz_stable_windows_size = ws;
      }

      void BaseEstimator::set_alpha_w_filter(const double& a)
      {
          if(a<0.0 || a>1.0)
          return SEND_MSG("alpha should be in [0..1]", MSG_TYPE_ERROR);
        m_alpha_w_filter = a;
      }


      void BaseEstimator::set_alpha_DC_acc(const double& a)
      {
          if(a<0.0 || a>1.0)
          return SEND_MSG("alpha should be in [0..1]", MSG_TYPE_ERROR);
        m_alpha_DC_acc = a;
      }
      
      void BaseEstimator::set_alpha_DC_vel(const double& a)
      {
          if(a<0.0 || a>1.0)
          return SEND_MSG("alpha should be in [0..1]", MSG_TYPE_ERROR);
        m_alpha_DC_vel = a;
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
        SE3 dummy, dummy1, lfMff, rfMff;
        m_oMrfs = SE3::Identity();
        m_oMlfs = SE3::Identity();
        kinematics_estimation(ftrf, m_K_rf, m_oMrfs, m_right_foot_id, rfMff, dummy, dummy1); //rfMff is obtain reading oMff becaused oMrfs is here set to Identity
        kinematics_estimation(ftlf, m_K_lf, m_oMlfs, m_left_foot_id,  lfMff, dummy, dummy1);

        // distance from ankle to ground
        const Vector3 & ankle_2_sole_xyz = m_robot_util->m_foot_util.m_Right_Foot_Sole_XYZ;
        const SE3 groundMfoot(Matrix3::Identity(), -1.0*ankle_2_sole_xyz);
        lfMff = groundMfoot * lfMff;
        rfMff = groundMfoot * rfMff;

        // set the world frame in between the feet
        const Vector3 w( 0.5*(se3::log3(lfMff.rotation())+se3::log3(rfMff.rotation())) );
        SE3 oMff = SE3(se3::exp3(w), 0.5*(lfMff.translation()+rfMff.translation()));
        // add a constant offset to make the world frame match the one in OpenHRP
        oMff.translation()(0) += 9.562e-03;

        m_oMlfs = oMff * lfMff.inverse() * groundMfoot;
        m_oMrfs = oMff * rfMff.inverse() * groundMfoot;

        m_oMlfs_xyzquat.head<3>() = m_oMlfs.translation();
        Eigen::Quaternion<double> quat_lf(m_oMlfs.rotation());
        m_oMlfs_xyzquat(3) = quat_lf.w();
        m_oMlfs_xyzquat(4) = quat_lf.x();
        m_oMlfs_xyzquat(5) = quat_lf.y();
        m_oMlfs_xyzquat(6) = quat_lf.z();

        m_oMrfs_xyzquat.head<3>() = m_oMrfs.translation();
        Eigen::Quaternion<double> quat_rf(m_oMrfs.rotation());
        m_oMrfs_xyzquat(3) = quat_rf.w();
        m_oMrfs_xyzquat(4) = quat_rf.x();
        m_oMrfs_xyzquat(5) = quat_rf.y();
        m_oMrfs_xyzquat(6) = quat_rf.z();

        //save this poses to use it if no ref is provided
        m_oMlfs_default_ref = m_oMlfs;
        m_oMrfs_default_ref = m_oMrfs;
        
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
                                                SE3 & oMff, SE3 & oMfa, SE3 & fsMff)
      {
        Vector3 xyz;
        xyz << -ft[0]/K(0), -ft[1]/K(1), -ft[2]/K(2);
        Matrix3 R;
        rpyToMatrix(-ft[3]/K(3), -ft[4]/K(4), -ft[5]/K(5), R);
        const SE3 fsMfa(R, xyz);                          // foot sole to foot ankle
        oMfa = oMfs*fsMfa;                                // world to foot ankle
        const SE3 faMff(m_data->oMf[foot_id].inverse());  // foot ankle to free flyer
        //~ sendMsg("faMff (foot_id="+toString(foot_id)+"):\n" + toString(faMff), MSG_TYPE_INFO);
        //~ sendMsg("fsMfa (foot_id="+toString(foot_id)+"):\n" + toString(fsMfa), MSG_TYPE_INFO);
        oMff = oMfa*faMff;                                // world to free flyer
        fsMff = fsMfa*faMff;                              // foot sole to free flyer
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
        assert(qj.size()==m_robot_util->m_nbJoints     && "Unexpected size of signal joint_positions");
        assert(dq.size()==m_robot_util->m_nbJoints     && "Unexpected size of signal joint_velocities");

        /* convert sot to pinocchio joint order */
        m_robot_util->joints_sot_to_urdf(qj, m_q_pin.tail(m_robot_util->m_nbJoints));
        m_robot_util->joints_sot_to_urdf(dq, m_v_pin.tail(m_robot_util->m_nbJoints));

        getProfiler().start(PROFILE_BASE_KINEMATICS_COMPUTATION);

        /* Compute kinematics assuming world is at free-flyer frame */
        m_q_pin.head<6>().setZero();
        m_q_pin(6) = 1.0;
        m_v_pin.head<6>().setZero();
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
        if(s.size()!=m_robot_util->m_nbJoints+6)
          s.resize(m_robot_util->m_nbJoints+6);
        
        const Eigen::VectorXd & qj          = m_joint_positionsSIN(iter);     //n+6
        const Eigen::Vector4d & quatIMU_vec = m_imu_quaternionSIN(iter);
        const Vector6 & ftrf                = m_forceRLEGSIN(iter);
        const Vector6 & ftlf                = m_forceLLEGSIN(iter);

        // if the weights are not specified by the user through the input signals w_lf, w_rf
        // then compute them
        // if one feet is not stable, force weight to 0.0
        double wL, wR;
        if(m_w_lf_inSIN.isPlugged())
          wL = m_w_lf_inSIN(iter);
        else
        {
          wL = m_w_lf_filteredSOUT(iter);
          if (m_left_foot_is_stable == false)
            wL = 0.0;
        }
        if(m_w_rf_inSIN.isPlugged())
          wR = m_w_rf_inSIN(iter);
        else
        {
          wR = m_w_rf_filteredSOUT(iter);
          if (m_right_foot_is_stable == false)
            wR = 0.0;
        }

        assert(qj.size()==m_robot_util->m_nbJoints && "Unexpected size of signal joint_positions");

        // if both weights are zero set them to a small positive value to avoid division by zero
        if(wR==0.0 && wL==0.0)
        {
          wR = 1e-3;
          wL = 1e-3;
        }

        m_kinematics_computationsSINNER(iter);

        if(m_reset_foot_pos)
          reset_foot_positions_impl(ftlf, ftrf);

        getProfiler().start(PROFILE_BASE_POSITION_ESTIMATION);
        {
          SE3 oMlfa, oMrfa, lfsMff, rfsMff;
          kinematics_estimation(ftrf, m_K_rf, m_oMrfs, m_right_foot_id, m_oMff_rf, oMrfa, rfsMff);
          kinematics_estimation(ftlf, m_K_lf, m_oMlfs, m_left_foot_id,  m_oMff_lf, oMlfa, lfsMff);

          // get rpy
          const SE3 ffMchest(m_data->oMf[m_IMU_body_id]);  // transform between freeflyer and body attached to IMU sensor 
          const SE3 chestMff(ffMchest.inverse());          // transform between body attached to IMU sensor and freeflyer
          
          Vector3 rpy_chest, rpy_chest_lf, rpy_chest_rf, rpy_chest_imu; // orientation of the body which imu is attached to. (fusion, from left kine, from right kine, from imu)

          matrixToRpy((m_oMff_lf*ffMchest).rotation(), rpy_chest_lf);
          matrixToRpy((m_oMff_rf*ffMchest).rotation(), rpy_chest_rf);
          Eigen::Quaternion<double> quatIMU(quatIMU_vec[0], quatIMU_vec[1], quatIMU_vec[2], quatIMU_vec[3]);
          matrixToRpy(quatIMU.toRotationMatrix(), rpy_chest_imu); 

          // average (we do not take into account the IMU yaw)
          double wSum = wL + wR + m_w_imu;
          rpy_chest(0) = (rpy_chest_lf[0]*wL + rpy_chest_rf[0]*wR + rpy_chest_imu[0]*m_w_imu) / wSum;
          rpy_chest(1) = (rpy_chest_lf[1]*wL + rpy_chest_rf[1]*wR + rpy_chest_imu[1]*m_w_imu) / wSum;
          rpy_chest(2) = (rpy_chest_lf[2]*wL + rpy_chest_rf[2]*wR )                 / (wL+wR);

          rpyToMatrix(rpy_chest, m_oRchest);
          m_oRff = m_oRchest * chestMff.rotation();

          // translation to get foot at the right position
          // evaluate Mrl Mlf for q with the good orientation and zero translation for freeflyer
          const Vector3 pos_lf_ff = m_oRff * m_data->oMf[m_left_foot_id].translation();
          const Vector3 pos_rf_ff = m_oRff * m_data->oMf[m_right_foot_id].translation();
          // get average translation
          m_q_pin.head<3>() = ((oMlfa.translation() - pos_lf_ff)*wL +
                               (oMrfa.translation() - pos_rf_ff)*wR) / (wL+wR);

          m_q_sot.tail(m_robot_util->m_nbJoints) = qj;
          base_se3_to_sot(m_q_pin.head<3>(), m_oRff, m_q_sot.head<6>());

          s = m_q_sot;
          
          // store estimation of the base pose in SE3 format
          const SE3 oMff_est(m_oRff, m_q_pin.head<3>());
          
          // feedback on feet poses
          if(m_K_fb_feet_posesSIN.isPlugged())
          {
            const double K_fb = m_K_fb_feet_posesSIN(iter);
            if (K_fb > 0.0)
            {
              assert(m_w_imu > 0.0 && "Update of the feet 6d poses should not be done if wIMU = 0.0");
              assert(K_fb < 1.0 && "Feedback gain on foot correction should be less than 1.0 (K_fb_feet_poses>1.0)");
              //feet positions in the world according to current base estimation
              const SE3 oMlfs_est( oMff_est*(lfsMff.inverse()) );
              const SE3 oMrfs_est( oMff_est*(rfsMff.inverse()) );
              //error in current foot position
              SE3 leftDrift   = m_oMlfs.inverse()*oMlfs_est;
              SE3 rightDrift  = m_oMrfs.inverse()*oMrfs_est;

              ///apply feedback correction
              SE3 leftDrift_delta;
              SE3 rightDrift_delta;
              se3Interp(leftDrift ,SE3::Identity(),K_fb*wR,leftDrift_delta);
              se3Interp(rightDrift,SE3::Identity(),K_fb*wL,rightDrift_delta);
              // if a feet is not stable on the ground (aka flying), fully update his position 
              if (m_right_foot_is_stable == false)
                rightDrift_delta = leftDrift;
              if (m_left_foot_is_stable == false)
                leftDrift_delta  = rightDrift;
              if (m_right_foot_is_stable == false && m_left_foot_is_stable == false)
                {
                    //robot is jumping, do not update any feet position
                    rightDrift_delta = SE3::Identity();
                    leftDrift_delta = SE3::Identity();
                }
              m_oMlfs = m_oMlfs * leftDrift_delta;
              m_oMrfs = m_oMrfs * rightDrift_delta;
              // dedrift (x, y, z, yaw) using feet pose references
              SE3 oMlfs_ref, oMrfs_ref;
              if (m_lf_ref_xyzquatSIN.isPlugged() and
                  m_rf_ref_xyzquatSIN.isPlugged())
              {
                ///convert from xyzquat to se3
                const Vector7 & lf_ref_xyzquat_vec  = m_lf_ref_xyzquatSIN(iter);
                const Vector7 & rf_ref_xyzquat_vec  = m_rf_ref_xyzquatSIN(iter);
                const Eigen::Quaterniond ql(m_lf_ref_xyzquatSIN(iter)(3),
                                            m_lf_ref_xyzquatSIN(iter)(4),
                                            m_lf_ref_xyzquatSIN(iter)(5),
                                            m_lf_ref_xyzquatSIN(iter)(6));
                const Eigen::Quaterniond qr(m_rf_ref_xyzquatSIN(iter)(3),
                                            m_rf_ref_xyzquatSIN(iter)(4),
                                            m_rf_ref_xyzquatSIN(iter)(5),
                                            m_rf_ref_xyzquatSIN(iter)(6));
                oMlfs_ref = SE3(ql.toRotationMatrix(), lf_ref_xyzquat_vec.head<3>());
                oMrfs_ref = SE3(qr.toRotationMatrix(), rf_ref_xyzquat_vec.head<3>());
              }
              else
              {
                oMlfs_ref = m_oMlfs_default_ref;
                oMrfs_ref = m_oMrfs_default_ref;
              }
              ///find translation to apply to both feet to minimise distances to reference positions
              const Vector3 translation_feet_drift = 0.5*( ( oMlfs_ref.translation() - m_oMlfs.translation()) +
                                                           ( oMrfs_ref.translation() - m_oMrfs.translation()) );
              ///two vectors define by left to right feet translation
              const Vector3 V_ref =  oMrfs_ref.translation() - oMlfs_ref.translation();
              const Vector3 V_est =  m_oMrfs.translation()  - m_oMlfs.translation();
              /// angle betwin this two vectors projected in horizontal plane is the yaw drift
              const double yaw_drift = (atan2(V_ref(1), V_ref(0)) -
                                        atan2(V_est(1), V_est(0)));
              //~ printf("yaw_drift=%lf\r\n",yaw_drift);
              /// apply correction to cancel this drift
              const Vector3 rpy_feet_drift(0.,0.,yaw_drift);
              Matrix3 rotation_feet_drift;
              rpyToMatrix(rpy_feet_drift,rotation_feet_drift);
              const SE3 drift_to_ref(rotation_feet_drift , translation_feet_drift);
              m_oMlfs = m_oMlfs * drift_to_ref;
              m_oMrfs = m_oMrfs * drift_to_ref;
            }
          }
          // convert to xyz+quaternion format //Rq: this convertions could be done in outupt signals function?
          m_oMlfs_xyzquat.head<3>() = m_oMlfs.translation();
          Eigen::Quaternion<double> quat_lf(m_oMlfs.rotation());
          m_oMlfs_xyzquat(3) = quat_lf.w();
          m_oMlfs_xyzquat(4) = quat_lf.x();
          m_oMlfs_xyzquat(5) = quat_lf.y();
          m_oMlfs_xyzquat(6) = quat_lf.z();

          m_oMrfs_xyzquat.head<3>() = m_oMrfs.translation();
          Eigen::Quaternion<double> quat_rf(m_oMrfs.rotation());
          m_oMrfs_xyzquat(3) = quat_rf.w();
          m_oMrfs_xyzquat(4) = quat_rf.x();
          m_oMrfs_xyzquat(5) = quat_rf.y();
          m_oMrfs_xyzquat(6) = quat_rf.z();
        }
        getProfiler().stop(PROFILE_BASE_POSITION_ESTIMATION);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(lf_xyzquat, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal lf_xyzquat before initialization!");
          return s;
        }
        if(s.size()!=7)
          s.resize(7);
        const Eigen::VectorXd & q = m_qSOUT(iter);
        s = m_oMlfs_xyzquat;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(rf_xyzquat, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal rf_xyzquat before initialization!");
          return s;
        }
        if(s.size()!=7)
          s.resize(7);
        const Eigen::VectorXd & q = m_qSOUT(iter);
        s = m_oMrfs_xyzquat;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(q_lf, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal q_lf before initialization!");
          return s;
        }
        if(s.size()!=m_robot_util->m_nbJoints+6)
          s.resize(m_robot_util->m_nbJoints+6);

        const Eigen::VectorXd & q = m_qSOUT(iter);
        s.tail(m_robot_util->m_nbJoints) = q.tail(m_robot_util->m_nbJoints);
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
        if(s.size()!=m_robot_util->m_nbJoints+6)
          s.resize(m_robot_util->m_nbJoints+6);

        const Eigen::VectorXd & q = m_qSOUT(iter);
        s.tail(m_robot_util->m_nbJoints) = q.tail(m_robot_util->m_nbJoints);
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
        if(s.size()!=m_robot_util->m_nbJoints+6)
          s.resize(m_robot_util->m_nbJoints+6);

        const Eigen::VectorXd & q = m_qSOUT(iter);
        s.tail(m_robot_util->m_nbJoints) = q.tail(m_robot_util->m_nbJoints);

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
        zmp.setZero();
        compute_zmp(wrench, zmp);
        double w_zmp = compute_zmp_weight(zmp, m_left_foot_sizes,
                                          m_zmp_std_dev_lf, m_zmp_margin_lf);
        double w_fz = compute_force_weight(wrench(2), m_fz_std_dev_lf, m_fz_margin_lf);
        //check that foot is sensing a force greater than the margin treshold for more than 'm_fz_stable_windows_size' samples
        if (wrench(2) > m_fz_margin_lf)
            m_lf_fz_stable_cpt++;
        else m_lf_fz_stable_cpt = 0;

        if (m_lf_fz_stable_cpt >= m_fz_stable_windows_size)
        {
            m_lf_fz_stable_cpt = m_fz_stable_windows_size;
            m_left_foot_is_stable = true;
        }
        else
        {
            m_left_foot_is_stable = false;
        }
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
        zmp.setZero();
        compute_zmp(wrench, zmp);
        double w_zmp = compute_zmp_weight(zmp, m_right_foot_sizes,
                                          m_zmp_std_dev_rf, m_zmp_margin_rf);
        double w_fz = compute_force_weight(wrench(2), m_fz_std_dev_rf, m_fz_margin_rf);
        //check that foot is sensing a force greater than the margin treshold for more than 'm_fz_stable_windows_size' samples
        if (wrench(2) > m_fz_margin_rf)
            m_rf_fz_stable_cpt++;
        else m_rf_fz_stable_cpt = 0;

        if (m_rf_fz_stable_cpt >= m_fz_stable_windows_size)
        {
            m_rf_fz_stable_cpt = m_fz_stable_windows_size;
            m_right_foot_is_stable = true;
        }
        else
        {
            m_right_foot_is_stable = false;
        }
        s = w_zmp*w_fz;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(w_rf_filtered, double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal w_rf_filtered before initialization!");
          return s;
        }
        double w_rf = m_w_rfSOUT(iter);
        m_w_rf_filtered = m_alpha_w_filter*w_rf + (1-m_alpha_w_filter)*m_w_rf_filtered; //low pass filter
        s = m_w_rf_filtered;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(w_lf_filtered, double)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal w_lf_filtered before initialization!");
          return s;
        }
        double w_lf = m_w_lfSOUT(iter);
        m_w_lf_filtered = m_alpha_w_filter*w_lf + (1-m_alpha_w_filter)*m_w_lf_filtered; //low pass filter
        s = m_w_lf_filtered;
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(v,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v before initialization!");
          return s;
        }
        if(s.size()!=m_robot_util->m_nbJoints+6)
          s.resize(m_robot_util->m_nbJoints+6);

        m_kinematics_computationsSINNER(iter);
        m_qSOUT(iter);

        getProfiler().start(PROFILE_BASE_VELOCITY_ESTIMATION);
        {
          const Eigen::VectorXd& dq          = m_joint_velocitiesSIN(iter);
          const Eigen::Vector3d& acc_imu     = m_accelerometerSIN(iter);
          const Eigen::Vector3d& gyr_imu     = m_gyroscopeSIN(iter);
          const Eigen::Vector4d& quatIMU_vec = m_imu_quaternionSIN(iter);
          const Vector6 & dftrf              = m_dforceRLEGSIN(iter);
          const Vector6 & dftlf              = m_dforceLLEGSIN(iter);
          assert(dq.size()==m_robot_util->m_nbJoints     && "Unexpected size of signal joint_velocities");

          // if the weights are not specified by the user through the input signals w_lf, w_rf
          // then compute them
          // if one feet is not stable, force weight to 0.0
          double wL, wR;
          if(m_w_lf_inSIN.isPlugged())
            wL = m_w_lf_inSIN(iter);
          else
          {
            wL = m_w_lf_filteredSOUT(iter);
            if (m_left_foot_is_stable == false)
              wL = 0.0;
          }
          if(m_w_rf_inSIN.isPlugged())
            wR = m_w_rf_inSIN(iter);
          else
          {
            wR = m_w_rf_filteredSOUT(iter);
            if (m_right_foot_is_stable == false)
              wR = 0.0;
          }
          // if both weights are zero set them to a small positive value to avoid division by zero
          if(wR==0.0 && wL==0.0)
          {
            wR = 1e-3;
            wL = 1e-3;
          }

          /* Compute foot velocities */
          const Frame & f_lf = m_model.frames[m_left_foot_id];
          const Motion v_lf_local = m_data->v[f_lf.parent];
          const SE3 ffMlf = m_data->oMi[f_lf.parent];
          Vector6 v_kin_l = -ffMlf.act(v_lf_local).toVector();
          v_kin_l.head<3>() = m_oRff * v_kin_l.head<3>();

          const Frame & f_rf = m_model.frames[m_right_foot_id];
          const Motion v_rf_local = m_data->v[f_rf.parent];
          const SE3 ffMrf = m_data->oMi[f_rf.parent];
          Vector6 v_kin_r = -ffMrf.act(v_rf_local).toVector();
          v_kin_r.head<3>() = m_oRff * v_kin_r.head<3>();
          
          m_v_kin.head<6>() = (wR*v_kin_r + wL*v_kin_l)/(wL+wR); //this is the velocity of the base in the frame of the base.
          
          /* Compute velocity induced by the flexibility */
          Vector6 v_flex_l;
          Vector6 v_flex_r;
          v_flex_l << -dftlf[0]/m_K_lf(0), -dftlf[1]/m_K_lf(1), -dftlf[2]/m_K_lf(2),
                      -dftlf[3]/m_K_lf(3), -dftlf[4]/m_K_lf(4), -dftlf[5]/m_K_lf(5); 
          v_flex_r << -dftrf[0]/m_K_rf(0), -dftrf[1]/m_K_rf(1), -dftrf[2]/m_K_rf(2),
                      -dftrf[3]/m_K_rf(3), -dftrf[4]/m_K_rf(4), -dftrf[5]/m_K_rf(5);
          const Eigen::Matrix<double, 6, 6> lfAff = ffMlf.inverse().toActionMatrix();
          const Eigen::Matrix<double, 6, 6> rfAff = ffMrf.inverse().toActionMatrix();
          Eigen::Matrix<double, 12, 6> A;
          A << lfAff,
               rfAff;
          Eigen::Matrix<double, 12, 1> b;
          b << v_flex_l,
               v_flex_r;         
          //~ m_v_flex.head<6>() = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
          m_v_flex.head<6>() = (A.transpose() * A).ldlt().solve(A.transpose() * b);
          m_v_flex.head<3>() = m_oRff * m_v_flex.head<3>();



          /* Get an estimate of linear velocities from gyroscope only*/
          // we make the asumtion than we are 'turning around the foot' with pure angular velocity in the ankle measured by the gyro
          const Matrix3 ffRimu = (m_data->oMf[m_IMU_body_id]).rotation();
          const Matrix3 lfRimu = ffMlf.rotation().transpose() * ffRimu;
          const Matrix3 rfRimu = ffMrf.rotation().transpose() * ffRimu;

          const SE3 chestMimu(Matrix3::Identity(), Vector3(-0.13, 0.0,  0.118)); ///TODO Read this transform from setable parameter /// TODO chesk the sign of the translation
          const SE3 ffMchest(m_data->oMf[m_IMU_body_id]);
          const SE3 imuMff = (ffMchest * chestMimu).inverse();
          //gVw_a =  gVo_g + gHa.act(aVb_a)-gVb_g //angular velocity in the ankle from gyro and d_enc
          const Frame & f_imu = m_model.frames[m_IMU_body_id];
          Vector3 gVo_a_l = Vector3(gyr_imu(0),gyr_imu(1),gyr_imu(2)) + (imuMff*ffMlf).act(v_lf_local).angular() - m_data->v[f_imu.parent].angular();
          Vector3 gVo_a_r = Vector3(gyr_imu(0),gyr_imu(1),gyr_imu(2)) + (imuMff*ffMrf).act(v_rf_local).angular() - m_data->v[f_imu.parent].angular();
          Motion v_gyr_ankle_l( Vector3(0.,0.,0.),  lfRimu * gVo_a_l);
          Motion v_gyr_ankle_r( Vector3(0.,0.,0.),  rfRimu * gVo_a_r);
          Vector6 v_gyr_l = -ffMlf.inverse().act(v_gyr_ankle_l).toVector();
          Vector6 v_gyr_r = -ffMrf.inverse().act(v_gyr_ankle_r).toVector();
          m_v_gyr.head<6>() =  (wL*v_gyr_l + wR*v_gyr_r)/(wL+wR);


          /* Get an estimate of linear velocities from filtered accelerometer integration */
          
          /* rotate acceleration to express it in the world frame */
          //~ pointRotationByQuaternion(acc_imu,quatIMU_vec,acc_world); //this is false because yaw from imuquat is drifting
          const Vector3 acc_world = m_oRchest * acc_imu;

          /* now, the acceleration is expressed in the world frame, 
           * so it can be written as the sum of the proper acceleration and the 
           * constant gravity vector. We could remove this assuming a [0,0,9.81]
           * but we prefer to filter this signal with low frequency high pass 
           * filter since it is robust to gravity norm error, and we know that 
           * globaly the robot can not accelerate continuously. */
          
          ///* Extract DC component by low pass filter and remove it*/
          if (m_isFirstIterFlag) 
          {
            m_last_DCacc = acc_world; // Copy the first acceleration data
            m_isFirstIterFlag = false;
            sendMsg("iter:"+toString(iter)+"\n", MSG_TYPE_INFO);
          }
          const Vector3 DCacc = acc_world * (1-m_alpha_DC_acc) + m_last_DCacc * (m_alpha_DC_acc);
          //~ sendMsg("acc_world            :"+toString(acc_world)+"\n", MSG_TYPE_INFO);
          m_last_DCacc = DCacc;
          const Vector3 ACacc = acc_world - DCacc;
          m_v_ac = ACacc;
          /* Then this acceleration is integrated.  */
          const Vector3 vel = m_last_vel + ACacc * m_dt;
          m_last_vel = vel;
          /* To stabilise the integrated velocity, we add the 
           * asumption that globaly, velocity is zero. So we remove DC again */
          const Vector3 DCvel = vel * (1-m_alpha_DC_vel) + m_last_DCvel * (m_alpha_DC_vel);
          m_last_DCvel = DCvel;
          const Vector3 ACvel = vel - DCvel;
          m_v_ac = ACvel;

          /* Express back velocity in the imu frame to get a full 6d velocity with the gyro*/
          const Vector3 imuVimu = m_oRchest.transpose() * ACvel;
          /* Here we could remove dc from gyrometer to remove bias*/  ///TODO 
          const Motion imuWimu(imuVimu,gyr_imu);
          //const Frame & f_imu = m_model.frames[m_IMU_body_id];
          const Motion ffWchest = m_data->v[f_imu.parent];
          //const SE3 ffMchest(m_data->oMf[m_IMU_body_id]);
          //const SE3 chestMimu(Matrix3::Identity(), +1.0*Vector3(-0.13, 0.0,  0.118)); ///TODO Read this transform from setable parameter /// TODO chesk the sign of the translation
          const SE3 ffMimu = ffMchest * chestMimu;
          const Motion v= ffMimu.act(imuWimu) ;//- ffWchest;
          m_v_imu.head<6>() = v.toVector();
          m_v_imu.head<3>() = m_oRff * m_v_imu.head<3>();

          
          //~ m_v_sot.head<6>() = m_v_kin.head<6>();
          //~ m_v_sot.head<6>() = m_v_flex.head<6>() + m_v_kin.head<6>();
          m_v_sot.head<6>() = m_v_gyr.head<6>() + m_v_kin.head<6>();
//          m_v_sot.head<6>() = m_v_gyr.head<6>();
          //~ m_v_sot.head<6>() = m_v_imu.head<6>();

          m_v_sot.tail( m_robot_util->m_nbJoints) = dq;
          m_v_kin.tail( m_robot_util->m_nbJoints) = dq;
          m_v_flex.tail(m_robot_util->m_nbJoints) = dq;
          m_v_gyr.tail( m_robot_util->m_nbJoints) = dq;
          m_v_imu.tail( m_robot_util->m_nbJoints) = dq;
          s = m_v_sot;

        }
        getProfiler().stop(PROFILE_BASE_VELOCITY_ESTIMATION);
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(v_kin, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v_kin before initialization!");
          return s;
        }
        m_vSOUT(iter);
        s = m_v_kin;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(v_flex, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v_flex before initialization!");
          return s;
        }
        m_vSOUT(iter);
        s = m_v_flex+m_v_kin;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(v_imu, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v_imu before initialization!");
          return s;
        }
        m_vSOUT(iter);
        s = m_v_imu;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(v_gyr, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v_gyr before initialization!");
          return s;
        }
        m_vSOUT(iter);
        s = m_v_gyr;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(v_ac, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v_ac before initialization!");
          return s;
        }
        m_vSOUT(iter);
        s = m_v_ac;
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(a_ac, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal a_ac before initialization!");
          return s;
        }
        m_vSOUT(iter);
        s = m_a_ac;
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

