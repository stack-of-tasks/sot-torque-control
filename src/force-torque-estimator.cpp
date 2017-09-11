/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-dyninv is free software: you can redistribute it and/or
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

#include <sot/torque_control/force-torque-estimator.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/motor-model.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

#define FORCE_TORQUE_SENSORS_SIGNALS m_ftSensLeftFootSIN << m_ftSensRightFootSIN \
                                     << m_ftSensLeftHandSIN << m_ftSensRightHandSIN

#define MOTOR_PARAMETER_SIGNALS m_motorParameterKt_pSIN << m_motorParameterKt_nSIN \
                                << m_motorParameterKf_pSIN << m_motorParameterKf_nSIN \
                                << m_motorParameterKv_pSIN << m_motorParameterKv_nSIN \
                                << m_motorParameterKa_pSIN << m_motorParameterKa_nSIN

#define ALL_INPUT_SIGNALS m_base6d_encodersSIN << m_accelerometerSIN << m_gyroscopeSIN \
                          << m_ddqRefSIN << m_dqRefSIN << m_currentMeasureSIN \
                          << m_saturationCurrentSIN << m_wCurrentTrustSIN << m_tauDesSIN \
                          << FORCE_TORQUE_SENSORS_SIGNALS << MOTOR_PARAMETER_SIGNALS

#define KINEMATIC_OUTPUT_SIGNALS \
                          m_jointsPositionsSOUT << m_jointsVelocitiesSOUT \
                           << m_jointsAccelerationsSOUT << m_torsoAccelerationSOUT \
                           << m_torsoAngularVelocitySOUT

#define ALL_OUTPUT_SIGNALS KINEMATIC_OUTPUT_SIGNALS << m_contactWrenchLeftFootSOUT \
                           << m_contactWrenchRightFootSOUT << m_contactWrenchLeftHandSOUT \
                           << m_contactWrenchRightSoleSOUT << m_contactWrenchLeftSoleSOUT \
                           << m_contactWrenchRightHandSOUT << m_contactWrenchBodySOUT \
                           << m_jointsTorquesSOUT \
                           << m_jointsTorquesFromMotorModelSOUT \
                           << m_jointsTorquesFromInertiaModelSOUT \
                           << m_baseAccelerationSOUT << m_baseAngularVelocitySOUT \
                           << m_ftSensRightFootPredictionSOUT << m_currentFilteredSOUT \
                           << m_dynamicsErrorSOUT

//Size to be aligned                                 "-------------------------------------------------------"
#define PROFILE_JOINTS_TORQUES_COMPUTATION           "ForceTorqueEst: tau computation                        "
#define PROFILE_JOINTS_TORQUES_MOTOR_COMPUTATION     "ForceTorqueEst: tau from motor model computation       "
#define PROFILE_JOINTS_TORQUES_INERTIAL_COMPUTATION  "ForceTorqueEst: tau from inertial model computation    "

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace metapod;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef ForceTorqueEstimator EntityClassName;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ForceTorqueEstimator,"ForceTorqueEstimator");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      ForceTorqueEstimator::
      ForceTorqueEstimator( const std::string & name )
        : Entity(name),
        CONSTRUCT_SIGNAL_IN(base6d_encoders,   ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(accelerometer,    ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(gyroscope,        ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(ftSensLeftFoot,   ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(ftSensRightFoot,  ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(ftSensLeftHand,   ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(ftSensRightHand,  ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(ddqRef,           ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(dqRef,            ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(currentMeasure,   ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(saturationCurrent,ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(wCurrentTrust,    ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(tauDes, ml::Vector)
        ,CONSTRUCT_SIGNAL_OUT(ftSensRightFootPrediction,  ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(currentFiltered,         ml::Vector, m_currentMeasureSIN)
        ,CONSTRUCT_SIGNAL_OUT(jointsPositions,         ml::Vector, m_q_dq_ddqSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsVelocities,        ml::Vector, m_q_dq_ddqSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsAccelerations,     ml::Vector, m_q_dq_ddqSINNER)
        ,CONSTRUCT_SIGNAL_OUT(torsoAcceleration,       ml::Vector, m_w_dv_torsoSINNER)    // 6d
        ,CONSTRUCT_SIGNAL_OUT(torsoAngularVelocity,    ml::Vector, m_w_dv_torsoSINNER)    // 3d
        ,CONSTRUCT_SIGNAL_OUT(baseAcceleration,        ml::Vector, m_torques_wrenchesSINNER)    // 6d
        ,CONSTRUCT_SIGNAL_OUT(baseAngularVelocity,     ml::Vector, m_torques_wrenchesSINNER)    // 3d
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchLeftFoot,   ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchRightFoot,  ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchLeftSole,   ml::Vector, m_contactWrenchLeftFootSOUT)
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchRightSole,  ml::Vector, m_contactWrenchRightFootSOUT)
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchLeftHand,   ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchRightHand,  ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(contactWrenchBody,       ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsTorques,           ml::Vector, m_torques_wrenchesSINNER << m_torquesFromMotorModelSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsTorquesFromMotorModel,   ml::Vector, m_torquesFromMotorModelSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsTorquesFromInertiaModel, ml::Vector, m_torques_wrenchesSINNER)
        ,CONSTRUCT_SIGNAL_OUT(dynamicsError, ml::Vector, m_contactWrenchBodySOUT <<
                                                         m_jointsTorquesSOUT <<
                                                         m_tauDesSIN)

        ,CONSTRUCT_SIGNAL_INNER(torques_wrenches,      ml::Vector, FORCE_TORQUE_SENSORS_SIGNALS
                                                                <<KINEMATIC_OUTPUT_SIGNALS
                                                                <<m_ddqRefSIN
                                                                <<m_dqRefSIN
                                                                <<m_base6d_encodersSIN)
                                                                
        ,CONSTRUCT_SIGNAL_INNER(torquesFromMotorModel, ml::Vector, MOTOR_PARAMETER_SIGNALS
                                                                   << m_jointsVelocitiesSOUT 
                                                                   << m_jointsAccelerationsSOUT
                                                                   << m_currentFilteredSOUT)
        ,CONSTRUCT_SIGNAL_INNER(q_dq_ddq,              ml::Vector, m_base6d_encodersSIN)
        ,CONSTRUCT_SIGNAL_INNER(w_dv_torso,            ml::Vector, m_accelerometerSIN<<m_gyroscopeSIN)

        ,m_node_right_foot(boost::fusion::at_c<Hrp2_14::r_ankle>(m_robot.nodes))
        ,m_node_left_foot(boost::fusion::at_c<Hrp2_14::l_ankle>(m_robot.nodes))
        ,m_node_right_hand(boost::fusion::at_c<Hrp2_14::r_wrist>(m_robot.nodes))
        ,m_node_left_hand(boost::fusion::at_c<Hrp2_14::l_wrist>(m_robot.nodes))
        ,m_node_torso(boost::fusion::at_c<Hrp2_14::torso>(m_robot.nodes))
        ,m_node_chest_link0(boost::fusion::at_c<Hrp2_14::CHEST_LINK0>(m_robot.nodes))
        ,m_node_body(boost::fusion::at_c<Hrp2_14::BODY>(m_robot.nodes))
        ,m_is_first_iter(true)
        ,m_useRawEncoders(true)
        ,m_useRefJointsVel(true)
        ,m_useRefJointsAcc(true)
        ,m_useFTsensors(true)
        ,m_computeFTsensorOffsets(true)
      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);

        /* Commands. */
        addCommand("getTimestep",
                   makeDirectGetter(*this,&m_dt,
                                    docDirectGetter("Control timestep [s ]","double")));
        addCommand("getDelayEnc",
                   makeDirectGetter(*this,&m_delayEncoders,
                                    docDirectGetter("Delay introduced by the estimation of joints pos/vel/acc [s]","double")));
        addCommand("getDelayFTsens",
                   makeDirectGetter(*this,&m_delayFTsens,
                                    docDirectGetter("Delay introduced by the filtering of the F/T sensors [s]","double")));
        addCommand("getDelayAcc",
                   makeDirectGetter(*this,&m_delayAcc,
                                    docDirectGetter("Delay introduced by the filtering of the accelerometer [s]","double")));
        addCommand("getDelayGyro",
                   makeDirectGetter(*this,&m_delayGyro,
                                    docDirectGetter("Delay introduced by the filtering of the gyroscope [s]","double")));
        addCommand("getDelayCurrent",
                   makeDirectGetter(*this,&m_delayCurrent,
                                    docDirectGetter("Delay introduced by the filtering of the motor current measure [s]","double")));
        addCommand("getUseRefJointVel",
                   makeDirectGetter(*this,&m_useRefJointsVel,
                                    docDirectGetter("Whether to use reference joints vel to estimate joints torques and ext forces","bool")));
        addCommand("setUseRefJointVel",
                   makeDirectSetter(*this, &m_useRefJointsVel,
                                    docDirectSetter("flag specifying whether the reference (rather than the estimated) joints velocities are used to estimated joints torques",
                                                    "bool")));
        addCommand("getUseRefJointAcc",
                   makeDirectGetter(*this,&m_useRefJointsAcc,
                                    docDirectGetter("Whether to use reference joints acc to estimate joints torques and ext forces","bool")));
        addCommand("setUseRefJointAcc",
                   makeDirectSetter(*this, &m_useRefJointsAcc,
                                    docDirectSetter("flag specifying whether the reference (rather than the estimated) joints accelerations are used to estimated joints torques",
                                                    "bool")));
        addCommand("getUseRawEncoders",
                   makeDirectGetter(*this,&m_useRawEncoders,
                                    docDirectGetter("Whether to filter encoders to estimate joints torques and ext forces",
                                                    "bool")));
        addCommand("setUseRawEncoders",
                   makeDirectSetter(*this, &m_useRawEncoders,
                                    docDirectSetter("flag specifying whether to filter encoders to estimated joints torques and ext forces",
                                                    "bool")));
        addCommand("getUseFTsensors",
                   makeDirectGetter(*this,&m_useFTsensors,
                                    docDirectGetter("Whether to use the F/T sensors to estimate joints torques and ext forces",
                                                    "bool")));
        addCommand("setUseFTsensors",
                   makeDirectSetter(*this, &m_useFTsensors,
                                    docDirectSetter("flag specifying whether to use the F/T sensors to estimated joints torques and ext forces",
                                                    "bool")));
        addCommand("getFTsensorOffsets",
                   makeDirectGetter(*this,&m_FTsensorOffsets,
                                    docDirectGetter("Offset used for the four F/T sensors",
                                                    "vector")));
        addCommand("setFTsensorOffsets", makeCommandVoid1(*this, &ForceTorqueEstimator::setFTsensorOffsets,
                                         docCommandVoid1("Set the 4 F/T sensor offsets.",
                                                         "24-d vector [s].")));
        addCommand("recomputeFTsensorOffsets", makeCommandVoid0(*this, &ForceTorqueEstimator::recomputeFTsensorOffsets,
                                         docCommandVoid0("Recompute the 4 F/T sensor offsets.")));
        addCommand("getTauDesDelay",
                   makeDirectGetter(*this,&m_delayTauDes,
                                    docDirectGetter("delay (in s) to introduce in tauDes for computing dynamicsError",
                                                    "vector")));
        addCommand("setTauDesDelay",
                   makeDirectSetter(*this, &m_delayTauDes,
                                    docDirectSetter("delay (in s) to introduce in tauDes for computing dynamicsError",
                                                    "float")));

        addCommand("init", makeCommandVoid7(*this, &ForceTorqueEstimator::init,
                              docCommandVoid7("Initialize the estimator.",
                                              "Control timestep [s].",
                                              "Estimation delay for joints pos/vel/acc [s].",
                                              "Estimation delay for F/T sensors [s].",
                                              "Estimation delay for accelerometer [s].",
                                              "Estimation delay for gyroscope [s].",
                                              "Estimation delay for motor current [s].",
                                              "If true the offset of the force sensors is computed.")));
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void ForceTorqueEstimator::init(const double &timestep, const double& delayEncoders,
                                      const double& delayFTsens, const double& delayAcc, const double& delayGyro, const double& delayCurrent,
                                      const bool &computeForceSensorsOffsets)
      {
        assert(timestep>0.0 && "Timestep should be > 0");
        assert(delayEncoders>=1.5*timestep && "Estimation delay for encoders should be >= 1.5*timestep");
        assert(delayFTsens>=timestep && "Estimation delay for F/T sensors should be >= timestep");
        assert(delayAcc>=timestep && "Estimation delay for accelerometer should be >= timestep");
        assert(delayGyro>=timestep && "Estimation delay for gyroscope should be >= timestep");
        assert(delayCurrent>=timestep && "Estimation delay for motor current should be >= timestep");
        m_dt = timestep;
        m_delayEncoders = delayEncoders;
        m_delayFTsens   = delayFTsens;
        m_delayAcc      = delayAcc;
        m_delayGyro     = delayGyro;
        m_delayCurrent  = delayCurrent;
        m_computeFTsensorOffsets = computeForceSensorsOffsets;
        int winSizeEnc     = (int)(2*delayEncoders/m_dt);
        int winSizeFT      = (int)(2*delayFTsens/m_dt);
        int winSizeAcc     = (int)(2*delayAcc/m_dt);
        int winSizeGyro    = (int)(2*delayGyro/m_dt);
        int winSizeCur     = (int)(2*delayCurrent/m_dt);
        assert(winSizeEnc>=3 && "Estimation-window's length for encoders should be >= 3");
        assert(winSizeFT>=2 && "Estimation-window's length for F/T sensors should be >= 2");
        assert(winSizeAcc>=2 && "Estimation-window's length for accelerometer should be >= 2");
        assert(winSizeGyro>=2 && "Estimation-window's length for gyroscope should be >= 2");

        m_encodersFilter         = new QuadEstimator(winSizeEnc, N_JOINTS, m_dt);
        m_accelerometerFilter    = new LinEstimator(winSizeAcc, 3, m_dt);
        m_gyroscopeFilter        = new LinEstimator(winSizeGyro, 3, m_dt);
        m_ftSensLeftFootFilter   = new LinEstimator(winSizeFT, 6, m_dt);
        m_ftSensRightFootFilter  = new LinEstimator(winSizeFT, 6, m_dt);
        m_ftSensLeftHandFilter   = new LinEstimator(winSizeFT, 6, m_dt);
        m_ftSensRightHandFilter  = new LinEstimator(winSizeFT, 6, m_dt);
        m_currentMeasureFilter   = new LinEstimator(winSizeCur, N_JOINTS, m_dt);

        m_delayTauDes = 0.0;
        m_tauDesBuffer = boost::circular_buffer<ml::Vector>(winSizeFT/2);
        m_tauBuffer = boost::circular_buffer<ml::Vector>(2);

        m_ddq_filter_std.resize(N_JOINTS);
        m_dq_filter_std.resize(N_JOINTS);
        m_q_filter_std.resize(N_JOINTS);
        m_q_std.resize(N_JOINTS);
        m_currentMeasure_std.resize(N_JOINTS);
        m_currentMeasure_filter_std.resize(N_JOINTS);
        m_saturationCurrent_std.resize(N_JOINTS);
        m_wCurrentTrust_std.resize(N_JOINTS);
        m_motorParameterKt_p_std.resize(N_JOINTS);
        m_motorParameterKt_n_std.resize(N_JOINTS);
        m_motorParameterKf_p_std.resize(N_JOINTS);
        m_motorParameterKf_n_std.resize(N_JOINTS);
        m_motorParameterKv_p_std.resize(N_JOINTS);
        m_motorParameterKv_n_std.resize(N_JOINTS);
        m_motorParameterKa_p_std.resize(N_JOINTS);
        m_motorParameterKa_n_std.resize(N_JOINTS);
        m_dv_IMU_std.resize(3);
        m_dv_IMU_filter_std.resize(3);
        m_w_IMU_std.resize(3);
        m_w_IMU_filter_std.resize(3);
        m_dw_IMU_filter_std.resize(3);
        m_ftSens_LH_std.resize(6);
        m_ftSens_LH_filter_std.resize(6);
        m_ftSens_RH_std.resize(6);
        m_ftSens_RH_filter_std.resize(6);
        m_ftSens_LF_std.resize(6);
        m_ftSens_LF_filter_std.resize(6);
        m_ftSens_RF_std.resize(6);
        m_ftSens_RF_filter_std.resize(6);

        m_ftSensLeftHand_offset.setZero(6);
        m_ftSensRightHand_offset.setZero(6);
        m_ftSensLeftFoot_offset.setZero(6);
        m_ftSensRightFoot_offset.setZero(6);
        m_FTsensorOffsets.resize(6*4);
        m_FTsensorOffsets.setZero();

        // since the base's position/orientation and linear velocity are not computed
        // we need to set q and dq to 0 during initialization
        m_q.setZero();
        m_dq.setZero();
        m_v_torso.v(Vector3d::Zero());

        // compute transformations from force/torque-sensor frames to hosting-link frame
        // The second argument of the Transform constructor has to be the position
        // of the body frame origin (i.e. the joint) w.r.t. the force/torque sensor
        Spatial::RotationMatrixIdentityTpl<double> eye;
        Spatial::RotationMatrixAboutZTpl<double> R_lh(cos(RIGHT_HAND_FORCE_SENSOR_Z_ROTATION), sin(RIGHT_HAND_FORCE_SENSOR_Z_ROTATION));
        Spatial::RotationMatrixAboutZTpl<double> R_rh(cos(LEFT_HAND_FORCE_SENSOR_Z_ROTATION),  sin(LEFT_HAND_FORCE_SENSOR_Z_ROTATION));
        typedef Map<const Vector3d> CMap3d;
        m_RF_X_ftSens = TransformNoRot(eye, - CMap3d(RIGHT_FOOT_FORCE_SENSOR_XYZ));
        m_LF_X_ftSens = TransformNoRot(eye, - CMap3d(LEFT_FOOT_FORCE_SENSOR_XYZ));
        m_RH_X_ftSens = TransformRotZ(R_rh, - CMap3d(RIGHT_HAND_FORCE_SENSOR_XYZ));
        m_LH_X_ftSens = TransformRotZ(R_lh, - CMap3d(LEFT_HAND_FORCE_SENSOR_XYZ));
        // compute transformation from foot frame to sole frame
        m_sole_X_RF = TransformNoRot(eye, CMap3d(RIGHT_FOOT_SOLE_XYZ));
        m_sole_X_LF = TransformNoRot(eye, CMap3d(LEFT_FOOT_SOLE_XYZ));
        // compute transformation from IMU's frame to torso's frame
        m_torso_X_imu = TransformNoRot(eye, -CMap3d(IMU_XYZ));
      }

      void ForceTorqueEstimator::setFTsensorOffsets(const ml::Vector& offsets)
      {
        m_FTsensorOffsets = offsets;
        // copy F/T sensor offsets from mal vector to eigen vectors
        for(int i=0;i<6;i++)
        {
          m_ftSensRightFoot_offset(i) = m_FTsensorOffsets(i);
          m_ftSensLeftFoot_offset(i)  = m_FTsensorOffsets(6+i);
          m_ftSensRightHand_offset(i) = m_FTsensorOffsets(12+i);
          m_ftSensLeftHand_offset(i)  = m_FTsensorOffsets(18+i);
        }
      }
      
      void ForceTorqueEstimator::recomputeFTsensorOffsets()
      {
        m_is_first_iter = true;
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      /** Estimate the joints' positions, velocities and accelerations. */
      DEFINE_SIGNAL_INNER_FUNCTION(q_dq_ddq, ml::Vector)
      {
        sotDEBUG(15)<<"Compute q_dq_ddq inner signal "<<iter<<endl;

        // read encoders and copy in std vector
        const ml::Vector& base_q = m_base6d_encodersSIN(iter);
        COPY_SHIFTED_VECTOR_TO_ARRAY(base_q, m_q_std, 6);


        // estimate joints' pos, vel and acc
        m_encodersFilter->estimate(m_q_filter_std, m_q_std);
        m_encodersFilter->getEstimateDerivative(m_dq_filter_std, 1);
        m_encodersFilter->getEstimateDerivative(m_ddq_filter_std, 2);

        // copy data in signal vector
        if(s.size()!=3*N_JOINTS)
          s.resize(3*N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = m_q_filter_std[i];
        for(int i=0; i<N_JOINTS; i++)
          s(i+N_JOINTS) = m_dq_filter_std[i];
        for(int i=0; i<N_JOINTS; i++)
          s(i+2*N_JOINTS) = m_ddq_filter_std[i];

        return s;
      }


      /** Estimate the torso's angular velocity. and linear/angular acceleration. */
      DEFINE_SIGNAL_INNER_FUNCTION(w_dv_torso, ml::Vector)
      {
//        SEND_MSG("Compute w_dv_torso inner signal "+toString(iter), MSG_TYPE_DEBUG);
        // read accelerometer and gyroscope and copy in std vector
        COPY_VECTOR_TO_ARRAY(m_accelerometerSIN(iter), m_dv_IMU_std);
        COPY_VECTOR_TO_ARRAY(m_gyroscopeSIN(iter),     m_w_IMU_std);

        // estimate lin/ang acceleration and ang vel
        m_accelerometerFilter->estimate(m_dv_IMU_filter_std, m_dv_IMU_std);
        m_gyroscopeFilter->estimate(m_w_IMU_filter_std, m_w_IMU_std);
        m_gyroscopeFilter->getEstimateDerivative(m_dw_IMU_filter_std, 1);

        // map data from IMU's frame to torso's frame
        EIGEN_VECTOR_FROM_STD_VECTOR(dv_IMU_eig, m_dv_IMU_filter_std);
        EIGEN_VECTOR_FROM_STD_VECTOR(w_IMU_eig, m_w_IMU_filter_std);
        EIGEN_VECTOR_FROM_STD_VECTOR(dw_IMU_eig, m_dw_IMU_filter_std);
        m_v_torso.w(w_IMU_eig);
        m_dv_torso.v(dv_IMU_eig);
        m_dv_torso.w(dw_IMU_eig);
        m_v_torso  = m_torso_X_imu.apply(m_v_torso);
        m_dv_torso = m_torso_X_imu.apply(m_dv_torso);

        // copy data in signal vector
        if(s.size()!=9)
          s.resize(9);
        int i;
        for(i=0; i<3; i++)
          s(i)   = m_v_torso.w()[i];
        for(i=0; i<3; i++)
          s(i+3) = m_dv_torso.v()[i];
        for(i=0; i<3; i++)
          s(i+6) = m_dv_torso.w()[i];
        return s;
      }


      /** Estimate the joints' torques from the motor model and current measurment */
      DEFINE_SIGNAL_INNER_FUNCTION(torquesFromMotorModel, ml::Vector)
      {
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);

        getProfiler().start(PROFILE_JOINTS_TORQUES_MOTOR_COMPUTATION);
        {

          const ml::Vector& currentFiltered          = m_currentFilteredSOUT(iter);
          // copy motor model parameters from mal to std vectors
          COPY_VECTOR_TO_ARRAY(m_motorParameterKt_pSIN(iter), m_motorParameterKt_p_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKt_nSIN(iter), m_motorParameterKt_n_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKf_pSIN(iter), m_motorParameterKf_p_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKf_nSIN(iter), m_motorParameterKf_n_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKv_pSIN(iter), m_motorParameterKv_p_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKv_nSIN(iter), m_motorParameterKv_n_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKa_pSIN(iter), m_motorParameterKa_p_std  );
          COPY_VECTOR_TO_ARRAY(m_motorParameterKa_nSIN(iter), m_motorParameterKa_n_std  );
          /// *** Get Joints Torques from gearmotors models
          
          for(int i=0; i<N_JOINTS; i++)
          {                                     
                  double torqueFromCurrent = motorModel.getTorque( currentFiltered(i) , m_dq(i+6), m_ddq(i+6),
                                                                   m_motorParameterKt_p_std[i], m_motorParameterKt_n_std[i],
                                                                   m_motorParameterKf_p_std[i], m_motorParameterKf_n_std[i],
                                                                   m_motorParameterKv_p_std[i], m_motorParameterKv_n_std[i],
                                                                   m_motorParameterKa_p_std[i], m_motorParameterKa_n_std[i] );
                  s(i)=torqueFromCurrent;
                  //m_torques(i+6) = m_torques(i+6)*(1-m_wCurrentTrust_std[i])+m_wCurrentTrust_std[i]*torqueFromCurrent ; 
          }
        }
        getProfiler().stop(PROFILE_JOINTS_TORQUES_MOTOR_COMPUTATION);
        return s;
      }
      /** Estimate the joints' torques and the 5 contact wrenches (feet, hand and body). */
      DEFINE_SIGNAL_INNER_FUNCTION(torques_wrenches, ml::Vector)
      {
        if(s.size()!=N_JOINTS+5*6)
          s.resize(N_JOINTS+5*6);

        getProfiler().start(PROFILE_JOINTS_TORQUES_INERTIAL_COMPUTATION);
        {
          // map data from mal to eigen vectors
          EIGEN_CONST_VECTOR_FROM_SIGNAL(w_torso_eig,  m_torsoAngularVelocitySOUT(iter));
          EIGEN_CONST_VECTOR_FROM_SIGNAL(dv_torso_eig, m_torsoAccelerationSOUT(iter));

          /// COMPUTE BASE ANGULAR VELOCITY AND ACCELERATION FROM IMU MEASUREMENTS
          if(m_useRawEncoders)
          {
            EIGEN_CONST_VECTOR_FROM_SIGNAL(base6d_encoders, m_base6d_encodersSIN(iter));
            m_q.tail<N_JOINTS>()    = base6d_encoders.tail<N_JOINTS>();
          }
          else
          {
            EIGEN_CONST_VECTOR_FROM_SIGNAL(q_eig,    m_jointsPositionsSOUT(iter));
            m_q.tail<N_JOINTS>()    = q_eig;
          }

          if(m_useRefJointsVel)
          {
            EIGEN_CONST_VECTOR_FROM_SIGNAL(dq_ref,       m_dqRefSIN(iter));
            m_dq.tail<N_JOINTS>()   = dq_ref;
          }
          else
          {
            EIGEN_CONST_VECTOR_FROM_SIGNAL(dq_eig,       m_jointsVelocitiesSOUT(iter));
            m_dq.tail<N_JOINTS>()   = dq_eig;
          }

          // compute homogeneous transformations (iX0 and sXp)
          // The iX0's are needed to transform the external wrenches to the base frame
          bcalc< Hrp2_14>::run(m_robot, m_q);

          // compute homogeneous transformations (sXp) and local velocities (vj)
          // The vj's are needed to propagate the torso's vel/acc to the base
          // Moreover jcalc is necessary for the RNEA, so we should compute it anyway
          jcalc< Hrp2_14>::run(m_robot, m_q, m_dq);

          // @todo: calling bcalc and jcalc is redundant because they both compute
          // sXp, so it'd be better to create a new algorithm that computes iX0, vj
          // and sXp without any overhead

          // Compute acceleration and angular velocity of base from those of the torso
          // set vel/acc of the torso link
          m_node_torso.body.vi.w(w_torso_eig);
          m_node_torso.body.ai.v(dv_torso_eig.head<3>());
          m_node_torso.body.ai.w(dv_torso_eig.tail<3>());

          // propagate vel/acc from torso to base link
          if(m_useRefJointsAcc)
          {
            EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq_ref,    m_ddqRefSIN(iter));
            m_ddq.tail<N_JOINTS>()  = ddq_ref;
          }
          else
          {
            EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq_eig,  m_jointsAccelerationsSOUT(iter));
            m_ddq.tail<N_JOINTS>()  = ddq_eig;
          }
          Vector1d ddqi = m_ddq.segment<1>(m_node_torso.q_idx);
          update_kinematics_backward<Hrp2_14, Hrp2_14::torso, Hrp2_14::CHEST_LINK0>::run(m_robot, ddqi);
          ddqi = m_ddq.segment<1>(m_node_chest_link0.q_idx);
          update_kinematics_backward<Hrp2_14, Hrp2_14::CHEST_LINK0, Hrp2_14::BODY>::run(m_robot, ddqi);
          // read vel/acc of base link
          // free flyer acc is lin+ang (disagree with Featherstone convention used by metapod)
          m_dq.segment<3>(3)  = m_node_body.body.vi.w();  // ang vel
          if( ::isnan(m_node_body.body.ai.v()[0]))
          {
            SEND_MSG("WARN nan detected on base linear velocity. Setting to (0,0,9.81) for hot fix ", MSG_TYPE_INFO);
            m_ddq.head<3>() = Vector3d::Zero();
            m_ddq(2) = 9.81;
          }
          else
            m_ddq.head<3>()     = m_node_body.body.ai.v();  // lin acc
          m_ddq.segment<3>(3) = m_node_body.body.ai.w();  // ang acc
          // remove gravity acceleration from IMU's measurement
          m_ddq(2)            -= 9.81; // assume the world z axis points upwards

          // since the velocity of the free-flyer is different from when we called
          // jcalc, we have to recompute jcalc for the base link before calling RNEA
          // with the jcalc flag to false (to avoid redoing the same computation)
          m_node_body.joint.jcalc(m_q.segment<6>(0), m_dq.segment<6>(0));


          Spatial::ForceTpl<double> zeroForce(Vector6d::Zero());
          m_node_right_foot.body.Fext = zeroForce;
          m_node_left_foot.body.Fext  = zeroForce;
          m_node_right_hand.body.Fext = zeroForce;
          m_node_left_hand.body.Fext  = zeroForce;

          if(m_useFTsensors)
          {
            /// *** COMPENSATE FOR WEIGHT MEASURED BY F/T SENSORS AND FILTER F/T MEASUREMENTS
            // Compute RNEA to compute F/T measurements in case of no external forces
            // The 2nd template argument is false because we have already computed jcalc.
            rnea< Hrp2_14, false>::run(m_robot, m_q, m_dq, m_ddq);

            // copy force/torque measurements from mal to std vectors
            COPY_VECTOR_TO_ARRAY(m_ftSensLeftHandSIN(iter),  m_ftSens_LH_std);
            COPY_VECTOR_TO_ARRAY(m_ftSensRightHandSIN(iter), m_ftSens_RH_std);
            COPY_VECTOR_TO_ARRAY(m_ftSensLeftFootSIN(iter),  m_ftSens_LF_std);
            COPY_VECTOR_TO_ARRAY(m_ftSensRightFootSIN(iter), m_ftSens_RF_std);

            // filter force/torque sensors' measurements
            m_ftSensLeftHandFilter->estimate(m_ftSens_LH_filter_std, m_ftSens_LH_std);
            m_ftSensRightHandFilter->estimate(m_ftSens_RH_filter_std, m_ftSens_RH_std);
            m_ftSensLeftFootFilter->estimate(m_ftSens_LF_filter_std, m_ftSens_LF_std);
            m_ftSensRightFootFilter->estimate(m_ftSens_RF_filter_std, m_ftSens_RF_std);

            // map filtered force/torque measurements from std to Eigen vectors
            EIGEN_VECTOR_FROM_STD_VECTOR(ftSens_LH_filter_eig, m_ftSens_LH_filter_std);
            EIGEN_VECTOR_FROM_STD_VECTOR(ftSens_RH_filter_eig, m_ftSens_RH_filter_std);
            EIGEN_VECTOR_FROM_STD_VECTOR(ftSens_LF_filter_eig, m_ftSens_LF_filter_std);
            EIGEN_VECTOR_FROM_STD_VECTOR(ftSens_RF_filter_eig, m_ftSens_RF_filter_std);

            if(m_is_first_iter && m_computeFTsensorOffsets)
            {
              // compute force sensor offsets assuming there is no contact at start
              m_ftSensRightFoot_offset.head<3>() = ftSens_RF_filter_eig.head<3>() + RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.f();
              m_ftSensRightFoot_offset.tail<3>() = ftSens_RF_filter_eig.tail<3>() + RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.n();
              m_ftSensLeftFoot_offset.head<3>()  = ftSens_LF_filter_eig.head<3>() + LEFT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_foot.joint.f.f();
              m_ftSensLeftFoot_offset.tail<3>()  = ftSens_LF_filter_eig.tail<3>() + LEFT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_foot.joint.f.n();
              m_ftSensRightHand_offset.head<3>() = ftSens_RH_filter_eig.head<3>() + RIGHT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_hand.joint.f.f();
              m_ftSensRightHand_offset.tail<3>() = ftSens_RH_filter_eig.tail<3>() + RIGHT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_hand.joint.f.n();
              m_ftSensLeftHand_offset.head<3>()  = ftSens_LH_filter_eig.head<3>() + LEFT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_hand.joint.f.f();
              m_ftSensLeftHand_offset.tail<3>()  = ftSens_LH_filter_eig.tail<3>() + LEFT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_hand.joint.f.n();
              
              SEND_MSG("w_torso_eig "+toString(w_torso_eig), MSG_TYPE_INFO);  
              SEND_MSG("dv_torso_eig "+toString(dv_torso_eig), MSG_TYPE_INFO);  
              SEND_MSG("ddq "+toString(m_ddq.transpose()), MSG_TYPE_INFO);  
              SEND_MSG("Foot weight "+toString(m_node_right_foot.joint.f.f()), MSG_TYPE_INFO);
              // copy F/T sensor offsets in mal vector read by the command getFTsensorOffsets()
              for(int i=0;i<6;i++)
              {
                m_FTsensorOffsets(i) = m_ftSensRightFoot_offset(i);
                m_FTsensorOffsets(6+i) = m_ftSensLeftFoot_offset(i);
                m_FTsensorOffsets(12+i) = m_ftSensRightHand_offset(i);
                m_FTsensorOffsets(18+i) = m_ftSensLeftHand_offset(i);
              }

              m_is_first_iter = false;
              SEND_MSG("Computing force/torque sensor offsets: the robot should make no contact with hands and feet at this moment", MSG_TYPE_WARNING);
              SEND_MSG("Force/torque sensor offsets is "+toString(m_FTsensorOffsets), MSG_TYPE_INFO);
            }

            // remove offset and prediction from force/torque sensors measurements
            ftSens_LH_filter_eig.head<3>() -= (m_ftSensLeftHand_offset.head<3>() - LEFT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_hand.joint.f.f());
            ftSens_LH_filter_eig.tail<3>() -= (m_ftSensLeftHand_offset.tail<3>() - LEFT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_hand.joint.f.n());
            ftSens_RH_filter_eig.head<3>() -= (m_ftSensRightHand_offset.head<3>() - RIGHT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_hand.joint.f.f());
            ftSens_RH_filter_eig.tail<3>() -= (m_ftSensRightHand_offset.tail<3>() - RIGHT_HAND_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_hand.joint.f.n());
            ftSens_LF_filter_eig.head<3>() -= (m_ftSensLeftFoot_offset.head<3>() - LEFT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_foot.joint.f.f());
            ftSens_LF_filter_eig.tail<3>() -= (m_ftSensLeftFoot_offset.tail<3>() - LEFT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_left_foot.joint.f.n());
            ftSens_RF_filter_eig.head<3>() -= (m_ftSensRightFoot_offset.head<3>() - RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.f());
            ftSens_RF_filter_eig.tail<3>() -= (m_ftSensRightFoot_offset.tail<3>() - RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.n());

            // convert forces from Eigen vectors to ForceTpl (swap first 3 elements with last 3 elements)
            Spatial::ForceTpl<double> f_RF(ftSens_RF_filter_eig.tail<3>(), ftSens_RF_filter_eig.head<3>());
            Spatial::ForceTpl<double> f_LF(ftSens_LF_filter_eig.tail<3>(), ftSens_LF_filter_eig.head<3>());
            Spatial::ForceTpl<double> f_RH(ftSens_RH_filter_eig.tail<3>(), ftSens_RH_filter_eig.head<3>());
            Spatial::ForceTpl<double> f_LH(ftSens_LH_filter_eig.tail<3>(), ftSens_LH_filter_eig.head<3>());

            // Map forces from F/T sensor frame to link frame (compute equivalent forces)
            f_RF = m_RF_X_ftSens.apply(f_RF);
            f_LF = m_LF_X_ftSens.apply(f_LF);
            f_RH = m_RH_X_ftSens.apply(f_RH);
            f_LH = m_LH_X_ftSens.apply(f_LH);

            // compute equivalent forces at the world frame and assign them to feet and hands
            m_node_right_foot.body.Fext = m_node_right_foot.body.iX0.applyInv(f_RF);
            m_node_left_foot.body.Fext  = m_node_left_foot.body.iX0.applyInv(f_LF);
            m_node_right_hand.body.Fext = m_node_right_hand.body.iX0.applyInv(f_RH);
            m_node_left_hand.body.Fext  = m_node_left_hand.body.iX0.applyInv(f_LH);

            for(int i=0; i<3; i++)
            {
              s(N_JOINTS+6*m_INDEX_WRENCH_LEFT_HAND+i)   = f_LH.f()[i];
              s(N_JOINTS+6*m_INDEX_WRENCH_LEFT_HAND+i+3) = f_LH.n()[i];

              s(N_JOINTS+6*m_INDEX_WRENCH_RIGHT_HAND+i)   = f_RH.f()[i];
              s(N_JOINTS+6*m_INDEX_WRENCH_RIGHT_HAND+i+3) = f_RH.n()[i];

              s(N_JOINTS+6*m_INDEX_WRENCH_LEFT_FOOT+i)   = f_LF.f()[i];
              s(N_JOINTS+6*m_INDEX_WRENCH_LEFT_FOOT+i+3) = f_LF.n()[i];

              s(N_JOINTS+6*m_INDEX_WRENCH_RIGHT_FOOT+i)   = f_RF.f()[i];
              s(N_JOINTS+6*m_INDEX_WRENCH_RIGHT_FOOT+i+3) = f_RF.n()[i];
            }
          }

          /// *** COMPUTE RNEA TO ESTIMATE JOINTS TORQUES
          // Second 3 coordinates of m_q are roll-pitch-yaw orientation of the base.
          // The 2nd template argument is false because we have already computed jcalc.
          rnea< Hrp2_14, false>::run(m_robot, m_q, m_dq, m_ddq);
          getTorques(m_robot, m_torques);
        }
        getProfiler().stop(PROFILE_JOINTS_TORQUES_INERTIAL_COMPUTATION);

//#define DEBUG_RNEA_BACKWARD
#ifdef DEBUG_RNEA_BACKWARD
        sotDEBUG(20)<<iter<<" PRE-RNEA torso acc:     "<<dv_torso_eig.transpose()<<endl;
        sotDEBUG(20)<<iter<<" POST-RNEA torso acc:     "<<m_node_torso.body.ai.v().transpose();
        sotDEBUG(20)<<" "<<m_node_torso.body.ai.w().transpose()<<endl;
        sotDEBUG(10)<<iter<<" POST-RNEA base body acc: "<<m_node_body.body.ai.v().transpose();
        sotDEBUG(10)<<" "<<m_node_body.body.ai.w().transpose()<<endl;
        sotDEBUG(10)<<iter<<" RNEA ddq(1:6):      "<<m_ddq.head<6>().transpose()<<endl;
#endif

//        SEND_DEBUG_STREAM_MSG("q: "+toString(m_q.transpose()));
//        SEND_DEBUG_STREAM_MSG("dq: "+toString(m_dq.transpose()));
//        SEND_MSG("w torso: "+toString(w_torso_eig.transpose()), MSG_TYPE_DEBUG);
//        SEND_MSG("dv torso: "+toString(dv_torso_eig.transpose()), MSG_TYPE_DEBUG);
//        SEND_MSG("ddq: "+toString(m_ddq.transpose()), MSG_TYPE_DEBUG);
//        SEND_MSG("Tau: "+toString(m_torques.transpose()), MSG_TYPE_DEBUG);
        //SEND_MSG("Tau without using current: "+toString(m_torques.transpose()), MSG_TYPE_DEBUG);

        // copy estimated joints' torques to output signal
        for(int i=0; i<N_JOINTS; i++)
          s(i) = m_torques(i+6);
        for(int i=0; i<6; i++)
          s(N_JOINTS+6*m_INDEX_WRENCH_BODY+i)   = m_torques(i);
        //SEND_MSG("currentMeasure: "+toString(m_currentMeasure_std), MSG_TYPE_DEBUG);
        //SEND_MSG("currentMeasure filter: "+toString(m_currentMeasure_filter_std), MSG_TYPE_DEBUG);
        return s;
      }


      /**  */
      DEFINE_SIGNAL_OUT_FUNCTION(ftSensRightFootPrediction, ml::Vector)
      {
        m_torques_wrenchesSINNER(iter);

        // compute equivalent forces at the world frame and assign them to feet and hands
        Spatial::ForceTpl<double> zeroForce(Vector6d::Zero());
        m_node_right_foot.body.Fext = zeroForce;
        m_node_left_foot.body.Fext  = zeroForce;
        m_node_right_hand.body.Fext = zeroForce;
        m_node_left_hand.body.Fext  = zeroForce;

        // neglect accelerations when estimating f/t sensor measurement
        m_ddq.tail<N_JOINTS>().setZero();

        // Compute RNEA to estimate joints' torques and contact wrenches.
        // Second 3 coordinates of m_q are roll-pitch-yaw orientation of the base.
        // The 2nd template argument is false because we have already computed jcalc.
        rnea< Hrp2_14, false>::run(m_robot, m_q, m_dq, m_ddq);

        // copy data
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<3; i++)
        {
          s(i)   = m_ftSensRightFoot_offset[i]   - RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.f()[i];
          s(i+3) = m_ftSensRightFoot_offset[i+3] - RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.n()[i];
        }

        if(m_is_first_iter)
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(ft_RF, m_ftSensRightFootSIN(iter));
          m_ftSensRightFoot_offset.head<3>() = ft_RF.head<3>() + RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.f();
          m_ftSensRightFoot_offset.tail<3>() = ft_RF.tail<3>() + RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE*m_node_right_foot.joint.f.n();
          m_is_first_iter = false;
        }

        return s;
      }


      /// ************************************************************************* ///
      /// The following signals depend only on other inner signals, so they
      /// just need to copy the interested part of the inner signal they depend on.
      /// ************************************************************************* ///

      DEFINE_SIGNAL_OUT_FUNCTION(baseAcceleration, ml::Vector)
      {
        m_torques_wrenchesSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        // add back gravity acceleration which was previously subtracted
        m_ddq(2) += 9.81; // assume the world z axis points upwards
        for(int i=0; i<6; i++)
          s(i) = m_ddq(i);
        m_ddq(2) -= 9.81;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(baseAngularVelocity, ml::Vector)
      {
        m_torques_wrenchesSINNER(iter);
        if(s.size()!=3)
          s.resize(3);
        for(int i=0; i<3; i++)
          s(i) = m_dq(3+i);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(jointsPositions, ml::Vector)
      {
        sotDEBUG(15)<<"Compute jointsPositions output signal "<<iter<<endl;

        const ml::Vector &q_dq_ddq = m_q_dq_ddqSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = q_dq_ddq(i);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(currentFiltered, ml::Vector) 
      {
        sotDEBUG(15)<<"Compute currentFiltered output signal "<<iter<<endl;

        // copy current measurements from mal to std vectors
        COPY_VECTOR_TO_ARRAY(m_currentMeasureSIN(iter)   ,  m_currentMeasure_std   );
        // filter current measurements
        m_currentMeasureFilter->estimate(m_currentMeasure_filter_std, m_currentMeasure_std);
        // map filtered current measurements from std to Eigen vectors
        EIGEN_VECTOR_FROM_STD_VECTOR(currentMeasure_eig, m_currentMeasure_filter_std);

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = m_currentMeasure_filter_std[i];
        return s;
      }




      DEFINE_SIGNAL_OUT_FUNCTION(jointsVelocities, ml::Vector)
      {
//        sotDEBUG(15)<<"Compute jointsVelocities output signal "<<iter<<endl;

        const ml::Vector &q_dq_ddq = m_q_dq_ddqSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = q_dq_ddq(i+N_JOINTS);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(jointsAccelerations, ml::Vector)
      {
        sotDEBUG(15)<<"Compute jointsAccelerations output signal "<<iter<<endl;

        const ml::Vector &q_dq_ddq = m_q_dq_ddqSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = q_dq_ddq(i+2*N_JOINTS);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(torsoAngularVelocity, ml::Vector)
      {
//        SEND_MSG("Compute torsoAngularVelocity output signal "+toString(iter), MSG_TYPE_DEBUG);
        const ml::Vector &w_dw_base = m_w_dv_torsoSINNER(iter);
        if(s.size()!=3)
          s.resize(3);
        for(int i=0; i<3; i++)
          s(i) = w_dw_base(i);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(torsoAcceleration, ml::Vector)
      {
//        SEND_MSG("Compute torsoAcceleration output signal "+toString(iter), MSG_TYPE_DEBUG);
        const ml::Vector &w_dw_base = m_w_dv_torsoSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<6; i++)
          s(i) = w_dw_base(i+3);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchLeftFoot, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchLeftFoot output signal "<<iter<<endl;

        const ml::Vector &torques_wrenches = m_torques_wrenchesSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<6; i++)
          s(i) = torques_wrenches(i+N_JOINTS+6*m_INDEX_WRENCH_LEFT_FOOT);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchRightFoot, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchRighttFoot output signal "<<iter<<endl;

        const ml::Vector &torques_wrenches = m_torques_wrenchesSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<6; i++)
          s(i) = torques_wrenches(i+N_JOINTS+6*m_INDEX_WRENCH_RIGHT_FOOT);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchLeftSole, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchLeftSole output signal "<<iter<<endl;

        METAPOD_FORCE_FROM_SIGNAL(f_LF, m_contactWrenchLeftFootSOUT(iter));
        f_LF = m_sole_X_LF.apply(f_LF);

        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<3; i++)
        {
          s(i)   = f_LF.f()(i);
          s(i+3) = f_LF.n()(i);
        }
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchRightSole, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchRightSole output signal "<<iter<<endl;

        METAPOD_FORCE_FROM_SIGNAL(f_RF, m_contactWrenchRightFootSOUT(iter));
        f_RF = m_sole_X_RF.apply(f_RF);

        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<3; i++)
        {
          s(i)   = f_RF.f()(i);
          s(i+3) = f_RF.n()(i);
        }
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchRightHand, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchRightHand output signal "<<iter<<endl;

        const ml::Vector &torques_wrenches = m_torques_wrenchesSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<6; i++)
          s(i) = torques_wrenches(i+N_JOINTS+6*m_INDEX_WRENCH_RIGHT_HAND);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchLeftHand, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchLeftHand output signal "<<iter<<endl;

        const ml::Vector &torques_wrenches = m_torques_wrenchesSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<6; i++)
          s(i) = torques_wrenches(i+N_JOINTS+6*m_INDEX_WRENCH_LEFT_HAND);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(contactWrenchBody, ml::Vector)
      {
        sotDEBUG(15)<<"Compute contactWrenchBody output signal "<<iter<<endl;

        const ml::Vector &torques_wrenches = m_torques_wrenchesSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
        for(int i=0; i<6; i++)
          s(i) = torques_wrenches(i+N_JOINTS+6*m_INDEX_WRENCH_BODY);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(jointsTorquesFromInertiaModel, ml::Vector)
      {
        sotDEBUG(15)<<"Compute jointsTorquesFromInertiaModel output signal "<<iter<<endl;
//        bool useVelocity = velocitySIN;
        const ml::Vector &torques_wrenches = m_torques_wrenchesSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = torques_wrenches(i);
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(jointsTorquesFromMotorModel, ml::Vector)
      {
          sotDEBUG(15)<<"Compute jointsTorquesFromMotorModel output signal "<<iter<<endl;
          const ml::Vector &torques = m_torquesFromMotorModelSINNER(iter);
          if(s.size()!=N_JOINTS)
            s.resize(N_JOINTS);
          for(int i=0; i<N_JOINTS; i++)
            s(i) = torques(i);

        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(jointsTorques, ml::Vector)
      {
        getProfiler().start(PROFILE_JOINTS_TORQUES_COMPUTATION);
        {
          sotDEBUG(15)<<"Compute jointsTorques output signal "<<iter<<endl;
          // copy current saturation and mixing weigth from mal to std vectors
          COPY_VECTOR_TO_ARRAY(m_saturationCurrentSIN(iter),  m_saturationCurrent_std);
          COPY_VECTOR_TO_ARRAY(m_wCurrentTrustSIN(iter)    ,  m_wCurrentTrust_std    );
          
          const ml::Vector &torques_fromMotor   = m_torquesFromMotorModelSINNER(iter);
          const ml::Vector &torques_fromInertia = m_torques_wrenchesSINNER(iter);

          if(s.size()!=N_JOINTS) s.resize(N_JOINTS);
          for(int i=0; i<N_JOINTS; i++)
          {
            if (( m_currentMeasure_std[i] < +m_saturationCurrent_std[i] ) && 
                ( m_currentMeasure_std[i] > -m_saturationCurrent_std[i] ) && m_wCurrentTrust_std[i]!=0) //Would we check it on the filtered data? With an eps?
            {
                s(i) =  torques_fromMotor(i)  *   m_wCurrentTrust_std[i]
                       +torques_fromInertia(i)*(1-m_wCurrentTrust_std[i]);
            }
            else
                s(i) =  torques_fromInertia(i);
          }
          m_tauBuffer.push_back(s);
        }
        getProfiler().stop(PROFILE_JOINTS_TORQUES_COMPUTATION);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dynamicsError, ml::Vector)
      {
        sotDEBUG(15)<<"Compute dynamicsError output signal "<<iter<<endl;
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);

        if(!m_tauDesSIN.isPlugged() || m_tauDesSIN.getTime()<=1)
          return s;

        const ml::Vector &contactWrenchBody = m_contactWrenchBodySOUT(iter);
        const ml::Vector &tauDesLast = m_tauDesSIN(iter-1);
        m_jointsTorquesSOUT(iter);

        // take the value in the buffer corresponding to the desired delay
        m_tauDesBuffer.push_back(tauDesLast);
        int index = m_tauDesBuffer.size() - 1 - (int)(m_delayTauDes/m_dt);
        if(index<0)
          index = 0;
        ml::Vector tauDes = m_tauDesBuffer[index];

        // take the second most recent value (i.e. the one computed at the last cycle)
        ml::Vector tau;
        if(m_tauBuffer.size()==1)
          tau = m_tauBuffer[0];
        else
          tau = m_tauBuffer[m_tauBuffer.size()-2];

        for(int i=0; i<6; i++)
          s(i) = -1.0*contactWrenchBody(i);
        for(int i=0; i<N_JOINTS; i++)
          s(6+i) = tauDes(i) - tau(i);
        return s;
      }

      void ForceTorqueEstimator::display( std::ostream& os ) const
      {
        os << "ForceTorqueEstimator "<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
