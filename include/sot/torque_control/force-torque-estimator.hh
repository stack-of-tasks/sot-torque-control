/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_ForceTorqueEstimator_H__
#define __sot_torque_control_ForceTorqueEstimator_H__
/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (force_torque_estimator_EXPORTS)
#    define SOTFORCETORQUEESTIMATOR_EXPORT __declspec(dllexport)
#  else
#    define SOTFORCETORQUEESTIMATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFORCETORQUEESTIMATOR_EXPORT
#endif

//#define VP_DEBUG 1        /// enable debug output
//#define VP_DEBUG_MODE 20

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/* HELPER */
#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/metapod-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/hrp2-common.hh>
#include <boost/circular_buffer.hpp>

/* Polynomial estimators */
#include <sot/torque_control/utils/lin-estimator.hh>
#include <sot/torque_control/utils/quad-estimator.hh>

/* Metapod */
#include <metapod/models/hrp2_14/hrp2_14.hh>
#include <metapod/algos/rnea.hh>
#include <metapod/tools/bcalc.hh>
#include <metapod/tools/print.hh>
#include <metapod/tools/initconf.hh>

/*Motor model*/
#include <sot/torque_control/motor-model.hh>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {




      /**
        * This Entity takes as inputs the joints' encoders,
        * the IMU and the force/torque sensors' measurements and it computes
        * as output estimates of the joints' torques, the contact
        * forces, the joints' positions, velocities and accelerations.
        * It also takes current measurment if any available, to compute 
        * a second estimate of the joints' torques from the motor model.
        * The two torques estimations can be mixed with signal wCurrentTrust
        * (value from 0 to 1)
        * As current measurment can saturate to a lower value than the 
        * real motor current, an input signal saturationCurrent must be provide.
        * If a current sensor is not available for some joint, this value 
        * should be set to zero.
        * 
        * QUICK START
        * Create the entity, plug all the input signals, call the init method
        * specifying the control-loop time step and the desired delay introduced
        * by the estimation (at least 1.5 times the time step). For instance:
        *   estimator = ForceTorqueEstimator("estimator");
        *   plug(device.robotState,     estimator.base6d_encoders);
        *   plug(device.accelerometer,  estimator.accelerometer);
        *   plug(device.gyrometer,      estimator.gyroscope);
        *   plug(device.forceRLEG,      estimator.ftSensRightFoot);
        *   plug(device.forceLLEG,      estimator.ftSensLeftFoot);
        *   plug(device.forceRARM,      estimator.ftSensRightHand);
        *   plug(device.forceLARM,      estimator.ftSensLeftHand);
        *   estimator.init(dt, estimationDelay);
        * Note that the input signals must be plugged before calling init, otherwise
        * the estimation cannot compute the force/torque sensors offsets and
        * the initialization will fail.
        *
        * After this you can read the estimates on the output signals:
        * - jointsPositions: in rad (30d)
        * - jointsVelocities: in rad/s (30d)
        * - jointsAccelerations: in rad/s^2 (30d)
        * - torsoAcceleration: linear + angular in torso's reference frame (6d)
        * - torsoAngularVelocity: in the torso's reference frame (3d)
        * - contactWrenchLeftFoot: equivalent wrench at the frame of the left foot in local coordinate (6d)
        * - contactWrenchRighttFoot: equivalent wrench at the frame of the right foot in local coordinate (6d)
        * - contactWrenchLeftSole: equivalent wrench at the sole of the left foot in local coordinate (6d)
        * - contactWrenchRighttSole: equivalent wrench at the sole of the right foot in local coordinate (6d)
        * - contactWrenchLeftHand: equivalent wrench at the frame of the left hand in local coordinate (6d)
        * - contactWrenchRightHand: equivalent wrench at the frame of the right hand in local coordinate (6d)
        * - contactWrenchBody: equivalent wrench at the base in local coordinate (6d)
        * - jointsTorques: in N*m (30d)
        *
        * DETAILS
        * Joints' velocities and accelerations are computed by fitting
        * a 2-nd order polynomial to a fixed-length window of the joints'
        * positions (as measured by the encoders). The position and velocity
        * are taken in the middle of the window (the acceleration is constant
        * over the whole window since the polynomial is only 2-nd order),
        * so that the delay is half of the window's length.
        *
        * The base's acceleration (lin+ang) and angular velocity are
        * estimated from the IMU's measurements (i.e. linear acceleration
        * and angular velocity). We use again a polynomial fitting over
        * a fixed-length window, but this time the polynomial is of 1-st
        * order because we only need to derivate once. The window's length
        * is the same used for the joints so that the estimation's delay
        * is consistent for all the quantities. Note that we do not need
        * to estimate the position, orientation and linear velocity of the
        * base because they do not affect the dynamics of the robot. Actually
        * the base's orientation indirectly affects the dynamics because it
        * is related to the gravity fields, but since the linear acceleration
        * measured by the IMU includes the gravity accelerations we do not
        * need to know the base's orientation to estimate the joints' torques
        * and the contact forces.
        *
        * The contact forces (actually they are 6d wrench, but in the following
        * we keep using force as a synonim of wrench) are always assumed to
        * be at the four end-effectors (hands and feet) and on the base of the
        * robot. Given that the robot is equipped with 4 F/T sensors, only 5
        * contact forces are observable. The assumption on the
        * position of 4 contact forces at the end-effectors is dictated by
        * the location of the F/T sensors. On the contrary, the 5-th contact
        * force could be assumed anywhere on the robot's body; currently
        * we assume it to be on the base, so that we do not have to modify
        * the order in which forces are propagated in the RNEA.
        * Given their locations, contact forces can be estimated by running
        * the Recursive Newton-Euler Algorithm (RNEA). We first propagates
        * pos, vel and acc from the robot's base to the end-effectors. Then
        * we propagate the measurements of the F/T sensors towards the locations
        * of the contacts. This procedure estimates at the same time the contact
        * forces and the joint torques.
        */
      class SOTFORCETORQUEESTIMATOR_EXPORT ForceTorqueEstimator
          :public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:  /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(base6d_encoders,  ml::Vector);
        DECLARE_SIGNAL_IN(accelerometer,    ml::Vector);
        DECLARE_SIGNAL_IN(gyroscope,        ml::Vector);
        DECLARE_SIGNAL_IN(ftSensLeftFoot,   ml::Vector);
        DECLARE_SIGNAL_IN(ftSensRightFoot,  ml::Vector);
        DECLARE_SIGNAL_IN(ftSensLeftHand,   ml::Vector);
        DECLARE_SIGNAL_IN(ftSensRightHand,  ml::Vector);
        DECLARE_SIGNAL_IN(ddqRef,           ml::Vector);
        DECLARE_SIGNAL_IN(dqRef,            ml::Vector);
        DECLARE_SIGNAL_IN(currentMeasure,   ml::Vector);
        DECLARE_SIGNAL_IN(saturationCurrent,ml::Vector);
        DECLARE_SIGNAL_IN(wCurrentTrust,    ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKt_p, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKt_n, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKf_p, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKf_n, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKv_p, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKv_n, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKa_p, ml::Vector);
        DECLARE_SIGNAL_IN(motorParameterKa_n, ml::Vector);
        DECLARE_SIGNAL_IN(tauDes, ml::Vector);  // desired joint torques
        
        DECLARE_SIGNAL_OUT(ftSensRightFootPrediction,  ml::Vector); /// debug signal

        DECLARE_SIGNAL_OUT(jointsPositions,         ml::Vector);
        DECLARE_SIGNAL_OUT(jointsVelocities,        ml::Vector);
        DECLARE_SIGNAL_OUT(jointsAccelerations,     ml::Vector);
        DECLARE_SIGNAL_OUT(torsoAcceleration,       ml::Vector);  // 6d
        DECLARE_SIGNAL_OUT(torsoAngularVelocity,    ml::Vector);  // 3d
        DECLARE_SIGNAL_OUT(baseAcceleration,        ml::Vector);  // 6d
        DECLARE_SIGNAL_OUT(baseAngularVelocity,     ml::Vector);  // 3d
        DECLARE_SIGNAL_OUT(contactWrenchLeftFoot,   ml::Vector);
        DECLARE_SIGNAL_OUT(contactWrenchRightFoot,  ml::Vector);
        DECLARE_SIGNAL_OUT(contactWrenchLeftSole,   ml::Vector);
        DECLARE_SIGNAL_OUT(contactWrenchRightSole,  ml::Vector);
        DECLARE_SIGNAL_OUT(contactWrenchLeftHand,   ml::Vector);
        DECLARE_SIGNAL_OUT(contactWrenchRightHand,  ml::Vector);
        DECLARE_SIGNAL_OUT(contactWrenchBody,       ml::Vector);
        DECLARE_SIGNAL_OUT(currentFiltered,         ml::Vector);
        DECLARE_SIGNAL_OUT(jointsTorques,           ml::Vector);
        DECLARE_SIGNAL_OUT(jointsTorquesFromMotorModel,    ml::Vector);
        DECLARE_SIGNAL_OUT(jointsTorquesFromInertiaModel,  ml::Vector);
        DECLARE_SIGNAL_OUT(dynamicsError,  ml::Vector); // error between desired torques and estimated (n+6)

        /// The following inner signals are used because this entity has some output signals
        /// whose related quantities are computed at the same time by the same algorithm
        /// (e.g. torques and contact wrenches are computed by the RNEA). to avoid the risk
        /// or recomputing the same things twice, we create an inner signal that groups together
        /// all the quantities that are computed together. Then the single output signals will depend
        /// on this inner signal, which is the one triggering the computations.
        /// Inner signals are not exposed, so that nobody can access them.

        /// This signal contains the joints' torques and all the 5 contact wrenches
        DECLARE_SIGNAL_INNER(torques_wrenches,      ml::Vector);
        /// This signal contains the joints' torques estimated from motor model and current measurment
        DECLARE_SIGNAL_INNER(torquesFromMotorModel, ml::Vector);
        /// This signal contains the estimated joints positions, velocities and accelerations.
        DECLARE_SIGNAL_INNER(q_dq_ddq,              ml::Vector);
        /// This signal contains the estimated base angular velocity and lin/ang accelerations.
        DECLARE_SIGNAL_INNER(w_dv_torso,            ml::Vector);
        
      protected:
      
        MotorModel motorModel;
      
        /// index ordering the 5 estimated contact wrenches inside the
        /// inner signal torques_wrenches
        static const int m_INDEX_WRENCH_LEFT_HAND   = 0;
        static const int m_INDEX_WRENCH_RIGHT_HAND  = 1;
        static const int m_INDEX_WRENCH_LEFT_FOOT   = 2;
        static const int m_INDEX_WRENCH_RIGHT_FOOT  = 3;
        static const int m_INDEX_WRENCH_BODY        = 4;

        /// number of samples collected to compute the offset of the force/torque sensors
        static const int N_SAMPLE_FT_SENS_OFFSET = 100;

        double m_dt;              /// timestep of the controller
        double m_delayEncoders;   /// delay introduced by the estimation of joints pos/vel/acc
        double m_delayFTsens;     /// delay introduced by the filtering of the F/T sensors
        double m_delayAcc;        /// delay introduced by the filtering of the accelerometer
        double m_delayGyro;       /// delay introduced by the filtering of the gyroscope
        double m_delayCurrent;    /// delay introduced by the filtering of the motor current measure

        bool m_is_first_iter;     /// true at the first iteration, false after
        bool m_useRefJointsAcc;   /// if true it uses the reference (rather than estimated) joints acc to estimate torques and ext forces
        bool m_useRefJointsVel;   /// if true it uses the reference (rather than estimated) joints vel to estimate torques and ext forces
        bool m_useRawEncoders;    /// if true it uses the raw encoders data (rather than the filtered values) to estimate torques and ext forces
        bool m_useFTsensors;      /// if true it uses the F/T sensors to estimate the joints torques
        bool m_computeFTsensorOffsets;  /// if true at the first iteration it computes the F/T sensor offsets

        /// std::vector to use with the filters
        /// All the variables whose name contains 'filter' are outputs of the filters
        std::vector<double> m_ddq_filter_std;  /// joints accelerations
        std::vector<double> m_dq_filter_std;   /// joints velocities
        std::vector<double> m_q_filter_std;    /// joints positions
        std::vector<double> m_q_std;           /// joints positions
        std::vector<double> m_dv_IMU_std, m_dv_IMU_filter_std;/// IMU lin acceleration
        std::vector<double> m_w_IMU_std, m_w_IMU_filter_std;  /// IMU angular vel
        std::vector<double> m_dw_IMU_filter_std;              /// IMU angular acc
        std::vector<double> m_ftSens_LH_std, m_ftSens_LH_filter_std;  /// force/torque sensor left hand
        std::vector<double> m_ftSens_RH_std, m_ftSens_RH_filter_std;  /// force/torque sensor right hand
        std::vector<double> m_ftSens_LF_std, m_ftSens_LF_filter_std;  /// force/torque sensor left foot
        std::vector<double> m_ftSens_RF_std, m_ftSens_RF_filter_std;  /// force/torque sensor right foot
        std::vector<double> m_currentMeasure_std, m_currentMeasure_filter_std;  /// motor current measure
        std::vector<double> m_saturationCurrent_std;   /// motor current sensor saturation
        std::vector<double> m_wCurrentTrust_std;       /// torque estimation mixing weight
        std::vector<double> m_motorParameterKt_p_std;  /// motor parameter Kt when dq>0
        std::vector<double> m_motorParameterKt_n_std;  /// motor parameter Kt when dq<0
        std::vector<double> m_motorParameterKf_p_std;  /// motor parameter Kf when dq>0
        std::vector<double> m_motorParameterKf_n_std;  /// motor parameter Kf when dq<0
        std::vector<double> m_motorParameterKv_p_std;  /// motor parameter Kv when dq>0
        std::vector<double> m_motorParameterKv_n_std;  /// motor parameter Kv when dq<0
        std::vector<double> m_motorParameterKa_p_std;  /// motor parameter Ka when dq>0
        std::vector<double> m_motorParameterKa_n_std;  /// motor parameter Ka when dq<0

        /// spatial velocity and acceleration of the torso
        metapod::Spatial::MotionTpl<double> m_v_torso;
        metapod::Spatial::MotionTpl<double> m_dv_torso;

        /// polynomial-fitting filters
        PolyEstimator* m_encodersFilter;
        PolyEstimator* m_accelerometerFilter;
        PolyEstimator* m_gyroscopeFilter;
        PolyEstimator* m_ftSensLeftFootFilter;
        PolyEstimator* m_ftSensRightFootFilter;
        PolyEstimator* m_ftSensLeftHandFilter;
        PolyEstimator* m_ftSensRightHandFilter;
        PolyEstimator* m_currentMeasureFilter;
        

        /// force/torque sensors' offsets taken at the initialization
        Eigen::VectorXd m_ftSensLeftHand_offset;
        Eigen::VectorXd m_ftSensRightHand_offset;
        Eigen::VectorXd m_ftSensLeftFoot_offset;
        Eigen::VectorXd m_ftSensRightFoot_offset;
        maal::boost::Vector m_FTsensorOffsets;

        // *************************** Metapod ******************************
        typedef metapod::hrp2_14<double> Hrp2_14;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::r_ankle>::type      RightFootNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::l_ankle>::type      LeftFootNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::r_wrist>::type      RightHandNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::l_wrist>::type      LeftHandNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::torso>::type        TorsoNode;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::CHEST_LINK0>::type  ChestLink0Node;
        typedef metapod::Nodes<Hrp2_14, Hrp2_14::BODY>::type         BodyNode;
        typedef metapod::Spatial::TransformT<double, metapod::Spatial::RotationMatrixIdentityTpl<double> > TransformNoRot;
        typedef metapod::Spatial::TransformT<double, metapod::Spatial::RotationMatrixAboutZTpl<double> > TransformRotZ;


        /// nodes corresponding to the four end-effectors
        RightFootNode&  m_node_right_foot;
        LeftFootNode&   m_node_left_foot;
        RightHandNode&  m_node_right_hand;
        LeftHandNode&   m_node_left_hand;
        TorsoNode&      m_node_torso;
        ChestLink0Node& m_node_chest_link0;
        BodyNode&       m_node_body;

        /// Transformation from force/torque sensors frame to hosting link frame
        TransformNoRot m_RF_X_ftSens;
        TransformNoRot m_LF_X_ftSens;
        TransformRotZ m_RH_X_ftSens;
        TransformRotZ m_LH_X_ftSens;
        /// Transformation from foot frame to sole frame
        TransformNoRot m_sole_X_RF;
        TransformNoRot m_sole_X_LF;
        /// Transformation from IMU's frame to torso's frame
        TransformNoRot m_torso_X_imu;

        /// robot geometric/inertial data
        Hrp2_14 m_robot;
        Hrp2_14::confVector m_q, m_dq, m_ddq;
        Hrp2_14::confVector m_torques;

        boost::circular_buffer<ml::Vector> m_tauDesBuffer;
        boost::circular_buffer<ml::Vector> m_tauBuffer;
        double m_delayTauDes;
        
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** --- CONSTRUCTOR ---- */
        ForceTorqueEstimator( const std::string & name );

        /** Initialize the ForceTorqueEstimator.
         * @param timestep Period (in seconds) after which the sensors' data are updated.
         * @param delayEncoders Delay (in seconds) introduced by the estimation of joints pos/vel/acc.
         *                        This should be a multiple of timestep.
         * @param delayFTsens Delay (in seconds) introduced by the low-pass filtering of the F/T sensors.
         *                        This should be a multiple of timestep.
         * @param delayAcc Delay (in seconds) introduced by the low-pass filtering of the accelerometer.
         *                        This should be a multiple of timestep.
         * @param delayGyro Delay (in seconds) introduced by the low-pass filtering of the gyroscope.
         *                        This should be a multiple of timestep.
         * @param delayCurrent Delay (in seconds) introduced by the low-pass filtering of the current sensors.
         *                        This should be a multiple of timestep.
         * @param computeForceSensorsOffsets If true it computes the offset on the F/T sensors at the beginning.
         * @note The estimationDelay is half of the length of the window used for the
         * polynomial fitting. The larger the delay, the smoother the estimations.
         */
        void init(const double &timestep, const double &delayEncoders,
                  const double& delayFTsens, const double& delayAcc,
                  const double& delayGyro,const double& delayCurrent,
                  const bool &computeForceSensorsOffsets);

        void setFTsensorOffsets(const ml::Vector& offsets);
        void recomputeFTsensorOffsets();
        
      protected:
        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("["+name+"] "+msg, t, file, line);
        }

      public: /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

      }; // class ForceTorqueEstimator

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_ForceTorqueEstimator_H__
