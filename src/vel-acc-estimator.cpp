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

#include <sot/torque_control/vel-acc-estimator.hh>
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

#define ALL_INPUT_SIGNALS m_base6d_encodersSIN << m_accelerometerSIN << m_gyroscopeSIN

#define ALL_OUTPUT_SIGNALS \
                          m_jointsPositionsSOUT << m_jointsVelocitiesSOUT \
                           << m_jointsAccelerationsSOUT << m_torsoAccelerationSOUT \
                           << m_torsoAngularVelocitySOUT

      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace metapod;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef VelAccEstimator EntityClassName;

      typedef Eigen::Matrix<double,N_JOINTS,1>                     VectorN;
      typedef Eigen::Matrix<double,N_JOINTS+6,1>                   VectorN6;
      typedef Eigen::Matrix<double,3,1>                            Vector3;
      typedef Eigen::Matrix<double,6,1>                            Vector6;
      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VelAccEstimator,"VelAccEstimator");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      VelAccEstimator::
      VelAccEstimator( const std::string & name )
        : Entity(name),
          CONSTRUCT_SIGNAL_IN(base6d_encoders,       dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(accelerometer,          dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(gyroscope,              dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_OUT(jointsPositions,       dynamicgraph::Vector, m_q_dq_ddqSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsVelocities,      dynamicgraph::Vector, m_q_dq_ddqSINNER)
        ,CONSTRUCT_SIGNAL_OUT(jointsAccelerations,   dynamicgraph::Vector, m_q_dq_ddqSINNER)
        ,CONSTRUCT_SIGNAL_OUT(torsoAcceleration,     dynamicgraph::Vector, m_w_dv_torsoSINNER) // 6d
        ,CONSTRUCT_SIGNAL_OUT(torsoAngularVelocity,  dynamicgraph::Vector, m_w_dv_torsoSINNER) // 3d
        ,CONSTRUCT_SIGNAL_INNER(q_dq_ddq,            dynamicgraph::Vector, m_base6d_encodersSIN)
        ,CONSTRUCT_SIGNAL_INNER(w_dv_torso,          dynamicgraph::Vector, m_accelerometerSIN<<m_gyroscopeSIN)
      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);
        
        /* Commands. */
        addCommand("getTimestep",
                   makeDirectGetter(*this,&m_dt,
                                    docDirectGetter("Control timestep [s ]","double")));
        addCommand("getDelayEnc",
                   makeDirectGetter(*this,&m_delayEncoders,
                                    docDirectGetter("Delay introduced by the estimation of joints pos/vel/acc [s]","double")));
        addCommand("getDelayAcc",
                   makeDirectGetter(*this,&m_delayAcc,
                                    docDirectGetter("Delay introduced by the filtering of the accelerometer [s]","double")));
        addCommand("getDelayGyro",
                   makeDirectGetter(*this,&m_delayGyro,
                                    docDirectGetter("Delay introduced by the filtering of the gyroscope [s]","double")));
        addCommand("init", makeCommandVoid4(*this, &VelAccEstimator::init,
                              docCommandVoid4("Initialize the estimator.",
                                              "Control timestep [s].",
                                              "Estimation delay for joints pos/vel/acc [s].",
                                              "Estimation delay for accelerometer [s].",
                                              "Estimation delay for gyroscope [s].")));
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void VelAccEstimator::init(const double &timestep, const double& delayEncoders,
                                 const double& delayAcc, const double& delayGyro)
      {
        assert(timestep>0.0 && "Timestep should be > 0");
        assert(delayEncoders>=1.5*timestep && "Estimation delay for encoders should be >= 1.5*timestep");
        assert(delayAcc>=timestep && "Estimation delay for accelerometer should be >= timestep");
        assert(delayGyro>=timestep && "Estimation delay for gyroscope should be >= timestep");
        m_dt = timestep;
        m_delayEncoders = delayEncoders;
        m_delayAcc      = delayAcc;
        m_delayGyro     = delayGyro;
        int winSizeEnc     = (int)(2*delayEncoders/m_dt);
        int winSizeAcc     = (int)(2*delayAcc/m_dt);
        int winSizeGyro    = (int)(2*delayGyro/m_dt);
        assert(winSizeEnc>=3 && "Estimation-window's length for encoders should be >= 3");
        assert(winSizeAcc>=2 && "Estimation-window's length for accelerometer should be >= 2");
        assert(winSizeGyro>=2 && "Estimation-window's length for gyroscope should be >= 2");

        m_encodersFilter         = new QuadEstimator(winSizeEnc, N_JOINTS, m_dt);
        m_accelerometerFilter    = new LinEstimator(winSizeAcc, 3, m_dt);
        m_gyroscopeFilter        = new LinEstimator(winSizeGyro, 3, m_dt);

        m_ddq_filter_std.resize(N_JOINTS);
        m_dq_filter_std.resize(N_JOINTS);
        m_q_filter_std.resize(N_JOINTS);
        m_q_std.resize(N_JOINTS);
        m_dv_IMU_std.resize(3);
        m_dv_IMU_filter_std.resize(3);
        m_w_IMU_std.resize(3);
        m_w_IMU_filter_std.resize(3);
        m_dw_IMU_filter_std.resize(3);

        // since the base's position/orientation and linear velocity are not computed
        // we need to set q and dq to 0 during initialization

        m_v_torso.v(Vector3d::Zero());

        // compute transformation from IMU's frame to torso's frame
        typedef Map<const Vector3d> CMap3d;
        Spatial::RotationMatrixIdentityTpl<double> eye;
        m_torso_X_imu = TransformNoRot(eye, -CMap3d(IMU_XYZ));
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      /** Estimate the joints' positions, velocities and accelerations. */
      DEFINE_SIGNAL_INNER_FUNCTION(q_dq_ddq, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute q_dq_ddq inner signal "<<iter<<std::endl;

        // read encoders and copy in std vector
        const dynamicgraph::Vector& base_q = m_base6d_encodersSIN(iter);
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
      DEFINE_SIGNAL_INNER_FUNCTION(w_dv_torso, dynamicgraph::Vector)
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
	//TODO: MOVE METAPOD VECTORS TO PINOCCHIO
	s.segment<3>(0) = m_v_torso.w();
	s.segment<3>(3) = m_dv_torso.v();
	s.segment<3>(6) = m_dv_torso.w();
        return s;
      }

      /// ************************************************************************* ///
      /// The following signals depend only on other inner signals, so they
      /// just need to copy the interested part of the inner signal they depend on.
      /// ************************************************************************* ///


      DEFINE_SIGNAL_OUT_FUNCTION(jointsPositions, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute jointsPositions output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &q_dq_ddq = m_q_dq_ddqSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s.head<N_JOINTS>() = q_dq_ddq.head<N_JOINTS>();
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(jointsVelocities, dynamicgraph::Vector)
      {
//        sotDEBUG(15)<<"Compute jointsVelocities output signal "<<iter<<endl;

        const dynamicgraph::Vector &q_dq_ddq = m_q_dq_ddqSINNER(iter);
        if(s.size()!=N_JOINTS)
	  s.resize(N_JOINTS);
	s = q_dq_ddq.segment<N_JOINTS>(N_JOINTS);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(jointsAccelerations, dynamicgraph::Vector)
      {
        sotDEBUG(15)<<"Compute jointsAccelerations output signal "<<iter<<std::endl;

        const dynamicgraph::Vector &q_dq_ddq = m_q_dq_ddqSINNER(iter);
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s = q_dq_ddq.segment<N_JOINTS>(2*N_JOINTS);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(torsoAcceleration, dynamicgraph::Vector)
      {
        //SEND_MSG("Compute torsoAcceleration output signal "+toString(iter), MSG_TYPE_DEBUG);
        const dynamicgraph::Vector &w_dw_base = m_w_dv_torsoSINNER(iter);
        if(s.size()!=6)
          s.resize(6);
	s = w_dw_base.segment<6>(3);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(torsoAngularVelocity, dynamicgraph::Vector)
      {
        //SEND_MSG("Compute torsoAngularVelocity output signal "+toString(iter), MSG_TYPE_DEBUG);
        const dynamicgraph::Vector &w_dw_base = m_w_dv_torsoSINNER(iter);
        if(s.size()!=3)
          s.resize(3);
	s = w_dw_base.head<3>();
        return s;
      }


      void VelAccEstimator::display( std::ostream& os ) const
      {
        os << "VelAccEstimator "<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
