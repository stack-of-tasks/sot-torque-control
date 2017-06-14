/*
 * Copyright 2014-2017, Andrea Del Prete, Rohan Budhiraja LAAS-CNRS
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

#include <sot/torque_control/torque-offset-estimator.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/motor-model.hh>
#include <sot/torque_control/hrp2-common.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

#define ALL_INPUT_SIGNALS m_base6d_encodersSIN << m_accelerometerSIN << m_gyroscopeSIN \
                          << m_jointTorquesSIN

#define ALL_OUTPUT_SIGNALS m_jointTorquesEstimatedSOUT

      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef TorqueOffsetEstimator EntityClassName;
      typedef int dummy;
      std::vector<Eigen::VectorXd,Eigen::aligned_allocator<Eigen::VectorXd> > stdAlignedVector;
      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TorqueOffsetEstimator,"TorqueOffsetEstimator");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      TorqueOffsetEstimator::
      TorqueOffsetEstimator( const std::string & name )
        : Entity(name),
          CONSTRUCT_SIGNAL_IN(base6d_encoders,      dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(accelerometer,         dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(gyroscope,             dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointTorques,          dynamicgraph::Vector)
        ,CONSTRUCT_SIGNAL_INNER(collectSensorData,  dynamicgraph::Vector, ALL_INPUT_SIGNALS)
        ,CONSTRUCT_SIGNAL_OUT(jointTorquesEstimated,
                              dynamicgraph::Vector, m_collectSensorDataSINNER)
        ,sensor_offset_status(PRECOMPUTATION)
      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);
        addCommand("init", makeCommandVoid2(*this, &TorqueOffsetEstimator::init,
                                            docCommandVoid2("Initialize the estimator" ,
                                                            "urdfFilePath",
                                                            "Definition of the imu in the chest frame")));
        addCommand("computeOffset",
                   makeCommandVoid2(*this, &TorqueOffsetEstimator::computeOffset,
                                    docCommandVoid2("Compute the offset for sensor calibration",
                                                    "Number of iteration to average over.",
                                                    "Maximum allowed offset")));
        addCommand("getSensorOffsets",
                   makeDirectGetter(*this,&jointTorqueOffsets,
                                    docDirectGetter("Return the computed sensor offsets",
                                                    "vector")));

        // encSignals.clear();
        // accSignals.clear();
        // gyrSignals.clear();
        // tauSignals.clear();
        stdVecJointTorqueOffsets.clear();
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void TorqueOffsetEstimator::init(const std::string &urdfFile,
                                       const Eigen::Matrix4d& _m_torso_X_imu)
      {
        try {
          se3::urdf::buildModel(urdfFile,se3::JointModelFreeFlyer(),m_model);
          assert(m_model.nq == N_JOINTS+7);
          assert(m_model.nv == N_JOINTS+6);

          jointTorqueOffsets.resize(N_JOINTS);
          jointTorqueOffsets.setZero();

          ffIndex = 0;
          torsoIndex = m_model.getJointId("torso_2_joint");
        }
        catch (const std::exception& e) 
        { 
          std::cout << e.what();
          return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
        }
        m_data = new se3::Data(m_model);
        m_torso_X_imu.rotation() = _m_torso_X_imu.block<3,3>(0,0);
        m_torso_X_imu.translation() = _m_torso_X_imu.block<3,1>(0,3);
      }

      void TorqueOffsetEstimator::computeOffset(const int& nIterations, const double& _epsilon)
      {
        if (sensor_offset_status == PRECOMPUTATION) {
          SEND_MSG("Starting offset computation with no. iterations:"+ nIterations, MSG_TYPE_DEBUG);
          n_iterations = nIterations;
          epsilon = _epsilon;
          sensor_offset_status = INPROGRESS;
        }
        else if (sensor_offset_status == INPROGRESS) {
          SEND_MSG("Collecting input signals. Please keep the graph running", MSG_TYPE_WARNING_STREAM);
        }
        else if (sensor_offset_status == COMPUTED) {
          SEND_MSG("Recomputing offset with no. iterations:"+ nIterations, MSG_TYPE_DEBUG);

          stdVecJointTorqueOffsets.clear();
          jointTorqueOffsets.setZero();

          n_iterations = nIterations;
          epsilon = _epsilon;
          sensor_offset_status = INPROGRESS;
        }
        else { //should not happend
          SEND_MSG("Problem in computeOffset sensor_offset_status", MSG_TYPE_ERROR);
          assert(false && "Problem in computeOffset sensor_offset_status");
        }
        return;
      }
      
      DEFINE_SIGNAL_INNER_FUNCTION(collectSensorData, dummy)
      {
        if (sensor_offset_status == PRECOMPUTATION)  {
          SEND_MSG("Currently Precomputation. Not collecting signals", MSG_TYPE_DEBUG);
        }
        else if (sensor_offset_status == INPROGRESS) {
          // Check the current iteration status
          int _i = stdVecJointTorqueOffsets.size();
          
          if (_i < n_iterations) {
            SEND_MSG("Collecting signals for iteration no:" + _i , MSG_TYPE_DEBUG);

            const Eigen::VectorXd& sot_enc = m_base6d_encodersSIN(iter);
            const Eigen::VectorXd& IMU_acc = m_accelerometerSIN(iter);
            const Eigen::VectorXd& IMU_gyr = m_gyroscopeSIN(iter);
            const Eigen::VectorXd& _tau = m_jointTorquesSIN(iter);

            Eigen::VectorXd _enc(N_JOINTS+7);
            base_sot_to_urdf(sot_enc.head<6>(), _enc.head<7>());
            _enc.tail<N_JOINTS>() = sot_enc.tail<N_JOINTS>();

            //Get the transformation from ff(f) to torso (t) to IMU(i) frame:
            // fMi = oMf^-1 * fMt * tMi
            se3::forwardKinematics(m_model,*m_data,_enc);
            se3::SE3 fMi = m_data->oMi[ffIndex].inverse()*m_data->oMi[torsoIndex]*m_torso_X_imu;

            //Move the IMU signal to the base frame.
            //angularAcceleration is zero. Intermediate frame acc and velocities are zero
            const dynamicgraph::Vector _acc = fMi.rotation()*IMU_acc; //Torso Acceleration

            //Deal with gravity predefined in robot model. Robot Z should be pointing upwards
            m_model.gravity.linear() = _acc;

            const Eigen::VectorXd& _tau_rnea = se3::rnea(m_model, *m_data, _enc,
                                                         Eigen::VectorXd::Zero(N_JOINTS),
                                                         Eigen::VectorXd::Zero(N_JOINTS));
            const Eigen::VectorXd _current_offset = _tau - _tau_rnea.tail<N_JOINTS>();
            if(_current_offset.array().abs().maxCoeff() >=epsilon) {
              SEND_MSG("Too high torque offset estimated for iteration"+ _i, MSG_TYPE_ERROR);
              assert(false);
            }
            // encSignals.push_back(_enc);
            // accSignals.push_back(_acc);
            // gyrSignals.push_back(_gyr);
            // tauSignals.push_back(_tau);
            stdVecJointTorqueOffsets.push_back(_current_offset);

          }
          else if (_i == n_iterations){
            // Find the mean, enough signals collected
            calculateSensorOffsets();
            sensor_offset_status = COMPUTED;
          }
          else if (_i > n_iterations) {
            SEND_MSG("This should not happen", MSG_TYPE_ERROR);
            assert(false);
          }
          else {
            SEND_MSG("This should not happen", MSG_TYPE_ERROR);
            assert(false);
          }
        }
        else if (sensor_offset_status == COMPUTED) {
          SEND_MSG("Offset computed", MSG_TYPE_DEBUG);
        } 
        else {
          SEND_MSG("This should not happen", MSG_TYPE_ERROR);
          assert(false);
        }
        return s; 
      }

      DEFINE_SIGNAL_OUT_FUNCTION(jointTorquesEstimated, dynamicgraph::Vector)
      {
        m_collectSensorDataSINNER(iter);
        if (s.size() != N_JOINTS) s.resize(N_JOINTS);

        if (sensor_offset_status == PRECOMPUTATION || sensor_offset_status == INPROGRESS) {
          s = m_jointTorquesSIN(iter);
        }
        else { //sensor_offset_status == COMPUTED
          const dynamicgraph::Vector& inputTorques = m_jointTorquesSIN(iter);
          s = inputTorques - jointTorqueOffsets;
        }
          return s;
      }

      void TorqueOffsetEstimator::calculateSensorOffsets() {
        // assert(encSignals.size() == n_iterations);
        // assert(accSignals.size() == n_iterations);
        // assert(gyrSignals.size() == n_iterations);
        // assert(tauSignals.size() == n_iterations);
        assert(stdVecJointTorqueOffsets.size() == n_iterations);

        jointTorqueOffsets.setZero();
        for (int _i=0; _i<n_iterations; _i++){
          const Eigen::VectorXd& current_offset = stdVecJointTorqueOffsets.at(_i);
          jointTorqueOffsets +=current_offset;
        }
        jointTorqueOffsets /=n_iterations;
        return;
      }

      void TorqueOffsetEstimator::display( std::ostream& os ) const
      {
        os << "TorqueOffsetEstimator"<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
