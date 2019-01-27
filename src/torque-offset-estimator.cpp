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
#include <sot/torque_control/common.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
namespace sot
{
namespace torque_control
{

#define ALL_INPUT_SIGNALS m_base6d_encodersSIN << m_accelerometerSIN    \
  << m_jointTorquesSIN << m_gyroscopeSIN
  
#define ALL_OUTPUT_SIGNALS m_jointTorquesEstimatedSOUT
  
  namespace dynamicgraph = ::dynamicgraph;
  using namespace dynamicgraph;
  using namespace dynamicgraph::command;
  using namespace Eigen;
  
  /// Define EntityClassName here rather than in the header file
  /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
  typedef TorqueOffsetEstimator EntityClassName;
  typedef int dummy;
  
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
                          dynamicgraph::Vector,
                          m_collectSensorDataSINNER << ALL_INPUT_SIGNALS)
    ,sensor_offset_status(PRECOMPUTATION)
    ,current_progress(0) 
  {
    Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);
    addCommand("init", makeCommandVoid5(*this, &TorqueOffsetEstimator::init,
                                        docCommandVoid5("Initialize the estimator" ,
                                                        "urdfFilePath",
                                                        "Homogeneous transformation from chest frame to IMU frame",
                                                        "Maximum angular velocity allowed in either axis",
                                                        "Freeflyer joint name",
                                                        "chest joint name")));
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
    // stdVecJointTorqueOffsets.clear();
  }


  /* --- COMMANDS ---------------------------------------------------------- */
  /* --- COMMANDS ---------------------------------------------------------- */
  /* --- COMMANDS ---------------------------------------------------------- */
  void TorqueOffsetEstimator::init(const std::string &robotRef,
                                   const Eigen::Matrix4d& m_torso_X_imu_,
                                   const double& gyro_epsilon_,
                                   const std::string& ffJointName,
                                   const std::string& torsoJointName) 
  {
    try 
    {
      /* Retrieve m_robot_util  informations */
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

      pinocchio::urdf::buildModel(m_robot_util->m_urdf_filename,
                            pinocchio::JointModelFreeFlyer(), m_model);
      // assert(m_model.nq == N_JOINTS+7);
      // assert(m_model.nv == N_JOINTS+6);
            
      jointTorqueOffsets.resize(m_model.nv-6);
      jointTorqueOffsets.setZero();
      ffIndex = m_model.getJointId(ffJointName);
      torsoIndex = m_model.getJointId(torsoJointName);
    }
    catch (const std::exception& e) 
    { 
      std::cout << e.what();
      SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
      return;
    }
    m_data = new pinocchio::Data(m_model);
    m_torso_X_imu.rotation() = m_torso_X_imu_.block<3,3>(0,0);
    m_torso_X_imu.translation() = m_torso_X_imu_.block<3,1>(0,3);
    gyro_epsilon = gyro_epsilon_;
  }

  void TorqueOffsetEstimator::computeOffset(const int& nIterations, const double& epsilon_) 
  {
    if (sensor_offset_status == PRECOMPUTATION) 
    {
      SEND_MSG("Starting offset computation with no. iterations:"+ nIterations, MSG_TYPE_DEBUG);
      n_iterations = nIterations;
      epsilon = epsilon_;
      sensor_offset_status = INPROGRESS;
    }
    else if (sensor_offset_status == INPROGRESS)  
    {
      SEND_MSG("Collecting input signals. Please keep the graph running", MSG_TYPE_WARNING);
    }
    else 
    { //sensor_offset_status == COMPUTED
      SEND_MSG("Recomputing offset with no. iterations:"+ nIterations, MSG_TYPE_DEBUG);
            
      // stdVecJointTorqueOffsets.clear();
      current_progress = 0;
      jointTorqueOffsets.setZero();
            
      n_iterations = nIterations;
      epsilon = epsilon_;
      sensor_offset_status = INPROGRESS;
    }
    return;
  }
      
  DEFINE_SIGNAL_INNER_FUNCTION(collectSensorData, dummy)
  {
    if (sensor_offset_status == INPROGRESS) 
    {

      const Eigen::VectorXd& gyro = m_gyroscopeSIN(iter);
      if(gyro.array().abs().maxCoeff() >=gyro_epsilon) 
      {
        SEND_MSG("Very High Angular Rotations.", MSG_TYPE_ERROR_STREAM);
      }
          
      // Check the current iteration status
      int i = current_progress;
          
      if (i < n_iterations) 
      {
        SEND_MSG("Collecting signals for iteration no:" + i , MSG_TYPE_DEBUG_STREAM);
              
        const Eigen::VectorXd& sot_enc = m_base6d_encodersSIN(iter);
        const Eigen::VectorXd& IMU_acc = m_accelerometerSIN(iter);
        const Eigen::VectorXd& tau = m_jointTorquesSIN(iter);

        Eigen::VectorXd enc(m_model.nq);
        //joints_sot_to_urdf(sot_enc.tail(m_model.nv-6), enc.tail(m_model.nv-6));
        enc.tail(m_model.nv-6) = sot_enc.tail(m_model.nv-6);
        //base_sot_to_urdf(sot_enc.head<6>(), enc.head<7>());
        enc.head<6>().setZero(); enc[6] = 1.0;


        //Get the transformation from ff(f) to torso (t) to IMU(i) frame:
        // fMi = oMf^-1 * fMt * tMi
        pinocchio::forwardKinematics(m_model,*m_data,enc);
        //pinocchio::SE3 fMi = m_data->oMi[ffIndex].inverse()*m_data->oMi[torsoIndex]*m_torso_X_imu;
        pinocchio::SE3 oMimu = m_data->oMi[torsoIndex]*m_torso_X_imu;

        //Move the IMU signal to the base frame.
        //angularAcceleration is zero. Intermediate frame acc and velocities are zero
        const dynamicgraph::Vector acc = oMimu.rotation()*IMU_acc; //Torso Acceleration

        //Deal with gravity predefined in robot model. Robot Z should be pointing upwards
        m_model.gravity.linear() = acc;
        //Set fixed for DEBUG
        //m_model.gravity.linear() = m_model.gravity981;

        const Eigen::VectorXd& tau_rnea = pinocchio::rnea(m_model, *m_data, enc,
                                                    Eigen::VectorXd::Zero(m_model.nv),
                                                    Eigen::VectorXd::Zero(m_model.nv));
        const Eigen::VectorXd current_offset = tau - tau_rnea.tail(m_model.nv-6);
        if(current_offset.array().abs().maxCoeff() >=epsilon) 
        {
          SEND_MSG("Too high torque offset estimated for iteration"+ i, MSG_TYPE_ERROR);
          assert(false);
        }
        // encSignals.push_back(_enc);
        // accSignals.push_back(_acc);
        // gyrSignals.push_back(_gyr);
        // tauSignals.push_back(_tau);
        jointTorqueOffsets += current_offset;
        current_progress++;
      }
      else if (i == n_iterations) 
      {
        // Find the mean, enough signals collected
        jointTorqueOffsets /=n_iterations;
        sensor_offset_status = COMPUTED;
      }
      else 
      { // i > n_iterations
        //Doesn't reach here.
        assert(false && "Error in collectSensorData. ");
      }
    }
    //        else { // sensor_offset_status == COMPUTED
    //        } 
    return s; 
  }

  DEFINE_SIGNAL_OUT_FUNCTION(jointTorquesEstimated, dynamicgraph::Vector)
  {
    m_collectSensorDataSINNER(iter);

    if (s.size() != m_model.nv-6) s.resize(m_model.nv-6);

    if (sensor_offset_status == PRECOMPUTATION || sensor_offset_status == INPROGRESS) 
    {
      s = m_jointTorquesSIN(iter);
    }
    else 
    { //sensor_offset_status == COMPUTED
      const dynamicgraph::Vector& inputTorques = m_jointTorquesSIN(iter);
      s = inputTorques - jointTorqueOffsets;
    }
    return s;
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
