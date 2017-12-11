/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#include <sot/torque_control/admittance-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/metapod-helper.hh>
#include <tsid/utils/stop-watch.hpp>

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
      using namespace Eigen;
      using namespace tsid;
      using namespace tsid::math;
      using namespace tsid::tasks;

#define PROFILE_DQ_DES_COMPUTATION "Admittance control computation"

#define REF_FORCE_SIGNALS m_fRightFootRefSIN << m_fLeftFootRefSIN
//                          m_fRightHandRefSIN << m_fLeftHandRefSIN
#define FORCE_SIGNALS     m_fRightFootSIN << m_fLeftFootSIN
//                          m_fRightHandSIN << m_fLeftHandSIN
#define GAIN_SIGNALS      m_kp_forceSIN << m_ki_forceSIN << m_kp_velSIN << m_ki_velSIN << m_force_integral_saturationSIN
#define STATE_SIGNALS     m_encodersSIN << m_jointsVelocitiesSIN

#define INPUT_SIGNALS     STATE_SIGNALS << REF_FORCE_SIGNALS << \
                          FORCE_SIGNALS << GAIN_SIGNALS << m_controlledJointsSIN << m_dampingSIN

#define FORCE_ERROR_SIGNALS m_fRightFootErrorSOUT << m_fLeftFootErrorSOUT //<< m_fRightHandErrorSOUT << m_fLeftHandErrorSOUT
#define OUTPUT_SIGNALS      m_uSOUT << m_dqDesSOUT << FORCE_ERROR_SIGNALS

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef AdmittanceController EntityClassName;

      typedef Eigen::Matrix<double,3,1>                            Vector3;
      typedef Eigen::Matrix<double,6,1>                            Vector6;
      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(AdmittanceController,
                                         "AdmittanceController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      AdmittanceController::AdmittanceController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(encoders,            dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(jointsVelocities,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_force,            dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(ki_force,            dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_vel,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(ki_vel,              dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(force_integral_saturation, dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFootRef,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFootRef,        dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFoot,          dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFoot,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(controlledJoints,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(damping,             dynamicgraph::Vector)
//            ,CONSTRUCT_SIGNAL_IN(fRightHandRef,       dynamicgraph::Vector)
//            ,CONSTRUCT_SIGNAL_IN(fLeftHandRef,        dynamicgraph::Vector)
//            ,CONSTRUCT_SIGNAL_IN(fRightHand,          dynamicgraph::Vector)
//            ,CONSTRUCT_SIGNAL_IN(fLeftHand,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(u,                  dynamicgraph::Vector, STATE_SIGNALS<<
                                                                FORCE_SIGNALS<<
                                                                REF_FORCE_SIGNALS<<
                                                                GAIN_SIGNALS<<
                                                                m_controlledJointsSIN)
            ,CONSTRUCT_SIGNAL_OUT(dqDes,            dynamicgraph::Vector, STATE_SIGNALS <<
                                                                GAIN_SIGNALS <<
                                                                FORCE_ERROR_SIGNALS <<
                                                                m_dampingSIN)
            ,CONSTRUCT_SIGNAL_OUT(fRightFootError,  dynamicgraph::Vector, m_fRightFootSIN <<
                                                                m_fRightFootRefSIN)
            ,CONSTRUCT_SIGNAL_OUT(fLeftFootError,   dynamicgraph::Vector, m_fLeftFootSIN <<
                                                                m_fLeftFootRefSIN)
//            ,CONSTRUCT_SIGNAL_OUT(fRightHandError,  dynamicgraph::Vector, m_fRightHandSIN <<
//                                                                m_fRightHandRefSIN)
//            ,CONSTRUCT_SIGNAL_OUT(fLeftHandError,   dynamicgraph::Vector, m_fLeftHandSIN <<
//                                                                m_fLeftHandRefSIN)
            ,m_initSucceeded(false)
            ,m_useJacobianTranspose(false)
            ,m_firstIter(true)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        m_v_RF_int.setZero();
        m_v_LF_int.setZero();

        /* Commands. */
        addCommand("getUseJacobianTranspose",
                   makeDirectGetter(*this,&m_useJacobianTranspose,
                                    docDirectGetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse","bool")));
        addCommand("setUseJacobianTranspose",
                   makeDirectSetter(*this, &m_useJacobianTranspose,
                                    docDirectSetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse",
                                                    "bool")));
        addCommand("init",
                   makeCommandVoid2(*this, &AdmittanceController::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)",
                                                    "Robot name (string)")));
      }

      void AdmittanceController::init(const double& dt, const std::string& robotRef)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_controlledJointsSIN.isPlugged())
          return SEND_MSG("Init failed: signal controlledJoints is not plugged", MSG_TYPE_ERROR);

        m_dt = dt;
        m_initSucceeded = true;

        /* Retrieve m_robot_util  informations */
        std::string localName(robotRef);
        if (isNameInRobotUtil(localName))
          m_robot_util = getRobotUtil(localName);
        else
          return SEND_MSG("You should have an entity controller manager initialized before", MSG_TYPE_ERROR);

        try
        {
          vector<string> package_dirs;
          m_robot = new robots::RobotWrapper(m_robot_util->m_urdf_filename,
                                             package_dirs,
                                             se3::JointModelFreeFlyer());
          m_data = new se3::Data(m_robot->model());

          assert(m_robot->nv()>=6);
          m_robot_util->m_nbJoints = m_robot->nv()-6;
          m_nj = m_robot->nv()-6;
          m_u.setZero(m_nj);
          m_dq_des_urdf.setZero(m_nj);
          m_dqErrIntegral.setZero(m_nj);
          //m_dqDesIntegral.setZero(m_nj);

          m_q_urdf.setZero(m_robot->nq());
          m_q_urdf(6) = 1.0;
          m_v_urdf.setZero(m_robot->nv());
          m_J_RF.setZero(6, m_robot->nv());
          m_J_LF.setZero(6, m_robot->nv());

          m_frame_id_rf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Right_Foot_Frame_Name);
          m_frame_id_lf = (int)m_robot->model().getFrameId(m_robot_util->m_foot_util.m_Left_Foot_Frame_Name);

        }
        catch (const std::exception& e)
        {
          std::cout << e.what();
          return SEND_MSG("Init failed: Could load URDF :" + m_robot_util->m_urdf_filename, MSG_TYPE_ERROR);
        }
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(u, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal u before initialization!");
          return s;
        }

        const Vector& dqDes  = m_dqDesSOUT(iter); // n
        const Vector& q      = m_encodersSIN(iter);  // n
        const Vector& dq     = m_jointsVelocitiesSIN(iter); // n
        const Vector& kp     = m_kp_velSIN(iter);
        const Vector& ki     = m_ki_velSIN(iter);

        if(m_firstIter)
        {
          m_qPrev = q;
          m_firstIter = false;
        }

        // estimate joint velocities using finite differences
        m_dq_fd = (q-m_qPrev)/m_dt;
        m_qPrev = q;

        m_dqErrIntegral += m_dt * ki.cwiseProduct(dqDes - m_dq_fd);
        s = kp.cwiseProduct(dqDes-dq) + m_dqErrIntegral;

        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(dqDes,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dqDes before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_DQ_DES_COMPUTATION);
        {
          const Vector6& e_f_RF          = m_fRightFootErrorSOUT(iter); // 6
          const Vector6& e_f_LF          = m_fLeftFootErrorSOUT(iter);  // 6
          const Vector6d& kp             = m_kp_forceSIN(iter);
          const Vector6d& ki             = m_ki_forceSIN(iter);
          const Vector6d& f_sat          = m_force_integral_saturationSIN(iter);
          const Vector& q_sot            = m_encodersSIN(iter);  // n
//          const Vector& dq_sot           = m_jointsVelocitiesSIN(iter); // n
          //const Vector& qMask            = m_controlledJointsSIN(iter); // n
          //const Eigen::Vector4d& damping = m_dampingSIN(iter);          // 4

          assert(q.size()==m_nj     && "Unexpected size of signal encoder");
          assert(dq.size()==m_nj    && "Unexpected size of signal dq");
          assert(qMask.size()==m_nj && "Unexpected size of signal controlledJoints");

          Eigen::Vector6d v_des_RF = -kp.cwiseProduct(e_f_RF);
          Eigen::Vector6d v_des_LF = -kp.cwiseProduct(e_f_LF);

          m_v_RF_int -= m_dt*ki.cwiseProduct(e_f_RF);
          m_v_LF_int -= m_dt*ki.cwiseProduct(e_f_LF);

          // saturate
          bool saturating_LF = false;
          bool saturating_RF = false;
          for(int i=0; i<6; i++)
          {
            if(m_v_RF_int(i) > f_sat(i))
            {
              saturating_RF = true;
              m_v_RF_int(i) = f_sat(i);
            }
            else if(m_v_RF_int(i) < -f_sat(i))
            {
              saturating_RF = true;
              m_v_RF_int(i) = -f_sat(i);
            }

            if(m_v_LF_int(i) > f_sat(i))
            {
              saturating_LF = true;
              m_v_LF_int(i) = f_sat(i);
            }
            else if(m_v_LF_int(i) < -f_sat(i))
            {
              saturating_LF = true;
              m_v_LF_int(i) = -f_sat(i);
            }
          }
          if(saturating_LF)
            SEND_INFO_STREAM_MSG("Saturate m_v_LF_int integral: "+toString(m_v_LF_int.transpose()));
          if(saturating_RF)
            SEND_INFO_STREAM_MSG("Saturate m_v_RF_int integral: "+toString(m_v_RF_int.transpose()));


          /// *** Compute all Jacobians ***
          m_robot_util->joints_sot_to_urdf(q_sot, m_q_urdf.tail(m_nj));
          m_robot->computeAllTerms(*m_data, m_q_urdf, m_v_urdf);
          m_robot->frameJacobianLocal(*m_data, m_frame_id_rf, m_J_RF);
          m_robot->frameJacobianLocal(*m_data, m_frame_id_lf, m_J_LF);

          //SEND_INFO_STREAM_MSG("RFoot Jacobian :" + toString(m_J_RF.rightCols(m_nj)));
          // set to zero columns of Jacobian corresponding to unactive joints
//          for(int i=0; i<m_nj; i++)
//            if(qMask(i)==0)
//              m_J_all.col(6+i).setZero();

          /// Compute admittance control law
          if(m_useJacobianTranspose)
          {
            m_dq_des_urdf  = m_J_RF.rightCols(m_nj).transpose()*(v_des_RF+m_v_RF_int);
            m_dq_des_urdf += m_J_LF.rightCols(m_nj).transpose()*(v_des_LF+m_v_LF_int);
          }
          else
          {
            m_J_RF_QR.compute(m_J_RF.rightCols(m_nj));
            m_J_LF_QR.compute(m_J_LF.rightCols(m_nj));

            m_dq_des_urdf = m_J_RF_QR.solve(v_des_RF+m_v_RF_int);
//            SEND_INFO_STREAM_MSG("v_des_RF+m_v_RF_int:" + toString((v_des_RF+m_v_RF_int).transpose()));
//            SEND_INFO_STREAM_MSG("dq_des_urdf (RF only):" + toString(m_dq_des_urdf.transpose()));
            m_dq_des_urdf += m_J_LF_QR.solve(v_des_LF+m_v_LF_int);
//            SEND_INFO_STREAM_MSG("dq_des_urdf (RF+LF):" + toString(m_dq_des_urdf.transpose()));
          }

          if(s.size()!=m_nj)
            s.resize(m_nj);

          m_robot_util->joints_urdf_to_sot(m_dq_des_urdf, s);
        }
        getProfiler().stop(PROFILE_DQ_DES_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fRightFootError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal fRightFootError before initialization!", MSG_TYPE_WARNING_STREAM);
          return s;
        }
        const Vector6& f =           m_fRightFootSIN(iter);      // 6
        const Vector6& fRef =        m_fRightFootRefSIN(iter);   // 6
        if(s.size()!=6)
          s.resize(6);
        s = fRef - f;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fLeftFootError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal fLeftFootError before initialization!", MSG_TYPE_WARNING_STREAM);
          return s;
        }
        const Vector6& f =           m_fLeftFootSIN(iter);      // 6
        const Vector6& fRef =        m_fLeftFootRefSIN(iter);   // 6
        if(s.size()!=6)
          s.resize(6);
        s = fRef - f;
        return s;
      }

//      DEFINE_SIGNAL_OUT_FUNCTION(fRightHandError,dynamicgraph::Vector)
//      {
//        if(!m_initSucceeded)
//        {
//          SEND_MSG("Cannot compute signal fRightHandError before initialization!", MSG_TYPE_WARNING_STREAM);
//          return s;
//        }

//        const Eigen::Matrix<double,24,1>& Kf =        m_KfSIN(iter); // 6*4
//        const Vector6& f =          m_fRightHandSIN(iter);      // 6
//        const Vector6& fRef =       m_fRightHandRefSIN(iter);   // 6
//        assert(f.size()==6     && "Unexpected size of signal fRightHand");
//        assert(fRef.size()==6  && "Unexpected size of signal fRightHandRef");

//        if(s.size()!=6)
//          s.resize(6);
//        s.tail<3>() = Kf.segment<3>(12).cwiseProduct(fRef.head<3>() - f.head<3>() );
//        s.head<3>() = Kf.segment<3>(15).cwiseProduct(fRef.tail<3>() - f.tail<3>());
//        return s;
//      }

//      DEFINE_SIGNAL_OUT_FUNCTION(fLeftHandError,dynamicgraph::Vector)
//      {
//        if(!m_initSucceeded)
//        {
//          SEND_MSG("Cannot compute signal fLeftHandError before initialization!", MSG_TYPE_WARNING_STREAM);
//          return s;
//        }

//        const Eigen::Matrix<double,24,1>&  Kf =         m_KfSIN(iter); // 6*4
//        const Vector6& f =           m_fLeftHandSIN(iter);      // 6
//        const Vector6& fRef =        m_fLeftHandRefSIN(iter);   // 6
//        assert(f.size()==6     && "Unexpected size of signal fLeftHand");
//        assert(fRef.size()==6  && "Unexpected size of signal fLeftHandRef");

//        if(s.size()!=6)
//          s.resize(6);
//	s.tail<3>() = Kf.segment<3>(18).cwiseProduct(fRef.head<3>() - f.head<3>() );
//	s.head<3>() = Kf.segment<3>(21).cwiseProduct(fRef.tail<3>() - f.tail<3>());

//        return s;
//      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void AdmittanceController::display(std::ostream& os) const
      {
        os << "AdmittanceController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }


      void AdmittanceController::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotAdmittanceController:\n"
              << "\t -." << std::endl;
          Entity::commandLine(cmdLine, cmdArgs, os);
        }
        else
        {
          Entity::commandLine(cmdLine,cmdArgs,os);
        }
      }

      //**************************************************************************************************
      VectorXd svdSolveWithDamping(const JacobiSVD<MatrixXd>& A, const VectorXd &b, double damping)
      {
          eigen_assert(A.computeU() && A.computeV() && "svdSolveWithDamping() requires both unitaries U and V to be computed (thin unitaries suffice).");
          assert(A.rows()==b.size());

          //    cout<<"b = "<<toString(b,1)<<endl;
          VectorXd tmp(A.cols());
          int nzsv = A.nonzeroSingularValues();
          tmp.noalias() = A.matrixU().leftCols(nzsv).adjoint() * b;
          //    cout<<"U^T*b = "<<toString(tmp,1)<<endl;
          double sv, d2 = damping*damping;
          for(int i=0; i<nzsv; i++)
          {
              sv = A.singularValues()(i);
              tmp(i) *= sv/(sv*sv + d2);
          }
          //    cout<<"S^+ U^T b = "<<toString(tmp,1)<<endl;
          VectorXd res = A.matrixV().leftCols(nzsv) * tmp;

          //    getLogger().sendMsg("sing val = "+toString(A.singularValues(),3), MSG_STREAM_INFO);
          //    getLogger().sendMsg("solution with damp "+toString(damping)+" = "+toString(res.norm()), MSG_STREAM_INFO);
          //    getLogger().sendMsg("solution without damping  ="+toString(A.solve(b).norm()), MSG_STREAM_INFO);

          return res;
      }
      
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

