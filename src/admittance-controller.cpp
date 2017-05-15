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
#include <sot/torque_control/utils/stop-watch.hh>

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
      using namespace metapod;
      using namespace Eigen;

#define PROFILE_DQ_DES_COMPUTATION "Admittance control computation"

#define REF_FORCE_SIGNALS m_fRightFootRefSIN << m_fLeftFootRefSIN << \
                          m_fRightHandRefSIN << m_fLeftHandRefSIN
#define FORCE_SIGNALS     m_fRightFootSIN << m_fLeftFootSIN << \
                          m_fRightHandSIN << m_fLeftHandSIN
#define GAIN_SIGNALS      m_KdSIN << m_KfSIN
#define STATE_SIGNALS     m_base6d_encodersSIN << m_jointsVelocitiesSIN

#define INPUT_SIGNALS     STATE_SIGNALS << REF_FORCE_SIGNALS << \
                          FORCE_SIGNALS << GAIN_SIGNALS << m_controlledJointsSIN << m_dampingSIN

#define FORCE_ERROR_SIGNALS m_fRightFootErrorSOUT << m_fLeftFootErrorSOUT << m_fRightHandErrorSOUT << m_fLeftHandErrorSOUT
#define OUTPUT_SIGNALS      m_qDesSOUT << m_dqDesSOUT << FORCE_ERROR_SIGNALS

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef AdmittanceController EntityClassName;

      typedef Eigen::Matrix<double,N_JOINTS,1>                     VectorN;
      typedef Eigen::Matrix<double,N_JOINTS+6,1>                   VectorN6;
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
            ,CONSTRUCT_SIGNAL_IN(base6d_encoders,     dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(jointsVelocities,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kd,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kf,                  dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFootRef,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFootRef,        dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightHandRef,       dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftHandRef,        dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightFoot,          dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftFoot,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fRightHand,          dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(fLeftHand,           dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(controlledJoints,    dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_IN(damping,             dynamicgraph::Vector)
            ,CONSTRUCT_SIGNAL_OUT(qDes,             dynamicgraph::Vector, STATE_SIGNALS<<
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
            ,CONSTRUCT_SIGNAL_OUT(fRightHandError,  dynamicgraph::Vector, m_fRightHandSIN <<
                                                                m_fRightHandRefSIN)
            ,CONSTRUCT_SIGNAL_OUT(fLeftHandError,   dynamicgraph::Vector, m_fLeftHandSIN <<
                                                                m_fLeftHandRefSIN)
            ,m_initSucceeded(false)
            ,m_useJacobianTranspose(false)
            ,m_node_right_foot(boost::fusion::at_c<Hrp2_14::r_ankle>(m_robot.nodes))
            ,m_node_left_foot(boost::fusion::at_c<Hrp2_14::l_ankle>(m_robot.nodes))
            ,m_node_right_hand(boost::fusion::at_c<Hrp2_14::r_wrist>(m_robot.nodes))
            ,m_node_left_hand(boost::fusion::at_c<Hrp2_14::l_wrist>(m_robot.nodes))
      {
        m_J_all = AllJacobian::Zero();

        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("getUseJacobianTranspose",
                   makeDirectGetter(*this,&m_useJacobianTranspose,
                                    docDirectGetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse","bool")));
        addCommand("setUseJacobianTranspose",
                   makeDirectSetter(*this, &m_useJacobianTranspose,
                                    docDirectSetter("If true it uses the Jacobian transpose, otherwise the pseudoinverse",
                                                    "bool")));
        addCommand("init",
                   makeCommandVoid1(*this, &AdmittanceController::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Time period in seconds (double)")));
      }

      void AdmittanceController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_KdSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
        if(!m_KfSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kf is not plugged", MSG_TYPE_ERROR);
        if(!m_controlledJointsSIN.isPlugged())
          return SEND_MSG("Init failed: signal controlledJoints is not plugged", MSG_TYPE_ERROR);

        m_dt = dt;
        m_qDes.setZero(N_JOINTS);
        m_dqDes.setZero(N_JOINTS);
        m_q.setZero();
        m_dq.setZero();

        // compute transformation from foot frame to sole frame
        // The second argument of the Transform constructor has to be the position
        // of the sole w.r.t. the local frame in local coordinates
        Spatial::RotationMatrixIdentityTpl<double> eye;
        typedef Eigen::Map<const Eigen::Vector3d> CMap3d;
        m_sole_X_RF = Transform(eye, CMap3d(RIGHT_FOOT_SOLE_XYZ));
        m_sole_X_LF = Transform(eye, CMap3d(LEFT_FOOT_SOLE_XYZ));
        m_gripper_X_RH = Transform(eye, CMap3d(RIGHT_HAND_GRIPPER_XYZ));
        m_gripper_X_LH = Transform(eye, CMap3d(LEFT_HAND_GRIPPER_XYZ));

        m_initSucceeded = true;
        m_firstIter = true;
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(qDes,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal qDes before initialization!");
          return s;
        }

        const Eigen::VectorXd& q =             m_base6d_encodersSIN(iter);
        if(m_firstIter)
        {
          // at the fist iteration store the joint positions as desired joint positions
          m_qDes = q.tail<N_JOINTS>();
          m_firstIter = false;
        }

        const Eigen::VectorXd& dqDes  =                m_dqDesSOUT(iter); // n
        const Eigen::VectorXd& qMask =       m_controlledJointsSIN(iter); // n

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
        {
          // if the joint is active integrate the desired velocity to get the new desired position
          if(qMask[i]!=0.0)
            m_qDes(i) += m_dt * dqDes(i);
          // if the joint is not active just check whether the desired joint position is too far
          // from the measured position, if this is the case it probably means that another
          // controller is moving that joint, so we update the desired joint position to the measured
          // position (@todo check this only when a joint switch from unactive to active)
          else if(fabs(q(6+i)-m_qDes(i))>DEFAULT_MAX_DELTA_Q)
          {
            m_qDes(i) = q(6+i);
//            SEND_MSG("Resetting qDes for joint "+JointUtil::get_name_from_id(i)+" because it was "+
//                     "too far from q,  q-qDes="+toString(q(6+i)-m_qDes(i)), MSG_TYPE_WARNING);
          }
        }

	s = m_qDes;
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
          const Vector6& e_f_RF =      m_fRightFootErrorSOUT(iter); // 6
          const Vector6& e_f_LF =      m_fLeftFootErrorSOUT(iter);  // 6
//          const Eigen::VectorXd Kd,          m_KdSIN(iter));               // n
          const VectorN6& q  =          m_base6d_encodersSIN(iter);  // n+6
          const VectorN& dq =          m_jointsVelocitiesSIN(iter); // n
          const VectorN& qMask =       m_controlledJointsSIN(iter); // n
          const Eigen::Vector4d& damping =     m_dampingSIN(iter);          // 4

          assert(e_f_RF.size()==6       && "Unexpected size of signal fRightFootError");
          assert(e_f_LF.size()==6       && "Unexpected size of signal fLeftFootError");
//          assert(Kd.size()==N_JOINTS    && "Unexpected size of signal Kd");
          assert(q.size()==N_JOINTS+6   && "Unexpected size of signal base6d_encoder");
          assert(dq.size()==N_JOINTS    && "Unexpected size of signal dq");
          assert(qMask.size()==N_JOINTS && "Unexpected size of signal controlledJoints");
          assert(damping.size()==4      && "Unexpected size of signal damping");

          /// *** Compute all Jacobians ***
          m_q.head<6>().setZero();
          m_q.tail<N_JOINTS>()   = q.tail<N_JOINTS>();
          m_dq.tail<N_JOINTS>()  = dq;
          // jcalc computes homogeneous transformations (sXp) and local velocities (vj)
          jcalc< Hrp2_14 >::run(m_robot, m_q, m_dq);
          // compute Jacobians
          jac< Hrp2_14 >::run(m_robot, m_J_all);
          // set to zero columns of Jacobian corresponding to unactive joints
          for(int i=0; i<N_JOINTS; i++)
            if(qMask(i)==0)
              m_J_all.col(6+i).setZero();
          // extract Jacobians of bodies of interest
          m_J_right_foot = m_J_all.block<6,6>(6*Hrp2_14::r_ankle,6);
          m_J_left_foot  = m_J_all.block<6,6>(6*Hrp2_14::l_ankle,12);
          // map foot Jacobians to soles and hand Jacobians to grippers
          applyTransformToMatrix(m_sole_X_RF, m_J_right_foot);
          applyTransformToMatrix(m_sole_X_LF, m_J_left_foot);
//          applyTransformToMatrix(m_gripper_X_RH, m_J_right_hand);
//          applyTransformToMatrix(m_gripper_X_LH, m_J_left_hand);

          /// Compute admittance control law
          if(m_useJacobianTranspose)
          {
            m_dqDes.head<6>()     = m_J_right_foot.transpose()*e_f_RF;
            m_dqDes.segment<6>(6) = m_J_left_foot.transpose()*e_f_LF;
          }
          else
          {
            m_J_right_foot_svd.compute(m_J_right_foot, ComputeThinU | ComputeThinV);
            m_J_left_foot_svd.compute( m_J_left_foot,  ComputeThinU | ComputeThinV);

            m_dqDes.head<6>()     = svdSolveWithDamping(m_J_right_foot_svd, e_f_RF, damping[0]);
            m_dqDes.segment<6>(6) = svdSolveWithDamping(m_J_left_foot_svd,  e_f_LF, damping[1]);
          }

//          SEND_INFO_STREAM_MSG("J_right_foot:\n"+toString(m_J_right_foot));
//          SEND_INFO_STREAM_MSG("J_right_foot sv: "+toString(m_J_right_foot_svd.singularValues().transpose()));
//          SEND_INFO_STREAM_MSG("e_f_RF: "+toString(e_f_RF.transpose()));
//          SEND_INFO_STREAM_MSG("dqDes: "+toString(m_dqDes.transpose()));

          // @todo Implement admittance control for upper body
//          m_J_right_hand.leftCols<2>()  = m_J_all.block<6,2>(6*Hrp2_14::r_wrist,12);
//          m_J_right_hand.rightCols<7>() = m_J_all.block<6,7>(6*Hrp2_14::r_wrist,16);
//          m_J_left_hand.leftCols<2>()   = m_J_all.block<6,2>(6*Hrp2_14::l_wrist,12);
//          m_J_left_hand.rightCols<7>()  = m_J_all.block<6,7>(6*Hrp2_14::l_wrist,23);
//          m_J_right_hand_svd.compute(m_J_right_hand, ComputeThinU | ComputeThinV);
//          m_J_left_hand_svd.compute( m_J_left_hand,  ComputeThinU | ComputeThinV);
//          m_dqDes.head<6>() = svdSolveWithDamping(m_J_right_hand_svd, e_f_RH, damping[2]);
//          m_dqDes.head<6>() = svdSolveWithDamping(m_J_left_hand_svd,  e_f_LH, damping[3]);

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
	s = m_dqDes;
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

        const Eigen::Matrix<double,24,1>& Kf =          m_KfSIN(iter); // 6*4
        const Vector6& f =           m_fRightFootSIN(iter);      // 6
        const Vector6& fRef =        m_fRightFootRefSIN(iter);   // 6
        assert(f.size()==6     && "Unexpected size of signal fRightFoot");
        assert(fRef.size()==6  && "Unexpected size of signal fRightFootRef");

        if(s.size()!=6)
          s.resize(6);
	s.tail<3>() = Kf.head<3>().cwiseProduct(fRef.head<3>() - f.head<3>() );
	s.head<3>() = Kf.segment<3>(3).cwiseProduct(fRef.tail<3>() - f.tail<3>());

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fLeftFootError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal fLeftFootError before initialization!", MSG_TYPE_WARNING_STREAM);
          return s;
        }

        const Eigen::Matrix<double,24,1>& Kf =         m_KfSIN(iter); // 6*4
        const Vector6& f =           m_fLeftFootSIN(iter);      // 6
        const Vector6& fRef =        m_fLeftFootRefSIN(iter);   // 6
        assert(f.size()==6     && "Unexpected size of signal fLeftFoot");
        assert(fRef.size()==6  && "Unexpected size of signal fLeftFootRef");

        if(s.size()!=6)
          s.resize(6);
	s.tail<3>() = Kf.segment<3>(6).cwiseProduct(fRef.head<3>() - f.head<3>() );
	s.head<3>() = Kf.segment<3>(9).cwiseProduct(fRef.tail<3>() - f.tail<3>());

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fRightHandError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal fRightHandError before initialization!", MSG_TYPE_WARNING_STREAM);
          return s;
        }

        const Eigen::Matrix<double,24,1>& Kf =        m_KfSIN(iter); // 6*4
        const Vector6& f =          m_fRightHandSIN(iter);      // 6
        const Vector6& fRef =       m_fRightHandRefSIN(iter);   // 6
        assert(f.size()==6     && "Unexpected size of signal fRightHand");
        assert(fRef.size()==6  && "Unexpected size of signal fRightHandRef");

        if(s.size()!=6)
          s.resize(6);
	s.tail<3>() = Kf.segment<3>(12).cwiseProduct(fRef.head<3>() - f.head<3>() );
	s.head<3>() = Kf.segment<3>(15).cwiseProduct(fRef.tail<3>() - f.tail<3>());
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(fLeftHandError,dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal fLeftHandError before initialization!", MSG_TYPE_WARNING_STREAM);
          return s;
        }

        const Eigen::Matrix<double,24,1>&  Kf =         m_KfSIN(iter); // 6*4
        const Vector6& f =           m_fLeftHandSIN(iter);      // 6
        const Vector6& fRef =        m_fLeftHandRefSIN(iter);   // 6
        assert(f.size()==6     && "Unexpected size of signal fLeftHand");
        assert(fRef.size()==6  && "Unexpected size of signal fLeftHandRef");

        if(s.size()!=6)
          s.resize(6);
	s.tail<3>() = Kf.segment<3>(18).cwiseProduct(fRef.head<3>() - f.head<3>() );
	s.head<3>() = Kf.segment<3>(21).cwiseProduct(fRef.tail<3>() - f.tail<3>());

        return s;
      }

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

