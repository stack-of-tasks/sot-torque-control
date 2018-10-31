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

#ifndef __sot_torque_control_trajectory_generator_H__
#define __sot_torque_control_trajectory_generator_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (trajectory_generator_EXPORTS)
#    define TRAJECTORYGENERATOR_EXPORT __declspec(dllexport)
#  else
#    define TRAJECTORYGENERATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define HRP2COMMON_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */
#include <iostream>
#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <map>
#include <fstream>  /// to read text file
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      #define MAXBUFSIZE  ((int) 1000000)

      Eigen::MatrixXd readMatrixFromFile(const char *filename)
      {
        int cols = 0, rows = 0;
        double buff[MAXBUFSIZE];

        // Read numbers from file into buffer.
        std::ifstream infile;
        infile.open(filename);
        while (! infile.eof())
        {
          std::string line;
          getline(infile, line);
//          std::cout<<"Read line "<<rows<<"\n";

          int temp_cols = 0;
          std::stringstream stream(line);
          while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

          if (temp_cols == 0)
            continue;

          if (cols == 0)
            cols = temp_cols;
          else if(temp_cols!=cols && !infile.eof())
          {
            std::cout<<"Error while reading matrix from file, line "<<rows<<" has "<<temp_cols<<" columnds, while preceding lines had "<<cols<<" columnds\n";
            std::cout<<line<<"\n";
            break;
          }

          rows++;
          if((rows+1)*cols>=MAXBUFSIZE)
          {
            std::cout<<"Max buffer size exceeded ("<<rows<<" rows, "<<cols<<" cols)\n";
            break;
          }
        }
        infile.close();
        rows--;

        // Populate matrix with numbers.
        Eigen::MatrixXd result(rows,cols);
        for (int i = 0; i < rows; i++)
          for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

        return result;
      }

      class AbstractTrajectoryGenerator
      {
      protected:
        Eigen::VectorXd m_x;          /// current position
        Eigen::VectorXd m_dx;         /// current velocity
        Eigen::VectorXd m_ddx;        /// current acceleration
        Eigen::VectorXd m_x_init;     /// initial position
        Eigen::VectorXd m_x_final;    /// final position

        double          m_traj_time;  /// time to go from x_init to x_final (sec)
        double          m_dt;         /// control dt (sampling period of the trajectory)
        double          m_t;          /// current time
	Eigen::VectorXd::Index        m_size;

        virtual void resizeAllData(Eigen::VectorXd::Index size)
        {
          m_size      = size;
          m_x.setZero(size);
          m_dx.setZero(size);
          m_ddx.setZero(size);
          m_x_init.setZero(size);
          m_x_final.setZero(size);
        }

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[AbstrTrajGen] "+msg, t, file, line);
        }

      public:

        AbstractTrajectoryGenerator(double dt, double traj_time, 
				    Eigen::VectorXd::Index size)
        {
          m_t         = 0.0;
          m_dt        = dt;
          m_traj_time = traj_time;
          resizeAllData(size);
        }

        AbstractTrajectoryGenerator(double dt, double traj_time, const Eigen::VectorXd &x_init, const Eigen::VectorXd &x_final)
        {
          assert(x_init.size()==x_final.size() && "Initial and final state must have the same size");
          m_dt = dt;
          m_traj_time = traj_time;
          resizeAllData(x_init.size());
          set_initial_point(x_init);
          set_final_point(x_final);
        }

        virtual bool set_initial_point(const Eigen::VectorXd& x_init)
        {
          if(x_init.size()!=m_x_init.size())
            return false;
          m_x_init = x_init;
          m_x      = x_init;
          m_dx.setZero();
          m_t      = 0.0;
          return true;
        }

        virtual bool set_initial_point(const double& x_init)
        {
          if(1!=m_x_init.size())
            return false;
          m_x_init(0) = x_init;
          m_x(0)      = x_init;
          m_dx(0)     = 0.0;
          m_t         = 0.0;
          return true;
        }

        virtual bool set_final_point(const Eigen::VectorXd& x_final)
        {
          if(x_final.size()!=m_x_final.size())
            return false;
          m_x_final = x_final;
          return true;
        }

        virtual bool set_final_point(const double& x_final)
        {
          if(1!=m_x_final.size())
            return false;
          m_x_final(0) = x_final;
          return true;
        }

        virtual bool set_trajectory_time(double traj_time)
        {
          if(traj_time<=0.0)
            return false;
          m_traj_time = traj_time;
          return true;
        }

        virtual const Eigen::VectorXd& compute_next_point() = 0;

        virtual const Eigen::VectorXd& getPos(){ return m_x; }
        virtual const Eigen::VectorXd& getVel(){ return m_dx; }
        virtual const Eigen::VectorXd& getAcc(){ return m_ddx; }
        virtual const Eigen::VectorXd& get_initial_point(){ return m_x_init; }
        virtual const Eigen::VectorXd& get_final_point(){ return m_x_final; }

        virtual bool isTrajectoryEnded(){ return m_t>= m_traj_time; }

      };

      /** Fake trajectory generator that actually generates no trajectory
       *  at all.
       * */
      class NoTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      public:
        NoTrajectoryGenerator(int size):
          AbstractTrajectoryGenerator(0.001, 1.0, size)
        {}

        virtual const Eigen::VectorXd& compute_next_point()
        {
          m_t += m_dt;
          return m_x;
        }

        virtual bool isTrajectoryEnded(){ return false; }
      };


      /** Trajectory generator that reads the trajectory and its derivatives from a text file.
       * */
      class TextFileTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      protected:
        Eigen::MatrixXd m_posTraj;
        Eigen::MatrixXd m_velTraj;
        Eigen::MatrixXd m_accTraj;

      public:
        TextFileTrajectoryGenerator(double dt, Eigen::VectorXd::Index size):
          AbstractTrajectoryGenerator(dt, 1.0, size)
        {}

        virtual bool loadTextFile(const std::string& fileName)
        {
          Eigen::MatrixXd data = readMatrixFromFile(fileName.c_str());
//          std::cout<<"Read matrix with "<<data.rows()<<" rows and "<<data.cols()<<" cols from text file\n";
          if(data.cols()!=3*m_size)
          {
            std::cout<<"Unexpected number of columns (expected "<<3*m_size<<", found "<<data.cols()<<")\n";
            return false;
          }

          m_traj_time = m_dt*(double)data.rows();
          m_t = 0.0;

          m_posTraj = data.leftCols(m_size);
          m_velTraj = data.middleCols(m_size,m_size);
          m_accTraj = data.rightCols(m_size);

          m_x_init = m_posTraj.row(0);

          return true;
        }

        virtual const Eigen::VectorXd& compute_next_point()
        {
	  Eigen::VectorXd::Index i = (Eigen::VectorXd::Index)std::floor(m_t/m_dt);
          if(i<m_posTraj.rows())
          {
            m_x   = m_posTraj.row(i);
            m_dx  = m_velTraj.row(i);
            m_ddx = m_accTraj.row(i);
          }
          m_t += m_dt;
          return m_x;
        }
      };


      /** Point-to-point minimum-jerk trajectory generator. */
      class MinimumJerkTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      public:
        MinimumJerkTrajectoryGenerator(double dt, double traj_time, int size):
          AbstractTrajectoryGenerator(dt, traj_time, size)
        {}

        virtual const Eigen::VectorXd& compute_next_point()
        {
          if(m_t <= m_traj_time)
          {
            double td  = m_t/m_traj_time;
            double td2 = td*td;
            double td3 = td2*td;
            double td4 = td3*td;
            double td5 = td4*td;
            double p   = 10*td3 - 15*td4 + 6*td5;
            double dp  = (30*td2 - 60*td3 + 30*td4)/m_traj_time;
            double ddp = (60*td - 180*td2 + 120*td3)/(m_traj_time*m_traj_time);
            m_x   = m_x_init + (m_x_final-m_x_init)*p;
            m_dx  = (m_x_final-m_x_init)*dp;
            m_ddx = (m_x_final-m_x_init)*ddp;
          }
          m_t += m_dt;
          return m_x;
        }

      };

      /** Endless sinusoidal trajectory generator.
       * The sinusoid is actually a cosine so that it starts with zero velocity.
       */
      class SinusoidTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      public:

        SinusoidTrajectoryGenerator(double dt, double traj_time, int size):
          AbstractTrajectoryGenerator(dt, traj_time, size)
        {}


        const Eigen::VectorXd& compute_next_point()
        {
          double f = 1.0/(2.0*m_traj_time);
          double two_pi_f   = 2*M_PI*f;
          double two_pi_f_t = two_pi_f*m_t;
          double p   = 0.5*(1.0-cos(two_pi_f_t));
          double dp  = 0.5*two_pi_f*sin(two_pi_f_t);
          double ddp = 0.5*two_pi_f*two_pi_f*cos(two_pi_f_t);
          m_x   = m_x_init + (m_x_final-m_x_init)*p;
          m_dx  = (m_x_final-m_x_init)*dp;
          m_ddx = (m_x_final-m_x_init)*ddp;

          m_t += m_dt;
          return m_x;
        }

        virtual bool isTrajectoryEnded(){ return false; }
      };

      /** Endless triangular trajectory generator.
       */
      class TriangleTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      protected:
        bool m_comingBack;
        double m_Tacc;

      public:

        TriangleTrajectoryGenerator(double dt, double traj_time, int size):
          AbstractTrajectoryGenerator(dt, traj_time, size),
          m_comingBack(false)
        {
          m_Tacc = 1.0;
        }

        bool set_acceleration_time(const double Tacc)
        {
          if(Tacc<0.0 || Tacc> 0.5*m_traj_time)
            return false;
          m_Tacc = Tacc;
          return true;
        }


        const Eigen::VectorXd& compute_next_point()
        {
          int way;
          double t=m_t;
          Eigen::VectorXd max_vel = (m_x_final-m_x_init)/(m_traj_time-m_Tacc);
          if  (m_t > m_traj_time)
          {
              way = -1;
              t=t-m_traj_time;
          }
          else
          {
              way = 1;
          }
          if (t < m_Tacc)
          {
              m_ddx = way*max_vel / m_Tacc;
              m_dx  = t/m_Tacc *way*max_vel;
          }
          else if (t > m_traj_time-m_Tacc)
          {
              m_ddx = -way*max_vel / m_Tacc;
              m_dx  = (m_traj_time-t)/m_Tacc *way*max_vel;
          }
          else
          {
               m_ddx = 0.0 * max_vel;
               m_dx  = way*max_vel;
          }
          m_x   += m_dt*m_dx;
          m_t += m_dt;
          if (m_t>=2*m_traj_time) m_t =m_t-2*m_traj_time;
          return m_x;
        }
        virtual bool isTrajectoryEnded(){ return false; }
      };

      /** Endless piece-wise constant acceleration trajectory generator.
       */
      class ConstantAccelerationTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      protected:
        bool m_is_accelerating; // if true then apply ddx0, otherwise apply -ddx0
        int m_counter;          // counter to decide when to switch from ddx0 to -ddx0
        int m_counter_max;      // value of the counter at which to switch from ddx0 to -ddx0
        Eigen::VectorXd m_ddx0; // acceleration to go halfway from x_init to x_final in half traj_time

      public:

        ConstantAccelerationTrajectoryGenerator(double dt, double traj_time, int size):
          AbstractTrajectoryGenerator(dt, traj_time, size),
          m_is_accelerating(true)
        {}

        virtual bool set_trajectory_time(double traj_time)
        {
          bool res = AbstractTrajectoryGenerator::set_trajectory_time(traj_time);
          if(!res)
            return false;
          m_counter_max = int(m_traj_time/m_dt);
          m_counter = int(0.5*m_counter_max);
          m_is_accelerating = true;
          return true;
        }

        const Eigen::VectorXd& compute_next_point()
        {
          m_ddx0 = 4.0*(m_x_final-m_x_init)/(m_traj_time*m_traj_time);

          if(m_counter==m_counter_max)
          {
            m_counter = 0;
            m_is_accelerating = ! m_is_accelerating;
          }
          m_counter += 1;

          m_ddx = m_ddx0;
          if(m_is_accelerating == false)
            m_ddx *= -1.0;

          m_x   += m_dt*m_dx + 0.5*m_dt*m_dt*m_ddx;
          m_dx  += m_dt*m_ddx;

          m_t += m_dt;
          return m_x;
        }

        virtual bool isTrajectoryEnded(){ return false; }
      };

      /** Linear chirp trajectory generator.
       *  A linear chirp is a sinusoid whose frequency is a linear function of time.
       *  In particular the frequency starts from a value f0 and it increases linearly
       *  up to a value f1. Then it goes back to f0 and the trajectory is ended.
       */
      class LinearChirpTrajectoryGenerator: public AbstractTrajectoryGenerator
      {
      protected:
        Eigen::VectorXd m_f0;          /// initial frequency
        Eigen::VectorXd m_f1;          /// final frequency

        /// Variables for temporary results
        Eigen::VectorXd m_k;              /// frequency first derivative
        Eigen::VectorXd m_f;              /// current frequency (i.e. time derivative of the phase over 2*pi)
        Eigen::VectorXd m_phi;            /// current phase
        Eigen::VectorXd m_phi_0;          /// phase shift for second half of trajectory
        Eigen::VectorXd m_p;
        Eigen::VectorXd m_dp;
        Eigen::VectorXd m_ddp;

      public:

        LinearChirpTrajectoryGenerator(double dt, double traj_time, int size):
          AbstractTrajectoryGenerator(dt, traj_time, size)
        {
          m_f0.setZero(size);
          m_f1.setZero(size);
          m_k.setZero(size);
          m_f.setZero(size);
          m_phi.setZero(size);
          m_phi_0.setZero(size);
          m_p.setZero(size);
          m_dp.setZero(size);
          m_ddp.setZero(size);
        }

        virtual bool set_initial_frequency(const Eigen::VectorXd& f0)
        {
          if(f0.size()!=m_f0.size())
            return false;
          m_f0 = f0;
          return true;
        }

        virtual bool set_initial_frequency(const double& f0)
        {
          if(1!=m_f0.size())
            return false;
          m_f0[0] = f0;
          return true;
        }

        virtual bool set_final_frequency(const Eigen::VectorXd& f1)
        {
          if(f1.size()!=m_f1.size())
            return false;
          m_f1 = f1;
          return true;
        }


        virtual bool set_final_frequency(const double& f1)
        {
          if(1!=m_f1.size())
            return false;
          m_f1[0] = f1;
          return true;
        }


        const Eigen::VectorXd& compute_next_point()
        {
          if(m_t==0.0)
          {
            m_k = 2.0*(m_f1-m_f0)/m_traj_time;
            m_phi_0 = M_PI*m_traj_time*(m_f0-m_f1);
          }

          if(m_t<0.5*m_traj_time)
          {
            m_f = m_f0 + m_k*m_t;
            m_phi = 2*M_PI*m_t*(m_f0 + 0.5*m_k*m_t);
          }
          else
          {
            m_f = m_f1 + m_k*(0.5*m_traj_time - m_t);
            m_phi = m_phi_0 + 2*M_PI*m_t*(m_f1 + 0.5*m_k*(m_traj_time - m_t));
          }
          m_p   = 0.5*(1.0-m_phi.array().cos());
          m_dp  = M_PI*m_f.array() * m_phi.array().sin();
          m_ddp = 2.0*M_PI*M_PI * m_f.array() * m_f.array() * m_phi.array().cos();

          m_x   = m_x_init.array() + (m_x_final.array()-m_x_init.array())*m_p.array();
          m_dx  = (m_x_final-m_x_init).array()*m_dp.array();
          m_ddx = (m_x_final-m_x_init).array()*m_ddp.array();

          m_t += m_dt;
          return m_x;
        }
      };



    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_trajectory_generators_H__
