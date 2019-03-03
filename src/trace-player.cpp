/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
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


#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <dynamic-graph/factory.h>
#include <sot/core/debug.hh>
#include <sot/torque_control/trace-player.hh>
#include <sot/torque_control/commands-helper.hh>


namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dynamicgraph = ::dynamicgraph;
      using namespace dynamicgraph;
      using namespace dynamicgraph::command;
      using namespace std;
      using namespace dynamicgraph::sot::torque_control;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef TracePlayer EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TracePlayer,
                                         "TracePlayer");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      TracePlayer::
      TracePlayer(const std::string& name)
        : Entity(name)
        ,CONSTRUCT_SIGNAL_OUT(trigger, int, sotNOSIGNAL)
      {
        Entity::signalRegistration(m_triggerSOUT);

        /* Commands. */
        addCommand("addOutputSignal",
                   makeCommandVoid2(*this, &TracePlayer::addOutputSignal,
                                    docCommandVoid2("Add a new output signal",
                                                    "Name of the text file where to read the data (string)",
                                                    "Name of the output signal (string)")));

        addCommand("playNext",
                   makeCommandVoid0(*this, &TracePlayer::playNext,
                                    docCommandVoid0("Update all the output signals.")));

        addCommand("rewind",
                   makeCommandVoid0(*this, &TracePlayer::rewind,
                                    docCommandVoid0("Rewind all the data.")));

        addCommand("clear",
                   makeCommandVoid0(*this, &TracePlayer::clear,
                                    docCommandVoid0("Clear all the output signals.")));
      }


      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(trigger, int)
      {
	std::string astr = toString(iter);
        playNext();
        return s;
      }


      /* --- COMMANDS ---------------------------------------------------------- */

      void TracePlayer::addOutputSignal(const string& fileName,
                                        const string& signalName)
      {
        // check there is no other signal with the same name
        if(m_outputSignals.find(signalName) != m_outputSignals.end())
          return SEND_MSG("It already exists a signal with name "+signalName, MSG_TYPE_ERROR);

        // read the text file
        std::ifstream datafile( fileName.c_str() );
        if(datafile.fail())
          return SEND_MSG("Error trying to read the file "+fileName, MSG_TYPE_ERROR);

        const unsigned int SIZE=1024;
        char buffer[SIZE];
        std::vector<double> newline;
        int nbLines = 0;
        bool firstIter = true;
	std::size_t size = -1;
        string fileNameShort = fileName.substr(1+fileName.find_last_of("/"));
        while( datafile.good() )
        {
          datafile.getline( buffer,SIZE );
          const std::size_t gcount = datafile.gcount();
          if( gcount>=SIZE )
            return SEND_MSG("Read error: line "+toString(nbLines)+
                            " too long in file "+fileNameShort, MSG_TYPE_ERROR);

          std::istringstream iss(buffer);
          newline.clear();
          double x;
          iss>>x; // discard the first value, which is the time step
          while( 1 )
          {
            iss>>x;
            if(iss.fail())
              break;
            newline.push_back(x);
          }

          if( newline.size()>0 )
          {
            if(firstIter)
              size = newline.size();
            else if(size!=newline.size())
            {
              SEND_MSG("In file "+fileNameShort+" nb of elements in each line changed from "+
                       toString(size)+" to "+toString(newline.size())+" at line "+
                       toString(nbLines), MSG_TYPE_WARNING);
              size = newline.size();
            }
            m_data[signalName].push_back( Eigen::Map<Vector>(&newline[0], newline.size()));
            nbLines++;
          }
        }
        SEND_MSG("Finished reading "+toString(nbLines)+" lines of "+toString(size)+
                 " elements from file "+fileNameShort, MSG_TYPE_INFO);
        m_dataPointers[signalName] = m_data[signalName].begin();

        // create a new output signal
        m_outputSignals[signalName] = new OutputSignalType(
                                        getClassName()+"("+getName()+
                                        ")::output(dynamicgraph::Vector)::"+
                                        signalName);

        // register the new signal
        m_triggerSOUT.addDependency(*m_outputSignals[signalName]);
        Entity::signalRegistration(*m_outputSignals[signalName]);

      }

      void TracePlayer::playNext()
      {
        typedef std::map<std::string, OutputSignalType* >::iterator it_type;
        for(it_type it=m_outputSignals.begin(); it!=m_outputSignals.end(); it++)
        {
          const string & signalName           = it->first;
          OutputSignalType * signal           = it->second;
          DataPointerType & dataPointer       = m_dataPointers[signalName];
          const DataHistoryType & dataSet     = m_data[signalName];

          if( dataPointer!=dataSet.end() )
            ++dataPointer;

          if( dataPointer==dataSet.end() )
            SEND_WARNING_STREAM_MSG("Reached end of dataset for signal "+signalName);
          else
            signal->setConstant(*dataPointer);
        }
      }

      void TracePlayer::rewind()
      {
        typedef std::map<std::string, DataPointerType>::iterator it_type;
        for(it_type it=m_dataPointers.begin(); it!=m_dataPointers.end(); it++)
        {
          const string & signalName           = it->first;
          DataPointerType & dataPointer       = it->second;
          const DataHistoryType & dataSet     = m_data[signalName];
          dataPointer = dataSet.begin();
        }
      }

      void TracePlayer::clear()
      {
        m_data.clear();
        m_dataPointers.clear();
        m_outputSignals.clear();
      }

      /* --- PROTECTED MEMBER METHODS ---------------------------------------------------------- */



      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */


      void TracePlayer::display(std::ostream& os) const
      {
        os << "TracePlayer "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

