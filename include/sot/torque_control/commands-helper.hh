/*
 * Copyright 2011, Nicolas Mansard, LAAS-CNRS
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

#ifndef __sot_torquecontrol_commands_helper_H__
#define __sot_torquecontrol_commands_helper_H__

#include <boost/function.hpp>

/* --- COMMON INCLUDE -------------------------------------------------- */
#include <dynamic-graph/command.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>


/* --- HELPER ---------------------------------------------------------- */
namespace dynamicgraph {
  namespace sot {
    namespace torquecontrol {
      using ::dynamicgraph::command::makeDirectGetter;
      using ::dynamicgraph::command::docDirectGetter;
      using ::dynamicgraph::command::makeDirectSetter;
      using ::dynamicgraph::command::docDirectSetter;
      using ::dynamicgraph::command::makeCommandVoid0;
      using ::dynamicgraph::command::docCommandVoid0;
      using ::dynamicgraph::command::makeCommandVoid1;
      using ::dynamicgraph::command::docCommandVoid1;
      using ::dynamicgraph::command::makeCommandVoid2;
      using ::dynamicgraph::command::docCommandVoid2;
      using ::dynamicgraph::command::makeCommandVerbose;
      using ::dynamicgraph::command::docCommandVerbose;
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

namespace dynamicgraph {
  namespace command {

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5>
    struct CommandVoid5
        : public Command
    {
      typedef boost::function<void(const T1&,const T2&,const T3&,const T4&,const T5&)> function_t;
      typedef boost::function<void(E*,const T1&,const T2&,const T3&,const T4&,const T5&)> memberFunction_t;
      typedef void (E::*memberFunction_ptr_t) (const T1&,const T2&,const T3&,const T4&,const T5&);

      CommandVoid5(E& entity, function_t function,
                   const std::string& docString)
        :Command(entity,
                 boost::assign::list_of
                 (ValueHelper<T1>::TypeID)
                 (ValueHelper<T2>::TypeID)
                 (ValueHelper<T3>::TypeID)
                 (ValueHelper<T4>::TypeID)
                 (ValueHelper<T5>::TypeID)
                 , docString)
        ,fptr(function)
      {}

    protected:
      virtual Value doExecute()
      {
        assert( getParameterValues().size() == 5 );
        T1 val1 = getParameterValues()[0].value();
        T2 val2 = getParameterValues()[1].value();
        T3 val3 = getParameterValues()[2].value();
        T4 val4 = getParameterValues()[3].value();
        T5 val5 = getParameterValues()[4].value();
        fptr(val1,val2,val3,val4,val5);
        return Value(); // void
      }
    private:
      function_t fptr;
    };


    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5>
    CommandVoid5<E,T1,T2,T3,T4,T5>*
    makeCommandVoid5(E& entity,
                     typename CommandVoid5<E,T1,T2,T3,T4,T5>::function_t function ,
                     const std::string& docString)
    {
      return new CommandVoid5<E,T1,T2,T3,T4,T5>( entity,function,docString );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5>
    CommandVoid5<E,T1,T2,T3,T4,T5>*
    makeCommandVoid5(E& entity,
                     boost::function<void(E*,const T1&,const T2&,const T3&,const T4&,const T5&)> function,
                     const std::string& docString)
    {
      return new CommandVoid5<E,T1,T2,T3,T4,T5>( entity,
                                                    boost::bind(function,&entity,_1,_2,_3,_4,_5),docString );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5>
    CommandVoid5<E,T1,T2,T3,T4,T5>*
    makeCommandVoid5(E& entity,
                     void (E::*function) (const T1&,const T2&,const T3&,const T4&,const T5&),
                     const std::string& docString)
    {
      return new CommandVoid5<E,T1,T2,T3,T4,T5>( entity,
                                                    boost::bind(function,&entity,_1,_2,_3,_4,_5),
                                                    docString );
      return NULL;
    }

    inline std::string docCommandVoid5( const std::string& doc,
                                        const std::string& type1,
                                        const std::string& type2,
                                        const std::string& type3,
                                        const std::string& type4,
                                        const std::string& type5)
    {
      return (std::string("\n")+doc+"\n\n"
              +"Input:\n - A "+type1+".\n"
              +"Input:\n - A "+type2+".\n"
              +"Input:\n - A "+type3+".\n"
              +"Input:\n - A "+type4+".\n"
              +"Input:\n - A "+type5+".\n"
              +"Void return.\n\n" );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6 >
    struct CommandVoid6
        : public Command
    {
      typedef boost::function<void(const T1&,const T2&,const T3&,const T4&,const T5&,const T6&)> function_t;
      typedef boost::function<void(E*,const T1&,const T2&,const T3&,const T4&,const T5&,const T6&)> memberFunction_t;
      typedef void (E::*memberFunction_ptr_t) (const T1&,const T2&,const T3&,const T4&,const T5&,const T6&);

      CommandVoid6(E& entity, function_t function,
                   const std::string& docString)
        :Command(entity,
                 boost::assign::list_of
                 (ValueHelper<T1>::TypeID)
                 (ValueHelper<T2>::TypeID)
                 (ValueHelper<T3>::TypeID)
                 (ValueHelper<T4>::TypeID)
                 (ValueHelper<T5>::TypeID)
                 (ValueHelper<T6>::TypeID)
                 , docString)
        ,fptr(function)
      {}

    protected:
      virtual Value doExecute()
      {
        assert( getParameterValues().size() == 6 );
        T1 val1 = getParameterValues()[0].value();
        T2 val2 = getParameterValues()[1].value();
        T3 val3 = getParameterValues()[2].value();
        T4 val4 = getParameterValues()[3].value();
        T5 val5 = getParameterValues()[4].value();
        T6 val6 = getParameterValues()[5].value();
        fptr(val1,val2,val3,val4,val5,val6);
        return Value(); // void
      }
    private:
      function_t fptr;
    };


    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6 >
    CommandVoid6<E,T1,T2,T3,T4,T5,T6>*
    makeCommandVoid6(E& entity,
                     typename CommandVoid6<E,T1,T2,T3,T4,T5,T6>::function_t function ,
                     const std::string& docString)
    {
      return new CommandVoid6<E,T1,T2,T3,T4,T5,T6>( entity,function,docString );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6 >
    CommandVoid6<E,T1,T2,T3,T4,T5,T6>*
    makeCommandVoid6(E& entity,
                     boost::function<void(E*,const T1&,const T2&,const T3&,const T4&,const T5&,const T6&)> function,
                     const std::string& docString)
    {
      return new CommandVoid6<E,T1,T2,T3,T4,T5,T6>( entity,
                                                    boost::bind(function,&entity,_1,_2,_3,_4,_5,_6),docString );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6 >
    CommandVoid6<E,T1,T2,T3,T4,T5,T6>*
    makeCommandVoid6(E& entity,
                     void (E::*function) (const T1&,const T2&,const T3&,const T4&,const T5&,const T6&),
                     const std::string& docString)
    {
      return new CommandVoid6<E,T1,T2,T3,T4,T5,T6>( entity,
                                                    boost::bind(function,&entity,_1,_2,_3,_4,_5,_6),
                                                    docString );
      return NULL;
    }

    inline std::string docCommandVoid6( const std::string& doc,
                                        const std::string& type1,
                                        const std::string& type2,
                                        const std::string& type3,
                                        const std::string& type4,
                                        const std::string& type5,
                                        const std::string& type6)
    {
      return (std::string("\n")+doc+"\n\n"
              +"Input:\n - A "+type1+".\n"
              +"Input:\n - A "+type2+".\n"
              +"Input:\n - A "+type3+".\n"
              +"Input:\n - A "+type4+".\n"
              +"Input:\n - A "+type5+".\n"
              +"Input:\n - A "+type6+".\n"
              +"Void return.\n\n" );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6,typename T7 >
    struct CommandVoid7
        : public Command
    {
      typedef boost::function<void(const T1&,const T2&,const T3&,const T4&,const T5&,const T6&,const T7&)> function_t;
      typedef boost::function<void(E*,const T1&,const T2&,const T3&,const T4&,const T5&,const T6&,const T7&)> memberFunction_t;
      typedef void (E::*memberFunction_ptr_t) (const T1&,const T2&,const T3&,const T4&,const T5&,const T6&,const T7&);

      CommandVoid7(E& entity, function_t function,
                   const std::string& docString)
        :Command(entity,
                 boost::assign::list_of
                 (ValueHelper<T1>::TypeID)
                 (ValueHelper<T2>::TypeID)
                 (ValueHelper<T3>::TypeID)
                 (ValueHelper<T4>::TypeID)
                 (ValueHelper<T5>::TypeID)
                 (ValueHelper<T6>::TypeID)
                 (ValueHelper<T7>::TypeID)
                 , docString)
        ,fptr(function)
      {}

    protected:
      virtual Value doExecute()
      {
        assert( getParameterValues().size() == 7 );
        T1 val1 = getParameterValues()[0].value();
        T2 val2 = getParameterValues()[1].value();
        T3 val3 = getParameterValues()[2].value();
        T4 val4 = getParameterValues()[3].value();
        T5 val5 = getParameterValues()[4].value();
        T6 val6 = getParameterValues()[5].value();
        T7 val7 = getParameterValues()[6].value();
        fptr(val1,val2,val3,val4,val5,val6,val7);
        return Value(); // void
      }
    private:
      function_t fptr;
    };


    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6,typename T7 >
    CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>*
    makeCommandVoid7(E& entity,
                     typename CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>::function_t function ,
                     const std::string& docString)
    {
      return new CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>( entity,function,docString );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6,typename T7 >
    CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>*
    makeCommandVoid7(E& entity,
                     boost::function<void(E*,const T1&,const T2&,const T3&,const T4&,const T5&,const T6&,const T7&)> function,
                     const std::string& docString)
    {
      return new CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>( entity,
                                                    boost::bind(function,&entity,_1,_2,_3,_4,_5,_6,_7),docString );
    }

    template <class E,typename T1,typename T2,typename T3,typename T4,typename T5,typename T6,typename T7 >
    CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>*
    makeCommandVoid7(E& entity,
                     void (E::*function) (const T1&,const T2&,const T3&,const T4&,const T5&,const T6&,const T7&),
                     const std::string& docString)
    {
      return new CommandVoid7<E,T1,T2,T3,T4,T5,T6,T7>( entity,
                                                          boost::bind(function,&entity,_1,_2,_3,_4,_5,_6,_7),
                                                          docString );
      return NULL;
    }

    inline std::string docCommandVoid7( const std::string& doc,
                                        const std::string& type1,
                                        const std::string& type2,
                                        const std::string& type3,
                                        const std::string& type4,
                                        const std::string& type5,
                                        const std::string& type6,
                                        const std::string& type7)
    {
      return (std::string("\n")+doc+"\n\n"
              +"Input:\n - A "+type1+".\n"
              +"Input:\n - A "+type2+".\n"
              +"Input:\n - A "+type3+".\n"
              +"Input:\n - A "+type4+".\n"
              +"Input:\n - A "+type5+".\n"
              +"Input:\n - A "+type6+".\n"
              +"Input:\n - A "+type7+".\n"
              +"Void return.\n\n" );
    }


  } // namespace command
} // namespace dynamicgraph



#endif // __sot_torquecontrol_commands_helper_H__


