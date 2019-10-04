// Copyright (C) 2014 CNRS-LAAS
// Author: Florent Lamiraux.
//
// This file is part of the hpp-manipulation-corba.
//
// hpp-manipulation-corba is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_RBPRM_CORBA_SERVER_HH
# define HPP_RBPRM_CORBA_SERVER_HH

# include <hpp/corba/template/server.hh>

# include <hpp/corbaserver/problem-solver-map.hh>
# include <hpp/corbaserver/rbprm/config.hh>
# include <hpp/corbaserver/server-plugin.hh>

namespace hpp {
  namespace rbprm {
    namespace impl {
      class RbprmBuilder;
    }
    class HPP_RBPRM_CORBA_DLLAPI Server : public corbaServer::ServerPlugin
    {
    public:
      Server (corbaServer::Server* parent);

      ~Server ();

      /// Start corba server
      /// Call hpp::corba::Server <impl::Problem>::startCorbaServer
      void startCorbaServer(const std::string& contextId,
                            const std::string& contextKind);

      std::string name () const;

    public:
      corba::Server <impl::RbprmBuilder>* rbprmBuilder_;
    }; // class Server
  } // namespace rbprm
} // namespace hpp

#endif // HPP_RBPRM_CORBA_SERVER_HH
