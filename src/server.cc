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

#include <hpp/corbaserver/rbprm/server.hh>

#include <hpp/util/exception.hh>
#include "rbprmbuilder.impl.hh"
#include <hpp/corbaserver/server.hh>

namespace hpp {
namespace rbprm {
Server::Server(corbaServer::Server* server) : corbaServer::ServerPlugin(server), rbprmBuilder_(NULL) {}

Server::~Server() {
  if (rbprmBuilder_) delete rbprmBuilder_;
}

std::string Server::name() const { return "rbprm"; }

/// Start corba server
void Server::startCorbaServer(const std::string& contextId, const std::string& contextKind) {
  initializeTplServer (rbprmBuilder_, contextId, contextKind, name(), "builder");
  rbprmBuilder_->implementation ().setServer (this);
}

::CORBA::Object_ptr Server::servant(const std::string& name) const
{
  if (name == "builder") return rbprmBuilder_->implementation()._this();
  throw std::invalid_argument ("No servant " + name);
}
}  // namespace rbprm
}  // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::rbprm::Server)
