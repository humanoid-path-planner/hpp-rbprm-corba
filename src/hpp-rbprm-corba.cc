// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/affordance/server.hh>
#include <hpp/corbaserver/rbprm/server.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/rbprm/planner/steering-dynamic.hh>
#include <hpp/rbprm/planner/steering-method-parabola.hh>
#include <hpp/rbprm/planner/dynamic-planner.hh>

typedef hpp::rbprm::Server RbprmServer;
typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::affordanceCorba::Server AffordanceServer;
int main (int argc, char* argv [])
{
    hpp::core::ProblemSolverPtr_t problemSolver (hpp::core::ProblemSolver::create());


    CorbaServer corbaServer (problemSolver, argc,
               const_cast<const char**> (argv), true);
		AffordanceServer affordanceServer (argc, const_cast<const char**> (argv),
																				true);
		affordanceServer.setProblemSolver (problemSolver);

		RbprmServer rbprmServer (argc, const_cast<const char**> (argv),
															true, "rbprmChild");
    rbprmServer.setProblemSolver (problemSolver);

    corbaServer.startCorbaServer ();
		affordanceServer.startCorbaServer ("hpp", "corbaserver",
																			"affordanceCorba", "affordance");
    rbprmServer.startCorbaServer ("hpp", "corbaserver",
                "rbprm");
    corbaServer.processRequest(true);
}
