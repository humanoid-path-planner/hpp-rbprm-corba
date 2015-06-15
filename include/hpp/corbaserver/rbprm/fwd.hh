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

#ifndef HPP_MANIPULATION_CORBA_FWD_HH
# define HPP_MANIPULATION_CORBA_FWD_HH

# include <hpp/manipulation/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace impl {
      typedef manipulation::ProblemSolver ProblemSolver;
      class Server;
      typedef core::ConstraintSet ConstraintSet;
      typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
      typedef core::ConfigProjector ConfigProjector;
      typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_CORBA_FWD_HH
