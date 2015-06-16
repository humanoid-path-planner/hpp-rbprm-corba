// Copyright (c) 2014 CNRS
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

#ifndef HPP_RBPRM_CORBA_BUILDER_IMPL_HH
# define HPP_RBPRM_CORBA_BUILDER_IMPL_HH

# include <hpp/corbaserver/rbprm/fwd.hh>
# include <hpp/core/problem-solver.hh>
# include "rbprmbuilder.hh"
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-shooter.hh>
# include <hpp/rbprm/rbprm-validation.hh>

namespace hpp {
  namespace rbprm {
    namespace impl {
      using CORBA::Short;

    struct BindShooter
    {
        BindShooter(const std::size_t shootLimit = 10000,
                    const std::size_t displacementLimit = 100)
            : shootLimit_(shootLimit)
            , displacementLimit_(displacementLimit) {}

        hpp::rbprm::RbPrmShooterPtr_t create (const hpp::model::DevicePtr_t& robot)
        {
            hpp::model::DevicePtr_t pRobot = robot;
            hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
            hpp::rbprm::RbPrmValidationPtr_t validator = hpp::rbprm::RbPrmValidation::create(robotcast);
            const hpp::core::ObjectVector_t& obstacles = problemSolver_->collisionObstacles();
            for(hpp::core::ObjectVector_t::const_iterator cit = obstacles.begin();
                cit != obstacles.end(); ++cit)
            {
                validator->addObstacle(*cit);
            }
            return hpp::rbprm::RbPrmShooter::create
                    (robotcast,problemSolver_->collisionObstacles(),validator,shootLimit_,displacementLimit_);
        }
        hpp::core::ProblemSolverPtr_t problemSolver_;
        std::size_t shootLimit_;
        std::size_t displacementLimit_;
    };

      class RbprmBuilder : public virtual POA_hpp::corbaserver::rbprm::RbprmBuilder
      {
        public:
        RbprmBuilder ();

        virtual void loadRobotRomModel (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        virtual void loadRobotCompleteModel (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        public:
        void SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver);

        private:
        /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
        core::ProblemSolverPtr_t problemSolver_;

        private:
        model::DevicePtr_t romDevice_;
        bool romLoaded_;
        BindShooter bindShooter_;
      }; // class RobotBuilder
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_RBPRM_CORBA_BUILDER_IMPL_HH
