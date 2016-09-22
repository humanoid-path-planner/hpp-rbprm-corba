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

# include <hpp/core/problem-solver.hh>
# include "rbprmbuilder.hh"
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-shooter.hh>
# include <hpp/rbprm/rbprm-validation.hh>
# include <hpp/rbprm/sampling/analysis.hh>
# include <hpp/core/collision-path-validation-report.hh>
# include <hpp/core/problem-solver.hh>
# include <hpp/core/discretized-collision-checking.hh>
# include <hpp/core/straight-path.hh>

#include <hpp/fcl/BVH/BVH_model.h>

namespace hpp {
  namespace rbprm {
    namespace impl {
      using CORBA::Short;
			typedef std::map<std::string, std::vector<boost::shared_ptr<model::CollisionObject> > > affMap_t;

    struct BindShooter
    {
        BindShooter(const std::size_t shootLimit = 10000,
                    const std::size_t displacementLimit = 1000)
            : shootLimit_(shootLimit)
            , displacementLimit_(displacementLimit) {}

        hpp::rbprm::RbPrmShooterPtr_t create (const hpp::model::DevicePtr_t& robot)
        {
            hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
		        if (affMap_.empty ()) {
    	        throw hpp::Error ("No affordances found. Unable to create shooter object.");
      		  }
            rbprm::RbPrmShooterPtr_t shooter = hpp::rbprm::RbPrmShooter::create
                    (robotcast, problemSolver_->problem ()->collisionObstacles(), affMap_,
										romFilter_,affFilter_,shootLimit_,displacementLimit_);
            if(!so3Bounds_.empty())
                shooter->BoundSO3(so3Bounds_);
            return shooter;
        }

        hpp::core::PathValidationPtr_t createPathValidation (const hpp::model::DevicePtr_t& robot, const hpp::model::value_type& val)
        {
            hpp::model::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::model::RbPrmDevice>(robot);
						affMap_ = problemSolver_->map
							<std::vector<boost::shared_ptr<model::CollisionObject> > > ();
		        if (affMap_.empty ()) {
    	        throw hpp::Error ("No affordances found. Unable to create Path Validaton object.");
      		  }
            hpp::rbprm::RbPrmValidationPtr_t validation
							(hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
            hpp::core::DiscretizedCollisionCheckingPtr_t collisionChecking = hpp::core::DiscretizedCollisionChecking::create(robot,val);
            collisionChecking->add (validation);
            return collisionChecking;
        }

        hpp::core::ProblemSolverPtr_t problemSolver_;
        std::vector<std::string> romFilter_;
        std::map<std::string, std::vector<std::string> > affFilter_;
        std::size_t shootLimit_;
        std::size_t displacementLimit_;
        std::vector<double> so3Bounds_;
				affMap_t affMap_;
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


        virtual void loadFullBodyRobot (const char* robotName,
                 const char* rootJointType,
                 const char* packageName,
                 const char* modelName,
                 const char* urdfSuffix,
                 const char* srdfSuffix) throw (hpp::Error);

        virtual void setFilter(const hpp::Names_t& roms) throw (hpp::Error);
				virtual void setAffordanceFilter(const char* romName, const hpp::Names_t& affordances) throw (hpp::Error);
        virtual std::vector<Eigen::Vector3d> getContactPoints (const char* limbname, model::T_Rom& romDevices,
                const core::ProblemSolverPtr_t& problemSolver,const unsigned int stateId, Eigen::VectorXd& contactPose);
        virtual hpp::floatSeqSeq * getDebugContactPoints (const char* limbname,
                unsigned short stateId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getPointsOnCurve (const hpp::floatSeq& radii,
                const hpp::floatSeq& trafo) throw (hpp::Error);
        virtual hpp::floatSeq * getReachableContactArea (const char* limbname,
                CORBA::Boolean ellipse, hpp::floatSeq_out pose, unsigned short stateId) throw (hpp::Error);
        virtual std::vector<double> getApproximatedEffector (const char* limbname, bool ellipse);
        virtual hpp::floatSeq* getApproximatedEffectorDebug (const char* limbname, 
                unsigned short stateId, CORBA::Boolean ellipse, floatSeq_out pose) throw (hpp::Error);
        virtual void boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error);


        virtual hpp::floatSeq* getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual hpp::floatSeq* getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual CORBA::UShort getNumSamples(const char* limb) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeNodeIds(const char* limb) throw (hpp::Error);
        virtual double getSampleValue(const char* limb, const char* valueName, unsigned short sampleId) throw (hpp::Error);

        virtual hpp::floatSeq* generateContacts(const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeq* getContactSamplesIds(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeq* getSamplesIdsInOctreeNode(const char* limb,
                                                   double octreeNodeId) throw (hpp::Error);

        virtual void addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                             unsigned short samples, const char *heuristicName, double resolution, const char *contactType,
                             double disableEffectorCollision) throw (hpp::Error);
        virtual void addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues,
                                     double disableEffectorCollision) throw (hpp::Error);

        virtual void setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual void setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolate(double timestep, double path, double robustnessTreshold) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold) throw (hpp::Error);
        virtual void interpolateBetweenStates(double state1, double state2, unsigned short numOptimizations) throw (hpp::Error);
        virtual void interpolateBetweenStatesFromPath(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);
        virtual void saveComputedStates(const char* filepath) throw (hpp::Error);
        virtual void saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeBox(const char* limbName, double sampleId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::Short isConfigBalanced(const hpp::floatSeq& config, const hpp::Names_t& contactLimbs, double robustnessTreshold) throw (hpp::Error);
        virtual void runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error);
        virtual hpp::floatSeq* runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error);

        public:
        void SetProblemSolver (hpp::core::ProblemSolverPtr_t problemSolver);

        private:
        /// \brief Pointer to hppPlanner object of hpp::corbaServer::Server.
        core::ProblemSolverPtr_t problemSolver_;

        private:
        model::T_Rom romDevices_;
        rbprm::RbPrmFullBodyPtr_t fullBody_;
        bool romLoaded_;
        bool fullBodyLoaded_;
        BindShooter bindShooter_;
        rbprm::State startState_;
        rbprm::State endState_;
        std::vector<rbprm::State> lastStatesComputed_;
        rbprm::T_StateFrame lastStatesComputedTime_;
        sampling::AnalysisFactory* analysisFactory_;
      }; // class RobotBuilder
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_RBPRM_CORBA_BUILDER_IMPL_HH
