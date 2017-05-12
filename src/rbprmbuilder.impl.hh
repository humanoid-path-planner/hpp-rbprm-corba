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
# include <hpp/core/path.hh>
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

        virtual void loadFullBodyRobotFromExistingRobot () throw (hpp::Error);


        virtual void setFilter(const hpp::Names_t& roms) throw (hpp::Error);
				virtual void setAffordanceFilter(const char* romName, const hpp::Names_t& affordances) throw (hpp::Error);
        virtual void boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error);


        virtual hpp::floatSeq* getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual hpp::floatSeq* getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getEffectorPosition(const char* limb, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::UShort getNumSamples(const char* limb) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeNodeIds(const char* limb) throw (hpp::Error);
        virtual double getSampleValue(const char* limb, const char* valueName, unsigned short sampleId) throw (hpp::Error);
        virtual double getEffectorDistance(unsigned short  state1, unsigned short  state2) throw (hpp::Error);

        virtual hpp::floatSeq* generateContacts(const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeq* generateGroundContact(const hpp::Names_t& contactLimbs) throw (hpp::Error);

        virtual hpp::floatSeq* getContactSamplesIds(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeqSeq* getContactSamplesProjected(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction,
                                                   unsigned short numSamples) throw (hpp::Error);

        virtual hpp::floatSeq* getSamplesIdsInOctreeNode(const char* limb,
                                                   double octreeNodeId) throw (hpp::Error);

        virtual void addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                             unsigned short samples, const char *heuristicName, double resolution, const char *contactType,
                             double disableEffectorCollision, double grasp) throw (hpp::Error);
        virtual void addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues,
                                     double disableEffectorCollision, double grasp) throw (hpp::Error);

        virtual void setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual void setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual hpp::floatSeq*  computeContactForConfig(const hpp::floatSeq& configuration, const char* limbNam) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPoints(unsigned short cId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPointsForLimb(unsigned short cId, const char* limbName) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolate(double timestep, double path, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getContactCone(unsigned short stateId, double friction) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getContactIntermediateCone(unsigned short stateId, double friction) throw (hpp::Error);
        virtual CORBA::Short generateComTraj(const hpp::floatSeqSeq& positions, const hpp::floatSeqSeq& velocities,
                                             const hpp::floatSeqSeq& accelerations, const double dt) throw (hpp::Error);

        virtual CORBA::Short straightPath(const hpp::floatSeqSeq& positions) throw (hpp::Error);
        virtual CORBA::Short generateCurveTraj(const hpp::floatSeqSeq& positions) throw (hpp::Error);
        virtual CORBA::Short generateCurveTrajParts(const hpp::floatSeqSeq& positions, const hpp::floatSeq& partitions) throw (hpp::Error);
        virtual CORBA::Short generateRootPath(const hpp::floatSeqSeq& rootPositions,
                                      const hpp::floatSeq& q1, const hpp::floatSeq& q2) throw (hpp::Error);
        virtual CORBA::Short limbRRT(double state1, double state2, unsigned short numOptimizations) throw (hpp::Error);
        virtual CORBA::Short limbRRTFromRootPath(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);
        virtual CORBA::Short configToPath(const hpp::floatSeqSeq& configs) throw (hpp::Error);
        virtual CORBA::Short comRRT(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);

        typedef core::PathPtr_t (*t_rrt)
            (RbPrmFullBodyPtr_t, core::ProblemPtr_t, const core::PathPtr_t,
             const  State &, const State &, const  std::size_t, const bool);

        hpp::floatSeq* rrt(t_rrt functor ,double state1,
                           unsigned short comTraj1, unsigned short comTraj2, unsigned short comTraj3,
                           unsigned short numOptimizations) throw (hpp::Error);

        virtual hpp::floatSeq* comRRTFromPos(double state1,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRT(double state1,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRTFromPath(double state1,
                                           unsigned short path,
                                           double path_from,
                                           double path_to,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations,
                                           const hpp::Names_t& trackedEffectors) throw (hpp::Error);
        virtual hpp::floatSeq* projectToCom(double state, const hpp::floatSeq& targetCom) throw (hpp::Error);
        virtual CORBA::Short createState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual hpp::floatSeq* getConfigAtState(unsigned short stateId) throw (hpp::Error);
        double projectStateToCOMEigen(unsigned short stateId, const model::Configuration_t& com_target)throw (hpp::Error);
        virtual double projectStateToCOM(unsigned short stateId, const hpp::floatSeq& com) throw (hpp::Error);
        virtual void saveComputedStates(const char* filepath) throw (hpp::Error);
        virtual void saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeBox(const char* limbName, double sampleId) throw (hpp::Error);
        virtual CORBA::Short  isLimbInContact(const char* limbName, double state) throw (hpp::Error);
        virtual CORBA::Short  isLimbInContactIntermediary(const char* limbName, double state) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::Short isConfigBalanced(const hpp::floatSeq& config, const hpp::Names_t& contactLimbs, double robustnessTreshold) throw (hpp::Error);
        virtual double isStateBalanced(unsigned short stateId) throw (hpp::Error);
        virtual void runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error);
        virtual hpp::floatSeq* runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error);
        virtual void dumpProfile(const char* logFile) throw (hpp::Error);

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
