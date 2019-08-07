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
# include "hpp/corbaserver/rbprm/rbprmbuilder-idl.hh"
# include <hpp/rbprm/rbprm-device.hh>
# include <hpp/rbprm/rbprm-fullbody.hh>
# include <hpp/rbprm/rbprm-shooter.hh>
# include <hpp/rbprm/rbprm-validation.hh>
# include <hpp/rbprm/sampling/analysis.hh>
# include <hpp/core/collision-path-validation-report.hh>
# include <hpp/core/problem-solver.hh>
# include <hpp/core/straight-path.hh>
# include <hpp/core/problem.hh>
#include <hpp/corbaserver/affordance/server.hh>
# include <hpp/corbaserver/problem-solver-map.hh>
# include <hpp/rbprm/rbprm-path-validation.hh>
# include <hpp/fcl/BVH/BVH_model.h>
# include <hpp/core/config-validations.hh>
#include <hpp/rbprm/dynamic/dynamic-path-validation.hh>
# include "hpp/corbaserver/fwd.hh"
# include "hpp/corbaserver/rbprm/server.hh"

namespace hpp {
  namespace rbprm {
    namespace impl {
      using CORBA::Short;

    typedef hpp::core::Container <hpp::core::AffordanceObjects_t> affMap_t;

    struct BindShooter
    {
        BindShooter(const std::size_t shootLimit = 10000,
                    const std::size_t displacementLimit = 100)
            : shootLimit_(shootLimit)
            , displacementLimit_(displacementLimit) {}

        hpp::rbprm::RbPrmShooterPtr_t create (/*const hpp::pinocchio::DevicePtr_t& robot,*/ const hpp::core::Problem& problem)
        {
            affMap_ = problemSolver_->affordanceObjects;
            hpp::pinocchio::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(problem.robot());
                if (affMap_.map.empty ()) {
    	        throw hpp::Error ("No affordances found. Unable to create shooter object.");
      		  }
            rbprm::RbPrmShooterPtr_t shooter = hpp::rbprm::RbPrmShooter::create
                    (robotcast, problemSolver_->problem ()->collisionObstacles(), affMap_,
										romFilter_,affFilter_,shootLimit_,displacementLimit_);
            if(!so3Bounds_.empty())
                shooter->BoundSO3(so3Bounds_);
            shooter->sampleExtraDOF(problem.getParameter("ConfigurationShooter/sampleExtraDOF").boolValue());
            shooter->ratioWeighted(problem.getParameter("RbprmShooter/ratioWeighted").floatValue ());
            return shooter;
        }

        hpp::core::PathValidationPtr_t createPathValidation (const hpp::pinocchio::DevicePtr_t& robot, const hpp::pinocchio::value_type& val)
        {
            hpp::pinocchio::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
                        affMap_ = problemSolver_->affordanceObjects;
                if (affMap_.map.empty ()) {
    	        throw hpp::Error ("No affordances found. Unable to create Path Validaton object.");
      		  }
            hpp::rbprm::RbPrmValidationPtr_t validation
              (hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
            hpp::rbprm::RbPrmPathValidationPtr_t collisionChecking = hpp::rbprm::RbPrmPathValidation::create(robot,val);
            collisionChecking->add (validation);
            problemSolver_->problem()->configValidation(core::ConfigValidations::create ());
            problemSolver_->problem()->configValidations()->add(validation);
            return collisionChecking;
        }

        hpp::core::PathValidationPtr_t createDynamicPathValidation (const hpp::pinocchio::DevicePtr_t& robot, const hpp::pinocchio::value_type& val)
        {
          hpp::pinocchio::RbPrmDevicePtr_t robotcast = boost::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
          affMap_ = problemSolver_->affordanceObjects;
          if (affMap_.map.empty ()) {
            throw hpp::Error ("No affordances found. Unable to create Path Validaton object.");
          }
          hpp::rbprm::RbPrmValidationPtr_t validation
            (hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
          hpp::rbprm::DynamicPathValidationPtr_t collisionChecking = hpp::rbprm::DynamicPathValidation::create(robot,val);
          collisionChecking->add (validation);
          problemSolver_->problem()->configValidation(core::ConfigValidations::create ());
          problemSolver_->problem()->configValidations()->add(validation);
          // build the dynamicValidation :
          double sizeFootX,sizeFootY,mass,mu;
          bool rectangularContact;
          sizeFootX = problemSolver_->problem()->getParameter (std::string("DynamicPlanner/sizeFootX")).floatValue()/2.;
          sizeFootY = problemSolver_->problem()->getParameter (std::string("DynamicPlanner/sizeFootY")).floatValue()/2.;
          if(sizeFootX > 0. && sizeFootY > 0.)
            rectangularContact = 1;
          else
            rectangularContact = 0;
          mass = robot->mass();
          mu = problemSolver_->problem()->getParameter (std::string("DynamicPlanner/friction")).floatValue();
          hppDout(notice,"mu define in python : "<<mu);
          DynamicValidationPtr_t dynamicVal = DynamicValidation::create(rectangularContact,sizeFootX,sizeFootY,mass,mu,robot);
          collisionChecking->addDynamicValidator(dynamicVal);

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

    class FullBodyMap {
      public:
        typedef std::map<std::string, rbprm::RbPrmFullBodyPtr_t> fMap_t;

        std::string selected_;
        fMap_t map_;

        FullBodyMap (const std::string& name = "None") :
          selected_ (name)
          {
            //map_[selected_] = init;
          }

        rbprm::RbPrmFullBodyPtr_t operator-> () {
          return selected();
        }
        operator rbprm::RbPrmFullBodyPtr_t () {
          return selected();
        }
        rbprm::RbPrmFullBodyPtr_t selected () {
          return map_[selected_];
        }
        bool has (const std::string& name) const
        {
          // ProblemMap_t::const_iterator it = map_.find (name);
          // return it != map_.end ();
          return map_.end() != map_.find (name);
        }
        template <typename ReturnType> ReturnType keys () const
        {
          ReturnType l;
          for (fMap_t::const_iterator it = map_.begin ();
              it != map_.end (); ++it)
            l.push_back (it->first);
          return l;
        }
    };

      class RbprmBuilder : public virtual POA_hpp::corbaserver::rbprm::RbprmBuilder
      {
        public:
        RbprmBuilder ();

        void setServer (Server* server)
        {
          server_ = server;
        }

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
                 const char* srdfSuffix,
                 const char* selectedProblem) throw (hpp::Error);

        virtual void loadFullBodyRobotFromExistingRobot () throw (hpp::Error);

        void setStaticStability(const bool staticStability) throw (hpp::Error);

        void setReferenceConfig(const hpp::floatSeq &referenceConfig) throw (hpp::Error);
        void setPostureWeights(const hpp::floatSeq &postureWeights) throw (hpp::Error);
        void setReferenceEndEffector(const char* romName, const hpp::floatSeq &ref) throw(hpp::Error);
        void usePosturalTaskContactCreation(const bool usePosturalTaskContactCreation) throw (hpp::Error);

        virtual void setFilter(const hpp::Names_t& roms) throw (hpp::Error);
				virtual void setAffordanceFilter(const char* romName, const hpp::Names_t& affordances) throw (hpp::Error);
        virtual void boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error);


        virtual hpp::floatSeq* getSampleConfig(const char* limb, unsigned int sampleId) throw (hpp::Error);
        virtual hpp::floatSeq* getSamplePosition(const char* limb, unsigned int sampleId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getEffectorPosition(const char* limb, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::UShort getNumSamples(const char* limb) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeNodeIds(const char* limb) throw (hpp::Error);
        virtual double getSampleValue(const char* limb, const char* valueName, unsigned int sampleId) throw (hpp::Error);
        virtual double getEffectorDistance(unsigned short  state1, unsigned short  state2) throw (hpp::Error);

        rbprm::State generateContacts_internal(const hpp::floatSeq& configuration,
          const hpp::floatSeq& direction,const hpp::floatSeq& acceleration, const double robustnessThreshold ) throw (hpp::Error);
        virtual hpp::floatSeq* generateContacts(const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& direction, const hpp::floatSeq& acceleration, const double robustnessThreshold) throw (hpp::Error);
        virtual CORBA::Short generateStateInContact(const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& direction, const hpp::floatSeq& acceleration, const double robustnessThreshold) throw (hpp::Error);

        virtual hpp::floatSeq* generateGroundContact(const hpp::Names_t& contactLimbs) throw (hpp::Error);

        virtual hpp::floatSeq* getContactSamplesIds(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction) throw (hpp::Error);

        virtual hpp::floatSeqSeq* getContactSamplesProjected(const char* limb,
                                                   const hpp::floatSeq& configuration,
                                                   const hpp::floatSeq& direction,
                                                   unsigned short numSamples) throw (hpp::Error);

        virtual short generateContactState(::CORBA::UShort  currentState, const char*  name,  const ::hpp::floatSeq& direction)  throw (hpp::Error);


        virtual hpp::floatSeq* getSamplesIdsInOctreeNode(const char* limb,
                                                   double octreeNodeId) throw (hpp::Error);

        virtual void addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                             unsigned int samples, const char *heuristicName, double resolution, const char *contactType,
                             double disableEffectorCollision, double grasp,const hpp::floatSeq& limbOffset,const char* kinematicConstraintsPath, double kinematicConstraintsMin) throw (hpp::Error);
        virtual void addNonContactingLimb(const char* id, const char* limb, const char* effector, unsigned int samples) throw (hpp::Error);

        virtual void addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues,
                                     double disableEffectorCollision, double grasp) throw (hpp::Error);

        virtual void setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual void setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual void setStartStateId(unsigned short stateId) throw (hpp::Error);
        virtual void setEndStateId(unsigned short stateId) throw (hpp::Error);
        virtual hpp::floatSeq*  computeContactForConfig(const hpp::floatSeq& configuration, const char* limbNam) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPoints(unsigned short cId) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPointsAtState(unsigned short cId, unsigned short isIntermediate) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPointsForLimb(unsigned short cId, const char* limbName) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeContactPointsAtStateForLimb(unsigned short cId, unsigned short isIntermediate, const char* limbName) throw (hpp::Error);
        virtual hpp::floatSeqSeq* computeCenterOfContactAtStateForLimb(unsigned short cId, unsigned short isIntermediate, const char *limbName) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolate(double timestep, double path, double robustnessTreshold, unsigned short filterStates, bool testReachability, bool quasiStatic, bool erasePreviousStates) throw (hpp::Error);
        virtual hpp::floatSeqSeq* interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold, unsigned short filterStates, bool testReachability, bool quasiStatic, bool erasePreviousStates) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getContactCone(unsigned short stateId, double friction) throw (hpp::Error);
        virtual hpp::floatSeqSeq* getContactIntermediateCone(unsigned short stateId, double friction) throw (hpp::Error);
        virtual CORBA::Short generateComTraj(const hpp::floatSeqSeq& positions, const hpp::floatSeqSeq& velocities,
                                             const hpp::floatSeqSeq& accelerations, const double dt) throw (hpp::Error);

        virtual CORBA::Short straightPath(const hpp::floatSeqSeq& positions) throw (hpp::Error);
        virtual CORBA::Short generateCurveTraj(const hpp::floatSeqSeq& positions) throw (hpp::Error);
        virtual CORBA::Short generateCurveTrajParts(const hpp::floatSeqSeq& positions, const hpp::floatSeq& partitions) throw (hpp::Error);
        virtual CORBA::Short generateRootPath(const hpp::floatSeqSeq& rootPositions,
                                      const hpp::floatSeq& q1, const hpp::floatSeq& q2) throw (hpp::Error);
        virtual CORBA::Short limbRRT(unsigned short state1, unsigned short state2, unsigned short numOptimizations) throw (hpp::Error);
        virtual CORBA::Short limbRRTFromRootPath(unsigned short state1, unsigned short state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);
        virtual CORBA::Short configToPath(const hpp::floatSeqSeq& configs) throw (hpp::Error);
        virtual CORBA::Short comRRT(unsigned short state1, unsigned short state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error);

        typedef core::PathPtr_t (*t_rrt)
            (RbPrmFullBodyPtr_t, core::ProblemSolverPtr_t, const core::PathPtr_t,
             const  State &, const State &, const  std::size_t, const bool);

        hpp::floatSeq* rrt(t_rrt functor ,unsigned short state1,unsigned short state2,
                           unsigned short comTraj1, unsigned short comTraj2, unsigned short comTraj3,
                           unsigned short numOptimizations) throw (hpp::Error);

        virtual hpp::floatSeq* comRRTFromPos(unsigned short state1,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* comRRTFromPosBetweenState(unsigned short state1,unsigned short state2,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRTFromPosBetweenState(unsigned short state1,unsigned short state2,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRT(unsigned short state1,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRTFromPath(unsigned short state1,
                                           unsigned short path,
                                           double path_from,
                                           double path_to,
                                           unsigned short comTraj1,
                                           unsigned short comTraj2,
                                           unsigned short comTraj3,
                                           unsigned short numOptimizations,
                                           const hpp::Names_t& trackedEffectors) throw (hpp::Error);
        virtual hpp::floatSeq* rrtOnePhase(t_rrt functor,unsigned short state1,unsigned short state2,
                                           unsigned short comTraj,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* effectorRRTOnePhase(unsigned short state1,unsigned short state2,
                                           unsigned short comTraj,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeq* comRRTOnePhase(unsigned short state1, unsigned short state2,
                                           unsigned short comTraj,
                                           unsigned short numOptimizations) throw (hpp::Error);
        virtual hpp::floatSeqSeq* generateEffectorBezierArray(unsigned short state1,unsigned short state2,
                                           unsigned short comTraj,
                                           unsigned short numOptimizations) throw (hpp::Error);

        virtual CORBA::Short generateEndEffectorBezier(unsigned short state1, unsigned short state2,
        unsigned short cT) throw (hpp::Error);

        virtual hpp::floatSeq* projectToCom(unsigned short state, const hpp::floatSeq& targetCom, unsigned short max_num_sample) throw (hpp::Error);
        virtual CORBA::Short createState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error);
        virtual hpp::floatSeq* getConfigAtState(unsigned short stateId) throw (hpp::Error);
        virtual double setConfigAtState(unsigned short stateId, const hpp::floatSeq& config) throw (hpp::Error);
        double projectStateToCOMEigen(State& s, const pinocchio::Configuration_t& com_target, unsigned short maxNumeSamples)throw (hpp::Error);
        double projectStateToCOMEigen(unsigned short stateId, const pinocchio::Configuration_t& com_target, unsigned short maxNumeSamples)throw (hpp::Error);

        virtual double projectStateToCOM(unsigned short stateId, const hpp::floatSeq& com, unsigned short max_num_sample) throw (hpp::Error);
        virtual double projectStateToRoot(unsigned short stateId, const hpp::floatSeq& root) throw (hpp::Error);
        virtual void saveComputedStates(const char* filepath) throw (hpp::Error);
        virtual void saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeBox(const char* limbName, double sampleId) throw (hpp::Error);
        virtual CORBA::Short  isLimbInContact(const char* limbName, unsigned short state) throw (hpp::Error);
        virtual CORBA::Short  isLimbInContactIntermediary(const char* limbName, unsigned short state) throw (hpp::Error);
        virtual CORBA::Short  computeIntermediary(unsigned short state1, unsigned short state2) throw (hpp::Error);
        virtual CORBA::Short  getNumStates() throw (hpp::Error);
        virtual hpp::floatSeqSeq* getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual hpp::floatSeq* getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error);
        virtual CORBA::Short isConfigBalanced(const hpp::floatSeq& config, const hpp::Names_t& contactLimbs, double robustnessTreshold,const hpp::floatSeq& CoM) throw (hpp::Error);
        virtual double isStateBalanced(unsigned short stateId) throw (hpp::Error);
        virtual void runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error);
        virtual hpp::floatSeq* runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error);
        virtual hpp::floatSeq* evaluateConfig(const hpp::floatSeq& configuration, const hpp::floatSeq &direction) throw (hpp::Error);
        virtual void dumpProfile(const char* logFile) throw (hpp::Error);
        virtual double getTimeAtState(unsigned short stateId)throw (hpp::Error);
        virtual Names_t* getContactsVariations(unsigned short stateIdFrom,unsigned short stateIdTo )throw (hpp::Error);
        virtual Names_t* getAllLimbsNames()throw (hpp::Error);
        virtual CORBA::Short addNewContact(unsigned short stateId, const char* limbName,
                                            const hpp::floatSeq& position, const hpp::floatSeq& normal, unsigned short max_num_sample, bool lockOtherJoints) throw (hpp::Error);
        virtual CORBA::Short removeContact(unsigned short stateId, const char* limbName) throw (hpp::Error);
        virtual hpp::floatSeq* computeTargetTransform(const char* limbName, const hpp::floatSeq& configuration, const hpp::floatSeq& p, const hpp::floatSeq& n) throw (hpp::Error);
        virtual Names_t* getEffectorsTrajectoriesNames(unsigned short pathId)throw (hpp::Error);
        virtual hpp::floatSeqSeqSeq* getEffectorTrajectoryWaypoints(unsigned short pathId,const char* effectorName)throw (hpp::Error);
        virtual hpp::floatSeqSeq* getPathAsBezier(unsigned short pathId)throw (hpp::Error);


        virtual bool areKinematicsConstraintsVerified(const hpp::floatSeq &point)throw (hpp::Error);
        virtual bool areKinematicsConstraintsVerifiedForState(unsigned short stateId,const hpp::floatSeq &point)throw (hpp::Error);
        virtual hpp::floatSeq *isReachableFromState(unsigned short stateFrom,unsigned short stateTo,const bool useIntermediateState)throw (hpp::Error);
        virtual hpp::floatSeq* isDynamicallyReachableFromState(unsigned short stateFrom, unsigned short stateTo, bool addPathPerPhase, const hpp::floatSeq &timings, short numPointPerPhase )throw (hpp::Error);


        void selectFullBody (const char* name) throw (hpp::Error)
        {
          std::string psName (name);
          bool has = fullBodyMap_.has (psName);
          if (!has)
              throw hpp::Error("unknown fullBody Problem");
          fullBodyMap_.selected_ = psName;
        }

        public:
        void SetProblemSolverMap (hpp::corbaServer::ProblemSolverMapPtr_t psMap);
        void initNewProblemSolver ();

        private:
        core::ProblemSolverPtr_t problemSolver()
        {
            return server_->problemSolver();
        }

        Server* server_;
        FullBodyMap fullBodyMap_;
        rbprm::RbPrmFullBodyPtr_t fullBody()
        {
            if(!fullBodyLoaded_)
                throw Error ("No full body robot was loaded");
            return fullBodyMap_.selected();
        }

        private:
        pinocchio::T_Rom romDevices_;
        //rbprm::RbPrmFullBodyPtr_t fullBody_;
        bool romLoaded_;
        bool fullBodyLoaded_;
        BindShooter bindShooter_;
        rbprm::State startState_;
        rbprm::State endState_;
        std::vector<rbprm::State> lastStatesComputed_;
        rbprm::T_StateFrame lastStatesComputedTime_;
        sampling::AnalysisFactory* analysisFactory_;
        pinocchio::Configuration_t refPose_;
      }; // class RobotBuilder
    } // namespace impl
  } // namespace manipulation
} // namespace hpp

#endif // HPP_RBPRM_CORBA_BUILDER_IMPL_HH
