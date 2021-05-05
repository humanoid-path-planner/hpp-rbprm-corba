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
#define HPP_RBPRM_CORBA_BUILDER_IMPL_HH

#include <hpp/core/problem-solver.hh>
#include <hpp/core/path.hh>
#include "hpp/corbaserver/rbprm/rbprmbuilder-idl.hh"
#include <hpp/rbprm/rbprm-device.hh>
#include <hpp/rbprm/rbprm-fullbody.hh>
#include <hpp/rbprm/rbprm-shooter.hh>
#include <hpp/rbprm/rbprm-validation.hh>
#include <hpp/rbprm/sampling/analysis.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/problem.hh>
#include <hpp/corbaserver/affordance/server.hh>
#include <hpp/corbaserver/problem-solver-map.hh>
#include <hpp/rbprm/rbprm-path-validation.hh>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/core/config-validations.hh>
#include <hpp/rbprm/dynamic/dynamic-path-validation.hh>
#include "hpp/corbaserver/fwd.hh"
#include "hpp/corbaserver/rbprm/server.hh"

namespace hpp {
namespace rbprm {
namespace impl {
using CORBA::Short;

typedef hpp::core::Container<hpp::core::AffordanceObjects_t> affMap_t;

struct BindShooter {
  BindShooter(const std::size_t shootLimit = 10000, const std::size_t displacementLimit = 100)
      : shootLimit_(shootLimit), displacementLimit_(displacementLimit) {}

  hpp::rbprm::RbPrmShooterPtr_t create(
      /*const hpp::pinocchio::DevicePtr_t& robot,*/ const hpp::core::ProblemConstPtr_t& problem) {
    affMap_ = problemSolver_->affordanceObjects;
    hpp::pinocchio::RbPrmDevicePtr_t robotcast =
        std::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(problem->robot());
    if (affMap_.map.empty()) {
      throw hpp::Error("No affordances found. Unable to create shooter object.");
    }
    rbprm::RbPrmShooterPtr_t shooter =
        hpp::rbprm::RbPrmShooter::create(robotcast, problemSolver_->problem()->collisionObstacles(), affMap_,
                                         romFilter_, affFilter_, shootLimit_, displacementLimit_);
    if (!so3Bounds_.empty()) shooter->BoundSO3(so3Bounds_);
    shooter->sampleExtraDOF(problem->getParameter("ConfigurationShooter/sampleExtraDOF").boolValue());
    shooter->ratioWeighted(problem->getParameter("RbprmShooter/ratioWeighted").floatValue());
    return shooter;
  }

  hpp::core::PathValidationPtr_t createPathValidation(const hpp::pinocchio::DevicePtr_t& robot,
                                                      const hpp::pinocchio::value_type& val) {
    hpp::pinocchio::RbPrmDevicePtr_t robotcast = std::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
    affMap_ = problemSolver_->affordanceObjects;
    if (affMap_.map.empty()) {
      throw hpp::Error("No affordances found. Unable to create Path Validaton object.");
    }
    hpp::rbprm::RbPrmValidationPtr_t validation(
        hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
    hpp::rbprm::RbPrmPathValidationPtr_t collisionChecking = hpp::rbprm::RbPrmPathValidation::create(robot, val);
    collisionChecking->add(validation);
    problemSolver_->problem()->configValidation(core::ConfigValidations::create());
    problemSolver_->problem()->configValidations()->add(validation);
    return collisionChecking;
  }

  hpp::core::PathValidationPtr_t createDynamicPathValidation(const hpp::pinocchio::DevicePtr_t& robot,
                                                             const hpp::pinocchio::value_type& val) {
    hpp::pinocchio::RbPrmDevicePtr_t robotcast = std::static_pointer_cast<hpp::pinocchio::RbPrmDevice>(robot);
    affMap_ = problemSolver_->affordanceObjects;
    if (affMap_.map.empty()) {
      throw hpp::Error("No affordances found. Unable to create Path Validaton object.");
    }
    hpp::rbprm::RbPrmValidationPtr_t validation(
        hpp::rbprm::RbPrmValidation::create(robotcast, romFilter_, affFilter_, affMap_));
    hpp::rbprm::DynamicPathValidationPtr_t collisionChecking = hpp::rbprm::DynamicPathValidation::create(robot, val);
    collisionChecking->add(validation);
    problemSolver_->problem()->configValidation(core::ConfigValidations::create());
    problemSolver_->problem()->configValidations()->add(validation);
    // build the dynamicValidation :
    double sizeFootX, sizeFootY, mass, mu;
    bool rectangularContact;
    sizeFootX = problemSolver_->problem()->getParameter(std::string("DynamicPlanner/sizeFootX")).floatValue() / 2.;
    sizeFootY = problemSolver_->problem()->getParameter(std::string("DynamicPlanner/sizeFootY")).floatValue() / 2.;
    if (sizeFootX > 0. && sizeFootY > 0.)
      rectangularContact = 1;
    else
      rectangularContact = 0;
    mass = robot->mass();
    mu = problemSolver_->problem()->getParameter(std::string("DynamicPlanner/friction")).floatValue();
    hppDout(notice, "mu define in python : " << mu);
    DynamicValidationPtr_t dynamicVal =
        DynamicValidation::create(rectangularContact, sizeFootX, sizeFootY, mass, mu, robot);
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

  FullBodyMap(const std::string& name = "None") : selected_(name) {
    // map_[selected_] = init;
  }

  rbprm::RbPrmFullBodyPtr_t operator->() { return selected(); }
  operator rbprm::RbPrmFullBodyPtr_t() { return selected(); }
  rbprm::RbPrmFullBodyPtr_t selected() { return map_[selected_]; }
  bool has(const std::string& name) const {
    // ProblemMap_t::const_iterator it = map_.find (name);
    // return it != map_.end ();
    return map_.end() != map_.find(name);
  }
  template <typename ReturnType>
  ReturnType keys() const {
    ReturnType l;
    for (fMap_t::const_iterator it = map_.begin(); it != map_.end(); ++it) l.push_back(it->first);
    return l;
  }
};

class RbprmBuilder : public virtual POA_hpp::corbaserver::rbprm::RbprmBuilder {
 public:
  RbprmBuilder();

  void setServer(Server* server) { server_ = server; }

  virtual void loadRobotRomModel(const char* robotName,
                                 const char* rootJointType,
                                 const char* urdfName);

  virtual void loadRobotCompleteModel(const char* robotName,
                                      const char* rootJointType,
                                      const char* urdfName,
                                      const char* srdfName);

  virtual void loadFullBodyRobot(const char* robotName,
                                 const char* rootJointType,
                                 const char* urdfName,
                                 const char* srdfName,
                                 const char* selectedProblem);

  virtual void loadFullBodyRobotFromExistingRobot();

  void setStaticStability(const bool staticStability);

  void setReferenceConfig(const hpp::floatSeq& referenceConfig);
  void setPostureWeights(const hpp::floatSeq& postureWeights);
  void setReferenceEndEffector(const char* romName, const hpp::floatSeq& ref);
  void usePosturalTaskContactCreation(const bool usePosturalTaskContactCreation);

  virtual void setFilter(const hpp::Names_t& roms);
  virtual void setAffordanceFilter(const char* romName, const hpp::Names_t& affordances);
  virtual void boundSO3(const hpp::floatSeq& limitszyx);

  virtual hpp::floatSeq* getSampleConfig(const char* limb, unsigned int sampleId);
  virtual hpp::floatSeq* getSamplePosition(const char* limb, unsigned int sampleId);
  virtual hpp::floatSeqSeq* getEffectorPosition(const char* limb,
                                                const hpp::floatSeq& configuration);
  virtual CORBA::UShort getNumSamples(const char* limb);
  virtual hpp::floatSeq* getOctreeNodeIds(const char* limb);
  virtual double getSampleValue(const char* limb, const char* valueName, unsigned int sampleId);
  virtual double getEffectorDistance(unsigned short state1, unsigned short state2);

  rbprm::State generateContacts_internal(const hpp::floatSeq& configuration, const hpp::floatSeq& direction,
                                         const hpp::floatSeq& acceleration,
                                         const double robustnessThreshold);
  virtual hpp::floatSeq* generateContacts(const hpp::floatSeq& configuration, const hpp::floatSeq& direction,
                                          const hpp::floatSeq& acceleration,
                                          const double robustnessThreshold);
  virtual CORBA::Short generateStateInContact(const hpp::floatSeq& configuration, const hpp::floatSeq& direction,
                                              const hpp::floatSeq& acceleration,
                                              const double robustnessThreshold);

  virtual hpp::floatSeq* generateGroundContact(const hpp::Names_t& contactLimbs);

  virtual hpp::floatSeq* getContactSamplesIds(const char* limb, const hpp::floatSeq& configuration,
                                              const hpp::floatSeq& direction);

  virtual hpp::floatSeqSeq* getContactSamplesProjected(const char* limb, const hpp::floatSeq& configuration,
                                                       const hpp::floatSeq& direction,
                                                       unsigned short numSamples);

  virtual short generateContactState(::CORBA::UShort currentState, const char* name,
                                     const ::hpp::floatSeq& direction);

  virtual hpp::floatSeq* getSamplesIdsInOctreeNode(const char* limb, double octreeNodeId);

  virtual void addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset,
                       const hpp::floatSeq& normal, double x, double y, unsigned int samples,
                       const char* heuristicName, double resolution, const char* contactType,
                       double disableEffectorCollision, double grasp, const hpp::floatSeq& limbOffset,
                       const char* kinematicConstraintsPath, double kinematicConstraintsMin);
  virtual void addNonContactingLimb(const char* id, const char* limb, const char* effector,
                                    unsigned int samples);

  virtual void addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues,
                               double disableEffectorCollision, double grasp);

  virtual void setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs);
  virtual void setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs);
  virtual void setStartStateId(unsigned short stateId);
  virtual void setEndStateId(unsigned short stateId);
  virtual hpp::floatSeq* computeContactForConfig(const hpp::floatSeq& configuration,
                                                 const char* limbNam);
  virtual hpp::floatSeqSeq* computeContactPoints(unsigned short cId);
  virtual hpp::floatSeqSeq* computeContactPointsAtState(unsigned short cId,
                                                        unsigned short isIntermediate);
  virtual hpp::floatSeqSeq* computeContactPointsForLimb(unsigned short cId, const char* limbName);
  virtual hpp::floatSeqSeq* computeContactPointsAtStateForLimb(unsigned short cId, unsigned short isIntermediate,
                                                               const char* limbName);
  virtual hpp::floatSeqSeq* computeCenterOfContactAtStateForLimb(unsigned short cId, unsigned short isIntermediate,
                                                                 const char* limbName);
  virtual hpp::floatSeqSeq* interpolate(double timestep, double path, double robustnessTreshold,
                                        unsigned short filterStates, bool testReachability, bool quasiStatic,
                                        bool erasePreviousStates);
  virtual hpp::floatSeqSeq* interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold,
                                               unsigned short filterStates, bool testReachability, bool quasiStatic,
                                               bool erasePreviousStates);
  virtual hpp::floatSeqSeq* getContactCone(unsigned short stateId, double friction);
  virtual hpp::floatSeqSeq* getContactIntermediateCone(unsigned short stateId, double friction);
  virtual CORBA::Short generateComTraj(const hpp::floatSeqSeq& positions, const hpp::floatSeqSeq& velocities,
                                       const hpp::floatSeqSeq& accelerations, const double dt);

  virtual CORBA::Short straightPath(const hpp::floatSeqSeq& positions);
  virtual CORBA::Short generateCurveTraj(const hpp::floatSeqSeq& positions);
  virtual CORBA::Short generateCurveTrajParts(const hpp::floatSeqSeq& positions,
                                              const hpp::floatSeq& partitions);
  virtual CORBA::Short generateRootPath(const hpp::floatSeqSeq& rootPositions, const hpp::floatSeq& q1,
                                        const hpp::floatSeq& q2);
  virtual CORBA::Short limbRRT(unsigned short state1, unsigned short state2,
                               unsigned short numOptimizations);
  virtual CORBA::Short limbRRTFromRootPath(unsigned short state1, unsigned short state2, unsigned short path,
                                           unsigned short numOptimizations);
  virtual CORBA::Short configToPath(const hpp::floatSeqSeq& configs);
  virtual CORBA::Short comRRT(unsigned short state1, unsigned short state2, unsigned short path,
                              unsigned short numOptimizations);

  typedef core::PathPtr_t (*t_rrt)(RbPrmFullBodyPtr_t, core::ProblemSolverPtr_t, const core::PathPtr_t, const State&,
                                   const State&, const std::size_t, const bool);

  hpp::floatSeq* rrt(t_rrt functor, unsigned short state1, unsigned short state2, unsigned short comTraj1,
                     unsigned short comTraj2, unsigned short comTraj3,
                     unsigned short numOptimizations);

  virtual hpp::floatSeq* comRRTFromPos(unsigned short state1, unsigned short comTraj1, unsigned short comTraj2,
                                       unsigned short comTraj3, unsigned short numOptimizations);
  virtual hpp::floatSeq* comRRTFromPosBetweenState(unsigned short state1, unsigned short state2,
                                                   unsigned short comTraj1, unsigned short comTraj2,
                                                   unsigned short comTraj3,
                                                   unsigned short numOptimizations);
  virtual hpp::floatSeq* effectorRRTFromPosBetweenState(unsigned short state1, unsigned short state2,
                                                        unsigned short comTraj1, unsigned short comTraj2,
                                                        unsigned short comTraj3,
                                                        unsigned short numOptimizations);
  virtual hpp::floatSeq* effectorRRT(unsigned short state1, unsigned short comTraj1, unsigned short comTraj2,
                                     unsigned short comTraj3, unsigned short numOptimizations);
  virtual hpp::floatSeq* effectorRRTFromPath(unsigned short state1, unsigned short path, double path_from,
                                             double path_to, unsigned short comTraj1, unsigned short comTraj2,
                                             unsigned short comTraj3, unsigned short numOptimizations,
                                             const hpp::Names_t& trackedEffectors);
  virtual hpp::floatSeq* rrtOnePhase(t_rrt functor, unsigned short state1, unsigned short state2,
                                     unsigned short comTraj, unsigned short numOptimizations);
  virtual hpp::floatSeq* effectorRRTOnePhase(unsigned short state1, unsigned short state2, unsigned short comTraj,
                                             unsigned short numOptimizations);
  virtual hpp::floatSeq* comRRTOnePhase(unsigned short state1, unsigned short state2, unsigned short comTraj,
                                        unsigned short numOptimizations);
  virtual hpp::floatSeqSeq* generateEffectorBezierArray(unsigned short state1, unsigned short state2,
                                                        unsigned short comTraj,
                                                        unsigned short numOptimizations);

  virtual CORBA::Short generateEndEffectorBezier(unsigned short state1, unsigned short state2,
                                                 unsigned short cT);

  virtual hpp::floatSeq* projectToCom(unsigned short state, const hpp::floatSeq& targetCom,
                                      unsigned short max_num_sample);
  virtual CORBA::Short createState(const hpp::floatSeq& configuration,
                                   const hpp::Names_t& contactLimbs);
  virtual hpp::floatSeq* getConfigAtState(unsigned short stateId);
  virtual double setConfigAtState(unsigned short stateId, const hpp::floatSeq& config);
  double projectStateToCOMEigen(State& s, const pinocchio::Configuration_t& com_target,
                                unsigned short maxNumeSamples);
  double projectStateToCOMEigen(unsigned short stateId, const pinocchio::Configuration_t& com_target,
                                unsigned short maxNumeSamples);

  virtual double projectStateToCOM(unsigned short stateId, const hpp::floatSeq& com,
                                   unsigned short max_num_sample);
  virtual CORBA::Short cloneState(unsigned short stateId);
  virtual double projectStateToRoot(unsigned short stateId, const hpp::floatSeq& root,
                                    const hpp::floatSeq& offset);
  virtual void saveComputedStates(const char* filepath);
  virtual void saveLimbDatabase(const char* limbname, const char* filepath);
  virtual hpp::floatSeq* getOctreeBox(const char* limbName, double sampleId);
  virtual CORBA::Short isLimbInContact(const char* limbName, unsigned short state);
  virtual CORBA::Short isLimbInContactIntermediary(const char* limbName, unsigned short state);
  virtual CORBA::Short computeIntermediary(unsigned short state1, unsigned short state2);
  virtual CORBA::Short getNumStates();
  virtual hpp::floatSeqSeq* getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration);
  virtual hpp::floatSeq* getOctreeTransform(const char* limbName,
                                            const hpp::floatSeq& configuration);
  virtual CORBA::Short isConfigBalanced(const hpp::floatSeq& config, const hpp::Names_t& contactLimbs,
                                        double robustnessTreshold, const hpp::floatSeq& CoM);
  virtual double isStateBalanced(unsigned short stateId);
  virtual void runSampleAnalysis(const char* analysis, double isstatic);
  virtual hpp::floatSeq* runLimbSampleAnalysis(const char* limbname, const char* analysis,
                                               double isstatic);
  virtual hpp::floatSeq* evaluateConfig(const hpp::floatSeq& configuration,
                                        const hpp::floatSeq& direction);
  virtual void dumpProfile(const char* logFile);
  virtual double getTimeAtState(unsigned short stateId);
  virtual Names_t* getContactsVariations(unsigned short stateIdFrom, unsigned short stateIdTo);
  virtual Names_t* getCollidingObstacleAtConfig(const ::hpp::floatSeq& configuration,
                                                const char* limbName);
  virtual floatSeqSeq* getContactSurfacesAtConfig(const ::hpp::floatSeq& configuration,
                                                  const char* limbName);

  virtual Names_t* getAllLimbsNames();
  virtual CORBA::Short addNewContact(unsigned short stateId, const char* limbName, const hpp::floatSeq& position,
                                     const hpp::floatSeq& normal, unsigned short max_num_sample, bool lockOtherJoints,
                                     const floatSeq& rotation);
  virtual CORBA::Short removeContact(unsigned short stateId, const char* limbName);
  virtual hpp::floatSeq* computeTargetTransform(const char* limbName, const hpp::floatSeq& configuration,
                                                const hpp::floatSeq& p, const hpp::floatSeq& n);
  virtual Names_t* getEffectorsTrajectoriesNames(unsigned short pathId);
  virtual hpp::floatSeqSeqSeq* getEffectorTrajectoryWaypoints(unsigned short pathId,
                                                              const char* effectorName);
  virtual hpp::floatSeqSeq* getPathAsBezier(unsigned short pathId);

  virtual bool toggleNonContactingLimb(const char* limbName);
  virtual bool areKinematicsConstraintsVerified(const hpp::floatSeq& point);
  virtual bool areKinematicsConstraintsVerifiedForState(unsigned short stateId,
                                                        const hpp::floatSeq& point);
  virtual hpp::floatSeq* isReachableFromState(unsigned short stateFrom, unsigned short stateTo,
                                              const bool useIntermediateState);
  virtual hpp::floatSeq* isDynamicallyReachableFromState(unsigned short stateFrom, unsigned short stateTo,
                                                         bool addPathPerPhase, const hpp::floatSeq& timings,
                                                         short numPointPerPhase);

  void selectFullBody(const char* name) {
    std::string psName(name);
    bool has = fullBodyMap_.has(psName);
    if (!has) throw hpp::Error("unknown fullBody Problem");
    fullBodyMap_.selected_ = psName;
  }

 public:
  void SetProblemSolverMap(hpp::corbaServer::ProblemSolverMapPtr_t psMap);
  void initNewProblemSolver();

 private:
  core::ProblemSolverPtr_t problemSolver() { return server_->problemSolver(); }

  Server* server_;
  FullBodyMap fullBodyMap_;
  rbprm::RbPrmFullBodyPtr_t fullBody() {
    if (!fullBodyLoaded_) throw Error("No full body robot was loaded");
    return fullBodyMap_.selected();
  }

 private:
  pinocchio::T_Rom romDevices_;
  // rbprm::RbPrmFullBodyPtr_t fullBody_;
  bool romLoaded_;
  bool fullBodyLoaded_;
  BindShooter bindShooter_;
  rbprm::State startState_;
  rbprm::State endState_;
  std::vector<rbprm::State> lastStatesComputed_;
  rbprm::T_StateFrame lastStatesComputedTime_;
  sampling::AnalysisFactory* analysisFactory_;
  pinocchio::Configuration_t refPose_;
};  // class RobotBuilder
}  // namespace impl
}  // namespace rbprm
}  // namespace hpp

#endif  // HPP_RBPRM_CORBA_BUILDER_IMPL_HH
