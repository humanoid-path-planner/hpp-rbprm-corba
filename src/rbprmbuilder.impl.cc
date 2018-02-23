// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux, Joseph Mirabel
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

//#include <hpp/fcl/math/transform.h>
#include <hpp/util/debug.hh>
#include <hpp/corbaserver/rbprm/rbprmbuilder.hh>
#include "rbprmbuilder.impl.hh"
#include "hpp/rbprm/rbprm-device.hh"
#include "hpp/rbprm/rbprm-validation.hh"
#include "hpp/rbprm/interpolation/rbprm-path-interpolation.hh"
#include "hpp/rbprm/interpolation/limb-rrt.hh"
#include "hpp/rbprm/interpolation/com-rrt.hh"
#include "hpp/rbprm/interpolation/com-trajectory.hh"
#include "hpp/rbprm/interpolation/spline/effector-rrt.hh"
#include "hpp/rbprm/projection/projection.hh"
#include "hpp/rbprm/contact_generation/contact_generation.hh"
#include "hpp/rbprm/contact_generation/algorithm.hh"
#include "hpp/rbprm/stability/stability.hh"
#include "hpp/rbprm/sampling/sample-db.hh"
#include "hpp/model/urdf/util.hh"
#include "hpp/core/straight-path.hh"
#include "hpp/model/joint.hh"
#include "hpp/core/config-validations.hh"
#include "hpp/core/collision-validation-report.hh"
#include <hpp/core/subchain-path.hh>
#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/collision-validation.hh>
#include <fstream>
#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <hpp/rbprm/planner/rbprm-steering-kinodynamic.hh>
#include <hpp/model/configuration.hh>
#include <algorithm>    // std::random_shuffle
#include <hpp/rbprm/interpolation/time-constraint-helper.hh>
#include "spline/bezier_curve.h"
#include "hpp/rbprm/interpolation/polynom-trajectory.hh"
#include <hpp/rbprm/planner/random-shortcut-dynamic.hh>
#include <hpp/rbprm/planner/oriented-path-optimizer.hh>
#include <hpp/rbprm/sampling/heuristic-tools.hh>
#include <hpp/rbprm/contact_generation/reachability.hh>
#ifdef PROFILE
    #include "hpp/rbprm/rbprm-profiler.hh"
#endif



namespace hpp {
  namespace rbprm {

  typedef spline::bezier_curve<> bezier;
    namespace impl {

    RbprmBuilder::RbprmBuilder ()
    : POA_hpp::corbaserver::rbprm::RbprmBuilder()
    , romLoaded_(false)
    , fullBodyLoaded_(false)
    , bindShooter_()
    , analysisFactory_(0)
    {
        // NOTHING
    }


    hpp::floatSeq vectorToFloatseq (const hpp::core::vector_t& input)
    {
      CORBA::ULong size = (CORBA::ULong) input.size ();
      double* dofArray = hpp::floatSeq::allocbuf(size);
      hpp::floatSeq floats (size, size, dofArray, true);
      for (std::size_t i=0; i<size; ++i) {
        dofArray [i] = input [i];
      }
      return floats;
    }

    void RbprmBuilder::loadRobotRomModel(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        try
        {
            hpp::model::DevicePtr_t romDevice = model::Device::create (robotName);
            romDevices_.insert(std::make_pair(robotName, romDevice));
            hpp::model::urdf::loadRobotModel (romDevice,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        romLoaded_ = true;
    }

    void RbprmBuilder::loadRobotCompleteModel(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix) throw (hpp::Error)
    {
        if(!romLoaded_)
        {
            std::string err("Rom must be loaded before loading complete model") ;
            hppDout (error, err );
            throw hpp::Error(err.c_str());
        }
        try
        {
            hpp::model::RbPrmDevicePtr_t device = hpp::model::RbPrmDevice::create (robotName, romDevices_);
            hpp::model::urdf::loadRobotModel (device,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
            // Add device to the planner
            problemSolver()->robot (device);
            problemSolver()->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
    }

    void RbprmBuilder::loadFullBodyRobot(const char* robotName,
         const char* rootJointType,
         const char* packageName,
         const char* modelName,
         const char* urdfSuffix,
         const char* srdfSuffix,
         const char* selectedProblem) throw (hpp::Error)
    {
        try
        {
            model::DevicePtr_t device = model::Device::create (robotName);
            hpp::model::urdf::loadRobotModel (device,
                    std::string (rootJointType),
                    std::string (packageName),
                    std::string (modelName),
                    std::string (urdfSuffix),
                    std::string (srdfSuffix));
            std::string name(selectedProblem);
            fullBodyLoaded_ = true;
            fullBodyMap_.map_[name] = rbprm::RbPrmFullBody::create(device);
            fullBodyMap_.selected_ = name;
            if(problemSolver()){
                if(problemSolver()->problem()){
                    try {
                      boost::any value = problemSolver()->problem()->get<boost::any> (std::string("friction"));
                      fullBody()->setFriction(boost::any_cast<double>(value));
                      hppDout(notice,"fullbody : mu define in python : "<<fullBody()->getFriction());
                    } catch (const std::exception& e) {
                      hppDout(notice,"fullbody : mu not defined, take : "<<fullBody()->getFriction()<<" as default.");
                    }
                }else{
                    hppDout(warning,"No instance of problem while initializing fullBody");
                }
            }else{
                hppDout(warning,"No instance of problemSolver while initializing fullBody");
            }
            problemSolver()->pathValidationType ("Discretized",0.05); // reset to avoid conflict with rbprm path
            problemSolver()->robot (fullBody()->device_);
            problemSolver()->resetProblem();
            problemSolver()->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
            refPose_ = fullBody()->device_->currentConfiguration();
        }
        catch (const std::exception& exc)
        {
            fullBodyLoaded_ = false;
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        analysisFactory_ = new sampling::AnalysisFactory(fullBody());
    }

    void RbprmBuilder::loadFullBodyRobotFromExistingRobot() throw (hpp::Error)
    {
        try
        {
            fullBody() = rbprm::RbPrmFullBody::create(problemSolver()->problem()->robot());
            problemSolver()->pathValidationType ("Discretized",0.05); // reset to avoid conflict with rbprm path
            problemSolver()->robot (fullBody()->device_);
            problemSolver()->resetProblem();
            problemSolver()->robot ()->controlComputation
            (model::Device::JOINT_POSITION);
        }
        catch (const std::exception& exc)
        {
            hppDout (error, exc.what ());
            throw hpp::Error (exc.what ());
        }
        fullBodyLoaded_ = true;
        analysisFactory_ = new sampling::AnalysisFactory(fullBody());
    }

    hpp::floatSeq* RbprmBuilder::getSampleConfig(const char* limb, unsigned short sampleId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        const T_Limb& limbs = fullBody()->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        const RbPrmLimbPtr_t& limbPtr = lit->second;
        hpp::floatSeq *dofArray;
        Eigen::VectorXd config = fullBody()->device_->currentConfiguration ();
        if(sampleId > limbPtr->sampleContainer_.samples_.size())
        {
            std::string err("Limb " + std::string(limb) + "does not have samples.");
            throw Error (err.c_str());
        }
        const sampling::Sample& sample = limbPtr->sampleContainer_.samples_[sampleId];
        config.segment(sample.startRank_, sample.length_) = sample.configuration_;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(config.rows()));
        for(std::size_t i=0; i< _CORBA_ULong(config.rows()); i++)
          (*dofArray)[(_CORBA_ULong)i] = config [i];
        return dofArray;
    }


    hpp::floatSeq* RbprmBuilder::getSamplePosition(const char* limb, unsigned short sampleId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        const T_Limb& limbs = fullBody()->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        const RbPrmLimbPtr_t& limbPtr = lit->second;
        hpp::floatSeq *dofArray;
        if(sampleId > limbPtr->sampleContainer_.samples_.size())
        {
            std::string err("Limb " + std::string(limb) + "does not have samples.");
            throw Error (err.c_str());
        }
        const sampling::Sample& sample = limbPtr->sampleContainer_.samples_[sampleId];
        const fcl::Vec3f& position = sample.effectorPosition_;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(3));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        return dofArray;
    }


    typedef Eigen::Matrix <value_type, 4, 3, Eigen::RowMajor> Matrix43;
    typedef Eigen::Matrix <value_type, 4, 3, Eigen::RowMajor> Rotation;
    typedef Eigen::Ref<Matrix43> Ref_matrix43;

    std::vector<fcl::Vec3f> computeRectangleContact(const rbprm::RbPrmFullBodyPtr_t device,
                                                    const rbprm::State& state, const std::vector<std::string> limbSelected = std::vector<std::string>())
    {
        device->device_->currentConfiguration(state.configuration_);
        device->device_->computeForwardKinematics();
        std::vector<fcl::Vec3f> res;
        const rbprm::T_Limb& limbs = device->GetLimbs();
        rbprm::RbPrmLimbPtr_t limb;
        Matrix43 p; Eigen::Matrix3d R;
        for(std::map<std::string, fcl::Vec3f>::const_iterator cit = state.contactPositions_.begin();
            cit != state.contactPositions_.end(); ++cit)
        {
            const std::string& name = cit->first;
            if(limbSelected.empty() || std::find(limbSelected.begin(), limbSelected.end(), name) != limbSelected.end())
            {
                const fcl::Vec3f& position = cit->second;
                limb = limbs.at(name);
                const fcl::Vec3f& normal = state.contactNormals_.at(name);
                const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
                const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
                const fcl::Matrix3f rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
                const fcl::Vec3f offset = rotation * limb->offset_;
                const double& lx = limb->x_, ly = limb->y_;
                p << lx,  ly, 0,
                     lx, -ly, 0,
                    -lx, -ly, 0,
                    -lx,  ly, 0;
                if(limb->contactType_ == _3_DOF)
                {
                    //create rotation matrix from normal
                    fcl::Vec3f z_fcl = state.contactNormals_.at(name);
                    Eigen::Vector3d z,x,y;
                    for(int i =0; i<3; ++i) z[i] = z_fcl[i];
                    x = z.cross(Eigen::Vector3d(0,-1,0));
                    if(x.norm() < 10e-6)
                    {
                        y = z.cross(fcl::Vec3f(1,0,0));
                        y.normalize();
                        x = y.cross(z);
                    }
                    else
                    {
                        x.normalize();
                        y = z.cross(x);
                    }
                    R.block<3,1>(0,0) = x;
                    R.block<3,1>(0,1) = y;
                    R.block<3,1>(0,2) = z;
                    /*for(std::size_t i =0; i<4; ++i)
                    {
                        res.push_back(position + (R*(p.row(i).transpose())) + offset);
                        res.push_back(state.contactNormals_.at(name));
                    }*/
                    res.push_back(position + (R*(offset)));
                    res.push_back(state.contactNormals_.at(name));
                }
                else
                {
                    const fcl::Matrix3f& fclRotation = state.contactRotation_.at(name);
                    for(int i =0; i< 3; ++i)
                        for(int j =0; j<3;++j)
                            R(i,j) = fclRotation(i,j);
                    fcl::Vec3f z_axis(0,0,1);
                    fcl::Matrix3f rotationLocal = tools::GetRotationMatrix(z_axis, limb->normal_);
                    for(std::size_t i =0; i<4; ++i)
                    {
                        res.push_back(position + (R*(rotationLocal*(p.row(i).transpose() + limb->offset_))));
                        res.push_back(state.contactNormals_.at(name));
                    }
                }
            }
        }
        return res;
    }

    std::vector<fcl::Vec3f> computeRectangleContactLocalTr(const rbprm::RbPrmFullBodyPtr_t device,
                                                    const rbprm::State& state,
                                                    const std::string& limbName)
    {
        device->device_->currentConfiguration(state.configuration_);
        device->device_->computeForwardKinematics();
        std::vector<fcl::Vec3f> res;
        const rbprm::T_Limb& limbs = device->GetLimbs();
        rbprm::RbPrmLimbPtr_t limb;
        Matrix43 p; Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); Eigen::Matrix3d cFrame = Eigen::Matrix3d::Identity();
        for(std::map<std::string, fcl::Vec3f>::const_iterator cit = state.contactPositions_.begin();
            cit != state.contactPositions_.end(); ++cit)
        {
            const std::string& name = cit->first;
            if(limbName == name)
            {
                const fcl::Vec3f& position = cit->second;
                limb = limbs.at(name);
                const fcl::Vec3f& normal = state.contactNormals_.at(name);
                const fcl::Vec3f z = limb->effector_->currentTransformation().getRotation() * limb->normal_;
                const fcl::Matrix3f alignRotation = tools::GetRotationMatrix(z,normal);
                const fcl::Matrix3f rotation = alignRotation * limb->effector_->currentTransformation().getRotation();
                const fcl::Vec3f offset = rotation * limb->offset_;
                const double& lx = limb->x_, ly = limb->y_;
                p << lx,  ly, 0,
                     lx, -ly, 0,
                    -lx, -ly, 0,
                    -lx,  ly, 0;
                if(limb->contactType_ == _3_DOF)
                {
                    //create rotation matrix from normal
                    Eigen::Vector3d z,x,y;
                    for(int i =0; i<3; ++i) z[i] = normal[i];
                    x = z.cross(Eigen::Vector3d(0,-1,0));
                    if(x.norm() < 10e-6)
                    {
                        y = z.cross(fcl::Vec3f(1,0,0));
                        y.normalize();
                        x = y.cross(z);
                    }
                    else
                    {
                        x.normalize();
                        y = z.cross(x);
                    }
                    cFrame.block<3,1>(0,0) = x;
                    cFrame.block<3,1>(0,1) = y;
                    cFrame.block<3,1>(0,2) = z;
                }
                fcl::Transform3f roWorld, roEffector;
                roWorld.setRotation(state.contactRotation_.at(name));
                roWorld.setTranslation(position);
                roEffector = roWorld; roEffector.inverse();
                fcl::Vec3f z_axis(0,0,1);
                fcl::Matrix3f rotationLocal = tools::GetRotationMatrix(z_axis, limb->normal_);
                if(limb->contactType_ == _3_DOF)
                {
                    fcl::Vec3f pworld = position +  offset;
                    res.push_back((roEffector * pworld).getTranslation());
                    res.push_back(roEffector.getRotation() * state.contactNormals_.at(name));
                }
                else
                {
                    for(std::size_t i =0; i<4; ++i)
                    {
                        /*if(limb->contactType_ == _3_DOF)
                        {
                            fcl::Vec3f pworld = position + (cFrame*(p.row(i).transpose())) + offset;
                            res.push_back((roEffector * pworld).getTranslation());
                            res.push_back(roEffector.getRotation() * state.contactNormals_.at(name));
                        }
                        else*/
                        {
                            res.push_back(rotationLocal*(p.row(i).transpose()) + limb->offset_);
                            //res.push_back(roEffector.getRotation() * state.contactNormals_.at(name));
                           res.push_back( limb->normal_);
                        }
                    }
                }
                return res;
            }
        }
        return res;
    }

    model::Configuration_t dofArrayToConfig (const std::size_t& deviceDim,
      const hpp::floatSeq& dofArray)
    {
        std::size_t configDim = (std::size_t)dofArray.length();
        // Fill dof vector with dof array.
        model::Configuration_t config; config.resize (configDim);
        for (std::size_t iDof = 0; iDof < configDim; iDof++) {
            config [iDof] = (double)dofArray[(_CORBA_ULong)iDof];
        }
        // fill the vector by zero
        hppDout (info, "config dimension: " <<configDim
           <<",  deviceDim "<<deviceDim);
        if(configDim != deviceDim){
            throw hpp::Error ("dofVector Does not match");
        }
        return config;
    }

    model::Configuration_t dofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeq& dofArray)
    {
        return dofArrayToConfig(robot->configSize(), dofArray);
    }

    T_Configuration doubleDofArrayToConfig (const std::size_t& deviceDim,
      const hpp::floatSeqSeq& doubleDofArray)
    {
        std::size_t configsDim = (std::size_t)doubleDofArray.length();
        T_Configuration res;
        for (_CORBA_ULong iConfig = 0; iConfig < configsDim; iConfig++)
        {
            res.push_back(dofArrayToConfig(deviceDim, doubleDofArray[iConfig]));
        }
        return res;
    }

    T_Configuration doubleDofArrayToConfig (const model::DevicePtr_t& robot,
      const hpp::floatSeqSeq& doubleDofArray)
    {
        return doubleDofArrayToConfig(robot->configSize(), doubleDofArray);
    }

    hpp::floatSeqSeq* RbprmBuilder::getEffectorPosition(const char* lb, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try
        {
            const std::string limbName(lb);
            const RbPrmLimbPtr_t limb = fullBody()->GetLimbs().at(limbName);
            model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
            fullBody()->device_->currentConfiguration(config);
            fullBody()->device_->computeForwardKinematics();
            State state;
            state.configuration_ = config;
            state.contacts_[limbName] = true;
            const fcl::Vec3f position = limb->effector_->currentTransformation().getTranslation();
            state.contactPositions_[limbName] = position;
            state.contactNormals_[limbName] = fcl::Vec3f(0,0,1);
            state.contactRotation_[limbName] = limb->effector_->currentTransformation().getRotation();
            std::vector<fcl::Vec3f> poss = (computeRectangleContact(fullBody(), state));

            hpp::floatSeqSeq *res;
            res = new hpp::floatSeqSeq ();
            res->length ((_CORBA_ULong)poss.size ());
            for(std::size_t i = 0; i < poss.size(); ++i)
            {
                _CORBA_ULong size = (_CORBA_ULong) (3);
                double* dofArray = hpp::floatSeq::allocbuf(size);
                hpp::floatSeq floats (size, size, dofArray, true);
                //convert the config in dofseq
                for(std::size_t j=0; j<3; j++)
                {
                    dofArray[j] = poss[i][j];
                }
                (*res) [(_CORBA_ULong)i] = floats;
            }
            return res;
        }
        catch (const std::exception& exc)
        {
            throw hpp::Error (exc.what ());
        }
    }


    CORBA::UShort RbprmBuilder::getNumSamples(const char* limb) throw (hpp::Error)
    {
        const T_Limb& limbs = fullBody()->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        return (CORBA::UShort)(lit->second->sampleContainer_.samples_.size());
    }

    floatSeq *RbprmBuilder::getOctreeNodeIds(const char* limb) throw (hpp::Error)
    {
        const T_Limb& limbs = fullBody()->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        const sampling::T_VoxelSampleId& ids =  lit->second->sampleContainer_.samplesInVoxels_;
        hpp::floatSeq* dofArray = new hpp::floatSeq();
        dofArray->length((_CORBA_ULong)ids.size());
        sampling::T_VoxelSampleId::const_iterator it = ids.begin();
        for(std::size_t i=0; i< _CORBA_ULong(ids.size()); ++i, ++it)
        {
          (*dofArray)[(_CORBA_ULong)i] = (double)it->first;
        }
        return dofArray;
    }

    double RbprmBuilder::getSampleValue(const char* limb, const char* valueName, unsigned short sampleId) throw (hpp::Error)
    {
        const T_Limb& limbs = fullBody()->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limb));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        const sampling::SampleDB& database = lit->second->sampleContainer_;
        if (database.samples_.size() <= sampleId)
        {
            std::string err("unexisting sample id " + sampleId);
            throw Error (err.c_str());
        }
        sampling::T_Values::const_iterator cit = database.values_.find(std::string(valueName));
        if(cit == database.values_.end())
        {
            std::string err("value not existing in database " + std::string(valueName));
            throw Error (err.c_str());
        }
        return cit->second[sampleId];
    }


    double RbprmBuilder::getEffectorDistance(unsigned short state1, unsigned short state2) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            return rbprm::effectorDistance(lastStatesComputed_[s1], lastStatesComputed_[s2]);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    std::vector<std::string> stringConversion(const hpp::Names_t& dofArray)
    {
        std::vector<std::string> res;
        std::size_t dim = (std::size_t)dofArray.length();
        for (_CORBA_ULong iDof = 0; iDof < dim; iDof++)
        {
            res.push_back(std::string(dofArray[iDof]));
        }
        return res;
    }

    std::vector<double> doubleConversion(const hpp::floatSeq& dofArray)
    {
        std::vector<double> res;
        std::size_t dim = (std::size_t)dofArray.length();
        for (_CORBA_ULong iDof = 0; iDof < dim; iDof++)
        {
            res.push_back(dofArray[iDof]);
        }
        return res;
    }

    void RbprmBuilder::setStaticStability(const bool staticStability) throw (hpp::Error){
      if(!fullBodyLoaded_)
        throw Error ("No full body robot was loaded");
      fullBody()->staticStability(staticStability);
    }

    void RbprmBuilder::setReferenceConfig(const hpp::floatSeq& referenceConfig) throw (hpp::Error){
      if(!fullBodyLoaded_)
        throw Error ("No full body robot was loaded");
      Configuration_t config(dofArrayToConfig (fullBody()->device_, referenceConfig));
      refPose_ = config;
      fullBody()->referenceConfig(config);
    }



    void RbprmBuilder::setFilter(const hpp::Names_t& roms) throw (hpp::Error)
    {
        bindShooter_.romFilter_ = stringConversion(roms);
    }


    void RbprmBuilder::setAffordanceFilter(const char* romName, const hpp::Names_t& affordances) throw (hpp::Error)
    {
        std::string name (romName);
				std::vector<std::string> affNames = stringConversion(affordances);
        bindShooter_.affFilter_.erase(name);
        bindShooter_.affFilter_.insert(std::make_pair(name,affNames));
    }

    void RbprmBuilder::boundSO3(const hpp::floatSeq& limitszyx) throw (hpp::Error)
    {
        std::vector<double> limits = doubleConversion(limitszyx);
        if(limits.size() !=6)
        {
            throw Error ("Can not bound SO3, array of 6 double required");
        }
        bindShooter_.so3Bounds_ = limits;

    }

    rbprm::T_Limb GetFreeLimbs(const RbPrmFullBodyPtr_t fullBody, const hpp::rbprm::State &from, const hpp::rbprm::State &to)
    {
        rbprm::T_Limb res;
        std::vector<std::string> fixedContacts = to.fixedContacts(from);
        std::vector<std::string> variations = to.contactVariations(from);
        for(rbprm::CIT_Limb cit = fullBody->GetLimbs().begin();
            cit != fullBody->GetLimbs().end(); ++cit)
        {
            if(std::find(fixedContacts.begin(), fixedContacts.end(), cit->first) == fixedContacts.end())
            {
                if(std::find(variations.begin(), variations.end(), cit->first) != variations.end())
                {
                    res.insert(*cit);
                }
            }
        }
        return res;
    }

    rbprm::T_Limb GetFreeLimbs(const RbPrmFullBodyPtr_t fullBody, const hpp::rbprm::State &state)
    {
        rbprm::T_Limb res;
        std::vector<std::string> fixedContacts = state.fixedContacts(state);
        for(rbprm::CIT_Limb cit = fullBody->GetLimbs().begin();
            cit != fullBody->GetLimbs().end(); ++cit)
        {
            if(std::find(fixedContacts.begin(), fixedContacts.end(), cit->first) == fixedContacts.end())
            {
                    res.insert(*cit);
            }
        }
        return res;
    }

    double RbprmBuilder::projectStateToCOMEigen(unsigned short stateId, const model::Configuration_t& com_target, unsigned short maxNumeSamples) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            }
            State s = lastStatesComputed_[stateId];
//            /hpp::model::Configuration_t c = rbprm::interpolation::projectOnCom(fullBody(), problemSolver()->problem(),s,com_target,succes);
            rbprm::projection::ProjectionReport rep = rbprm::projection::projectToComPosition(fullBody(),com_target,s);
            hpp::model::Configuration_t& c = rep.result_.configuration_;
            ValidationReportPtr_t rport (ValidationReportPtr_t(new CollisionValidationReport));
            CollisionValidationPtr_t val = fullBody()->GetCollisionValidation();
            if(!rep.success_){
                hppDout(notice,"Projection failed for state "<<stateId<<" , config = "<<model::displayConfig(c));
            }
            if(rep.success_){
                hppDout(notice,"Projection successfull for state "<<stateId<<" without collision check.");
                rep.success_ =  rep.success_ &&  val->validate(rep.result_.configuration_,rport);
                if(!rep.success_){
                    hppDout(notice,"Projection failed after collision check for state "<<stateId<<" , config = "<<model::displayConfig(c));
                    hppDout(notice,"report : "<<*rport);
                }
            }
            if (! rep.success_ && maxNumeSamples>0)
            {
                hppDout(notice,"Projection for state "<<stateId<<" failed, try to randomly sample other initial point : ");
                Configuration_t head = s.configuration_.head<7>();
                BasicConfigurationShooterPtr_t shooter = BasicConfigurationShooter::create(fullBody()->device_);
                for(std::size_t i =0; !rep.success_ && i< maxNumeSamples; ++i)
                {
                    s.configuration_ = *shooter->shoot();
                    s.configuration_.head<7>() = head;
                    //c = rbprm::interpolation::projectOnCom(fullBody(), problemSolver()->problem(),s,com_target,succes);
                    rep = rbprm::projection::projectToComPosition(fullBody(),com_target,s);
                    if(!rep.success_){
                        hppDout(notice,"Projection failed on iter "<<i<<" for state "<<stateId<<" , config = "<<model::displayConfig(c));
                    }
                    if(rep.success_){
                        hppDout(notice,"Projection successfull on iter "<<i<<" for state "<<stateId<<" without collision check.");
                        rep.success_ =  rep.success_ &&  val->validate(rep.result_.configuration_,rport);
                        if(!rep.success_){
                            hppDout(notice,"Projection failed on iter "<<i<<" after collision check for state "<<stateId<<" , config = "<<model::displayConfig(c));
                            hppDout(notice,"report : "<<*rport);
                        }
                    }
                    c = rep.result_.configuration_;
                }
            }
            if(rep.success_)
            {
                hpp::model::Configuration_t trySave = c;
                rbprm::T_Limb fLimbs = GetFreeLimbs(fullBody(), s);
                for(rbprm::CIT_Limb cit = fLimbs.begin(); cit != fLimbs.end() && rep.success_; ++cit)
                {
                    // get part of reference configuration that concerns the limb.
                    RbPrmLimbPtr_t limb = cit->second;
                    const sampling::Sample& sample = limb->sampleContainer_.samples_[0];
                    s.configuration_.segment(sample.startRank_, sample.length_) = refPose_.segment(sample.startRank_, sample.length_) ;
                    rep = rbprm::projection::projectToComPosition(fullBody(),com_target,s);
                    rep.success_ = rep.success_ && val->validate(rep.result_.configuration_,rport);
                    if(rep.success_)
                    {
                        std::cout << "yay " << std::endl;
                        trySave = rep.result_.configuration_;
                    }
                    else
                    {
                        std::cout << "ow" << std::endl;
                    }
                }
                lastStatesComputed_[stateId].configuration_ = trySave;
                lastStatesComputedTime_[stateId].second.configuration_ = trySave;
                return 1.;
            }
            return 0;
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERREUR " << std::endl;
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::createState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
        fullBody()->device_->currentConfiguration(config);
        fullBody()->device_->computeForwardKinematics();
        State state;
        state.configuration_ = config;
        std::vector<std::string> names = stringConversion(contactLimbs);
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end(); ++cit)
        {
            rbprm::RbPrmLimbPtr_t limb = fullBody()->GetLimbs().at(*cit);
            const std::string& limbName = *cit;
            state.contacts_[limbName] = true;
            const fcl::Vec3f position = limb->effector_->currentTransformation().getTranslation();
            state.contactPositions_[limbName] = position;
            state.contactNormals_[limbName] = limb->effector_->currentTransformation().getRotation() * limb->normal_;
            state.contactRotation_[limbName] = limb->effector_->currentTransformation().getRotation();
            state.contactOrder_.push(limbName);
        }
        state.nbContacts = state.contactNormals_.size();
        lastStatesComputed_.push_back(state);
        lastStatesComputedTime_.push_back(std::make_pair(-1., state));
        return lastStatesComputed_.size()-1;
    }

    double RbprmBuilder::projectStateToCOM(unsigned short stateId, const hpp::floatSeq& com, unsigned short max_num_sample) throw (hpp::Error)
    {        
        model::Configuration_t com_target = dofArrayToConfig (3, com);
        return projectStateToCOMEigen(stateId, com_target, max_num_sample);
    }

    rbprm::State RbprmBuilder::generateContacts_internal(const hpp::floatSeq& configuration,
      const hpp::floatSeq& direction,const hpp::floatSeq& acceleration, const double robustnessThreshold ) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir,acc;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[(_CORBA_ULong)i];
                acc[i] = acceleration[(_CORBA_ULong)i];
            }
            dir = dir.normalize();

            const affMap_t &affMap = problemSolver()->map<std::vector<boost::shared_ptr<model::CollisionObject> > > ();
		        if (affMap.empty ()) {
    	        throw hpp::Error ("No affordances found. Unable to generate Contacts.");
              }
            model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
            rbprm::State state = rbprm::contact::ComputeContacts(fullBody(),config,
              affMap, bindShooter_.affFilter_, dir,robustnessThreshold,acc);
            return state;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::generateContacts(const hpp::floatSeq& configuration,
      const hpp::floatSeq& direction,const hpp::floatSeq& acceleration, const double robustnessThreshold ) throw (hpp::Error)
    {
        try
        {
            rbprm::State state = generateContacts_internal(configuration,direction,acceleration,robustnessThreshold);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(_CORBA_ULong(state.configuration_.rows()));
            for(std::size_t i=0; i< _CORBA_ULong(state.configuration_.rows()); i++)
              (*dofArray)[(_CORBA_ULong)i] = state.configuration_ [i];
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    CORBA::Short RbprmBuilder::generateStateInContact(const hpp::floatSeq& configuration,
      const hpp::floatSeq& direction,const hpp::floatSeq& acceleration, const double robustnessThreshold ) throw (hpp::Error)
    {
        try
        {
            rbprm::State state = generateContacts_internal(configuration,direction,acceleration,robustnessThreshold);
            lastStatesComputed_.push_back(state);
            lastStatesComputedTime_.push_back(std::make_pair(-1., state));
            return lastStatesComputed_.size()-1;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::generateGroundContact(const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f z(0,0,1);
            ValidationReportPtr_t report = ValidationReportPtr_t(new CollisionValidationReport);
            core::BasicConfigurationShooterPtr_t  shooter = core::BasicConfigurationShooter::create(fullBody()->device_);
            for(int i =0; i< 1000; ++i)
            {
                core::DevicePtr_t device = fullBody()->device_->clone();
                std::vector<std::string> names = stringConversion(contactLimbs);
                core::ConfigProjectorPtr_t proj = core::ConfigProjector::create(device,"proj", 1e-4, 40);
                //hpp::tools::LockJointRec(limb->limb_->name(), body->device_->rootJoint(), proj);
                for(std::vector<std::string>::const_iterator cit = names.begin(); cit !=names.end(); ++cit)
                {
                    rbprm::RbPrmLimbPtr_t limb = fullBody()->GetLimbs().at(*cit);
                    fcl::Transform3f localFrame, globalFrame;
                    localFrame.setTranslation(-limb->offset_);
                    std::vector<bool> posConstraints;
                    std::vector<bool> rotConstraints;
                    posConstraints.push_back(false);posConstraints.push_back(false);posConstraints.push_back(true);
                    rotConstraints.push_back(true);rotConstraints.push_back(true);rotConstraints.push_back(true);
                    proj->add(core::NumericalConstraint::create (constraints::Position::create("",fullBody()->device_,
                                                                                               limb->effector_,
                                                                                               globalFrame,
                                                                                               localFrame,
                                                                                               posConstraints)));
                    if(limb->contactType_ == hpp::rbprm::_6_DOF)
                    {
                        // rotation matrix around z
                        value_type theta = 2*(value_type(rand()) / value_type(RAND_MAX) - 0.5) * M_PI;
                        fcl::Matrix3f r = tools::GetZRotMatrix(theta);
                        fcl::Matrix3f rotation = r * limb->effector_->initialPosition ().getRotation();
                        proj->add(core::NumericalConstraint::create (constraints::Orientation::create("",fullBody()->device_,
                                                                                                      limb->effector_,
                                                                                                      fcl::Transform3f(rotation),
                                                                                                      rotConstraints)));
                    }
                }
                ConfigurationPtr_t configptr = shooter->shoot();
                Configuration_t config = *configptr;
                if(proj->apply(config) && config[2]> 0.3)
                {
                    if(problemSolver()->problem()->configValidations()->validate(config,report))
                    {
                        State tmp;
                        for(std::vector<std::string>::const_iterator cit = names.begin(); cit !=names.end(); ++cit)
                        {
                            std::string limbId = *cit;
                            rbprm::RbPrmLimbPtr_t limb = fullBody()->GetLimbs().at(*cit);
                            tmp.contacts_[limbId] = true;
                            tmp.contactPositions_[limbId] = limb->effector_->currentTransformation().getTranslation();
                            tmp.contactRotation_[limbId] = limb->effector_->currentTransformation().getRotation();
                            tmp.contactNormals_[limbId] = z;
                            tmp.configuration_ = config;
                            ++tmp.nbContacts;
                        }
                        if(stability::IsStable(fullBody(),tmp)>=0)
                        {
                            config[0]=0;
                            config[1]=0;
                            hpp::floatSeq* dofArray = new hpp::floatSeq();
                            dofArray->length(_CORBA_ULong(config.rows()));
                            for(std::size_t j=0; j< config.rows(); j++)
                            {
                              (*dofArray)[(_CORBA_ULong)j] = config [j];
                            }
                            return dofArray;
                        }
                    }
                }
            }
            throw (std::runtime_error("could not generate contact configuration after 1000 trials"));
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::getContactSamplesIds(const char* limbname,
                                        const hpp::floatSeq& configuration,
                                        const hpp::floatSeq& direction) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[(_CORBA_ULong)i];
            }
            model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
            model::Configuration_t save = fullBody()->device_->currentConfiguration();
            fullBody()->device_->currentConfiguration(config);

            sampling::T_OctreeReport finalSet;
            rbprm::T_Limb::const_iterator lit = fullBody()->GetLimbs().find(std::string(limbname));
            if(lit == fullBody()->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + std::string(limbname) + " to robot; limb not defined.");
            }
            const RbPrmLimbPtr_t& limb = lit->second;
            fcl::Transform3f transform = limb->limb_->robot()->rootJoint()->childJoint(0)->currentTransformation (); // get root transform from configuration
                        // TODO fix as in rbprm-fullbody.cc!!
            std::vector<sampling::T_OctreeReport> reports(problemSolver()->collisionObstacles().size());
            std::size_t i (0);
            //#pragma omp parallel for
            for(model::ObjectVector_t::const_iterator oit = problemSolver()->collisionObstacles().begin();
                oit != problemSolver()->collisionObstacles().end(); ++oit, ++i)
            {
                sampling::GetCandidates(limb->sampleContainer_, transform, *oit, dir, reports[i], sampling::HeuristicParam());
            }
            for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
                cit != reports.end(); ++cit)
            {
                finalSet.insert(cit->begin(), cit->end());
            }
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length((_CORBA_ULong)finalSet.size());
            sampling::T_OctreeReport::const_iterator candCit = finalSet.begin();
            for(std::size_t i=0; i< _CORBA_ULong(finalSet.size()); ++i, ++candCit)
            {
              (*dofArray)[(_CORBA_ULong)i] = (double)candCit->sample_->id_;
            }
            fullBody()->device_->currentConfiguration(save);
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeqSeq* RbprmBuilder::getContactSamplesProjected(const char* limbname,
                                        const hpp::floatSeq& configuration,
                                        const hpp::floatSeq& direction,
                                        unsigned short numSamples) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f dir;
            for(std::size_t i =0; i <3; ++i)
            {
                dir[i] = direction[(_CORBA_ULong)i];
            }
            model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
            fullBody()->device_->currentConfiguration(config);

            sampling::T_OctreeReport finalSet;
            rbprm::T_Limb::const_iterator lit = fullBody()->GetLimbs().find(std::string(limbname));
            if(lit == fullBody()->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + std::string(limbname) + " to robot; limb not defined.");
            }
            const RbPrmLimbPtr_t& limb = lit->second;
            fcl::Transform3f transform = limb->octreeRoot(); // get root transform from configuration
                        // TODO fix as in rbprm-fullbody.cc!!
            const affMap_t &affMap = problemSolver()->map
                        <std::vector<boost::shared_ptr<model::CollisionObject> > > ();
            if (affMap.empty ())
            {
                throw hpp::Error ("No affordances found. Unable to interpolate.");
            }
            const model::ObjectVector_t objects = contact::getAffObjectsForLimb(std::string(limbname), affMap,
                                                                       bindShooter_.affFilter_);

            std::vector<sampling::T_OctreeReport> reports(objects.size());
            std::size_t i (0);
            //#pragma omp parallel for
            for(model::ObjectVector_t::const_iterator oit = objects.begin();
                oit != objects.end(); ++oit, ++i)
            {
                sampling::GetCandidates(limb->sampleContainer_, transform, *oit, dir, reports[i], sampling::HeuristicParam());
            }
            for(std::vector<sampling::T_OctreeReport>::const_iterator cit = reports.begin();
                cit != reports.end(); ++cit)
            {
                finalSet.insert(cit->begin(), cit->end());
            }
            // randomize samples
            std::random_shuffle(reports.begin(), reports.end());
            unsigned short num_samples_ok (0);
            bool success(false);
            model::Configuration_t sampleConfig = config;
            std::vector<model::Configuration_t> results;
            sampling::T_OctreeReport::const_iterator candCit = finalSet.begin();
            for(std::size_t i=0; i< _CORBA_ULong(finalSet.size()) && num_samples_ok < numSamples; ++i, ++candCit)
            {
                const sampling::OctreeReport& report = *candCit;
                success = false;
                State state;
                state.configuration_ = config;
                hpp::rbprm::projection::ProjectionReport rep =
                hpp::rbprm::projection::projectSampleToObstacle(fullBody(),std::string(limbname), limb, report, fullBody()->GetCollisionValidation(), sampleConfig, state);
                if(rep.success_)
                {
                    results.push_back(sampleConfig);
                    ++num_samples_ok;
                }
            }
            hpp::floatSeqSeq *res;
            res = new hpp::floatSeqSeq ();

            res->length ((_CORBA_ULong)results.size ());
            i=0;
            std::size_t id = 0;
            for(std::vector<model::Configuration_t>::const_iterator cit = results.begin();
                        cit != results.end(); ++cit, ++id)
            {
                /*std::cout << "ID " << id;
                cit->print();*/
                const core::Configuration_t& config = *cit;
                _CORBA_ULong size = (_CORBA_ULong) config.size ();
                double* dofArray = hpp::floatSeq::allocbuf(size);
                hpp::floatSeq floats (size, size, dofArray, true);
                //convert the config in dofseq
                for (model::size_type j=0 ; j < config.size() ; ++j) {
                  dofArray[j] = config [j];
                }
                (*res) [(_CORBA_ULong)i] = floats;
                ++i;
            }
            return res;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    hpp::floatSeq* RbprmBuilder::getSamplesIdsInOctreeNode(const char* limb,
                                                           double octreeNodeId) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            long ocId ((long)octreeNodeId);
            const T_Limb& limbs = fullBody()->GetLimbs();
            T_Limb::const_iterator lit = limbs.find(std::string(limb));
            if(lit == limbs.end())
            {
                std::string err("No limb " + std::string(limb) + "was defined for robot" + fullBody()->device_->name());
                throw Error (err.c_str());
            }
            const sampling::T_VoxelSampleId& sampleIds =  lit->second->sampleContainer_.samplesInVoxels_;
            sampling::T_VoxelSampleId::const_iterator cit = sampleIds.find(ocId);
            if(cit == sampleIds.end())
            {
                std::stringstream ss; ss << ocId;
                std::string err("No octree node with id " + ss.str() + "was defined for robot" + fullBody()->device_->name());
                throw Error (err.c_str());
            }
            const sampling::VoxelSampleId& ids = cit->second;
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length((_CORBA_ULong)ids.second);
            std::size_t sampleId = ids.first;
            for(std::size_t i=0; i< _CORBA_ULong(ids.second); ++i, ++sampleId)
            {
              (*dofArray)[(_CORBA_ULong)i] = (double)sampleId;
            }
            return dofArray;
        } catch (const std::exception& exc) {
        throw hpp::Error (exc.what ());
        }
    }

    void RbprmBuilder::addLimb(const char* id, const char* limb, const char* effector, const hpp::floatSeq& offset, const hpp::floatSeq& normal, double x, double y,
                               unsigned int samples, const char* heuristicName, double resolution, const char *contactType, double disableEffectorCollision, double grasp, const hpp::floatSeq &limbOffset) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            fcl::Vec3f off, norm,limbOff;
            for(std::size_t i =0; i <3; ++i)
            {
                off[i] = offset[(_CORBA_ULong)i];
                norm[i] = normal[(_CORBA_ULong)i];
                limbOff[i] = limbOffset[(_CORBA_ULong)i];

            }
            ContactType cType = hpp::rbprm::_6_DOF;
            if(std::string(contactType) == "_3_DOF")
            {
                cType = hpp::rbprm::_3_DOF;
            }
            fullBody()->AddLimb(std::string(id), std::string(limb), std::string(effector), off,limbOff, norm, x, y,
                               problemSolver()->collisionObstacles(), samples,heuristicName,resolution,cType,disableEffectorCollision > 0.5, grasp > 0.5);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::addNonContactingLimb(const char* id, const char* limb, const char* effector, unsigned int samples) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {

            fullBody()->AddNonContactingLimb(std::string(id), std::string(limb), std::string(effector), problemSolver()->collisionObstacles(), samples);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    void RbprmBuilder::addLimbDatabase(const char* databasePath, const char* id, const char* heuristicName, double loadValues, double disableEffectorCollision, double grasp) throw (hpp::Error)
    {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        try
        {
            std::string fileName(databasePath);
            fullBody()->AddLimb(fileName, std::string(id), problemSolver()->collisionObstacles(), heuristicName, loadValues > 0.5,
                               disableEffectorCollision > 0.5, grasp > 0.5);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void SetPositionAndNormal(rbprm::State& state,
			hpp::rbprm::RbPrmFullBodyPtr_t fullBody, const hpp::floatSeq& configuration,
            std::vector<std::string>& names)
    {
        core::Configuration_t old = fullBody->device_->currentConfiguration();
        model::Configuration_t config = dofArrayToConfig (fullBody->device_, configuration);
        fullBody->device_->currentConfiguration(config);
        fullBody->device_->computeForwardKinematics();
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end();++cit)
        {
            rbprm::T_Limb::const_iterator lit = fullBody->GetLimbs().find(*cit);
            if(lit == fullBody->GetLimbs().end())
            {
                throw std::runtime_error ("Impossible to find limb for joint "
                                          + (*cit) + " to robot; limb not defined");
            }
            const core::JointPtr_t joint = fullBody->device_->getJointByName(lit->second->effector_->name());
            const fcl::Transform3f& transform =  joint->currentTransformation ();
            const fcl::Matrix3f& rot = transform.getRotation();
            state.contactPositions_[*cit] = transform.getTranslation();
            state.contactRotation_[*cit] = rot;
            const fcl::Vec3f z = transform.getRotation() * lit->second->normal_;
            state.contactNormals_[*cit] = z;
            state.contacts_[*cit] = true;
            state.contactOrder_.push(*cit);
        }        
        state.nbContacts = state.contactNormals_.size() ;
        state.configuration_ = config;
        state.robustness =  stability::IsStable(fullBody,state);
        state.stable = state.robustness >= 0;
        fullBody->device_->currentConfiguration(old);
    }

    void RbprmBuilder::setStartState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        try
        {
            std::vector<std::string> names = stringConversion(contactLimbs);
            SetPositionAndNormal(startState_,fullBody(), configuration, names);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    hpp::floatSeq* RbprmBuilder::computeContactForConfig(const hpp::floatSeq& configuration, const char *limbNam) throw (hpp::Error)
    {
        State state;
        std::string limb(limbNam);
        try
        {
            std::vector<std::string> limbs; limbs.push_back(limbNam);
            SetPositionAndNormal(state,fullBody(), configuration, limbs);

            const std::vector<fcl::Vec3f>& positions = computeRectangleContactLocalTr(fullBody(),state,limb);
            _CORBA_ULong size = (_CORBA_ULong) (positions.size () * 3);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(size);
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    (*dofArray)[(_CORBA_ULong)j] = positions[h][k];
                }
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }

    }

    void RbprmBuilder::setEndState(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs) throw (hpp::Error)
    {
        try
        {
            std::vector<std::string> names = stringConversion(contactLimbs);
            SetPositionAndNormal(endState_,fullBody(), configuration, names);
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::setStartStateId(unsigned short stateId) throw (hpp::Error){
        try{
            if(lastStatesComputed_.size() == 0)
            {
                throw std::runtime_error ("states not yet computed, call interpolate() first.");
            }
            if(lastStatesComputed_.size() <= stateId){
              throw std::runtime_error ("invalid state id : "+std::string(""+stateId)+" number of state = "+std::string(""+lastStatesComputed_.size()));
            }
            startState_ = lastStatesComputed_[stateId];
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::setEndStateId(unsigned short stateId) throw (hpp::Error){
        try{
            if(lastStatesComputed_.size() == 0)
            {
                throw std::runtime_error ("states not yet computed, call interpolate() first.");
            }
            if(lastStatesComputed_.size() <= stateId){
              throw std::runtime_error ("invalid state id : "+std::string(""+stateId)+" number of state = "+std::string(""+lastStatesComputed_.size()));
            }
            endState_ = lastStatesComputed_[stateId];

        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    double RbprmBuilder::getTimeAtState(unsigned short stateId)throw (hpp::Error){
      try
      {
          if(lastStatesComputed_.size() == 0)
          {
              throw std::runtime_error ("states not yet computed, call interpolate() first.");
          }
          if(lastStatesComputedTime_.size() <= stateId){
            throw std::runtime_error ("invalid state id : "+std::string(""+stateId)+" number of state = "+std::string(""+lastStatesComputedTime_.size()));
          }
          return lastStatesComputedTime_[stateId].first;
      }
      catch(std::runtime_error& e)
      {
          throw Error(e.what());
      }
    }

    Names_t* RbprmBuilder::getContactsVariations(unsigned short stateIdFrom,unsigned short stateIdTo )throw (hpp::Error){
      try
      {
          if(lastStatesComputed_.size() == 0)
          {
              throw std::runtime_error ("states not yet computed, call interpolate() first.");
          }
          if(lastStatesComputedTime_.size() <= stateIdFrom){
            throw std::runtime_error ("invalid state id : "+std::string(""+stateIdFrom)+" number of state = "+std::string(""+lastStatesComputedTime_.size()));
          }
          if(lastStatesComputedTime_.size() <= stateIdTo){
            throw std::runtime_error ("invalid state id : "+std::string(""+stateIdTo)+" number of state = "+std::string(""+lastStatesComputedTime_.size()));
          }
          State stateFrom = lastStatesComputed_[stateIdFrom];
          State stateTo = lastStatesComputed_[stateIdTo];
          std::vector<std::string> variations_s = stateTo.allVariations(stateFrom,rbprm::interpolation::extractEffectorsName(fullBody()->GetLimbs()));
          CORBA::ULong size = (CORBA::ULong) variations_s.size ();
          char** nameList = Names_t::allocbuf(size);
          Names_t *variations = new Names_t (size,size,nameList);
          for (std::size_t i = 0 ; i < variations_s.size() ; ++i){
            nameList[i] = (char*) malloc (sizeof(char)*(variations_s[i].length ()+1));
            strcpy (nameList [i], variations_s[i].c_str ());
          }
          return variations;
      }
      catch(std::runtime_error& e)
      {
          throw Error(e.what());
      }
    }

    std::vector<State> TimeStatesToStates(const T_StateFrame& ref)
    {
        std::vector<State> res;
        for(CIT_StateFrame cit = ref.begin(); cit != ref.end(); ++cit)
        {
            res.push_back(cit->second);
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::interpolateConfigs(const hpp::floatSeqSeq& configs, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error)
    {
        try
        {
            if(startState_.configuration_.rows() == 0)
            {
                throw std::runtime_error ("Start state not initialized, can not interpolate ");
            }
            if(endState_.configuration_.rows() == 0)
            {
                throw std::runtime_error ("End state not initialized, can not interpolate ");
            }
            T_Configuration configurations = doubleDofArrayToConfig(fullBody()->device_, configs);
            const affMap_t &affMap = problemSolver()->map
                        <std::vector<boost::shared_ptr<model::CollisionObject> > > ();
            if (affMap.empty ())
            {
                throw hpp::Error ("No affordances found. Unable to interpolate.");
            }
            hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator = rbprm::interpolation::RbPrmInterpolation::create(fullBody(),startState_,endState_);
            lastStatesComputedTime_ = interpolator->Interpolate(affMap, bindShooter_.affFilter_,configurations,robustnessTreshold, filterStates != 0);
            lastStatesComputed_ = TimeStatesToStates(lastStatesComputedTime_);
            hpp::floatSeqSeq *res;
            res = new hpp::floatSeqSeq ();

            res->length ((_CORBA_ULong)lastStatesComputed_.size ());
            std::size_t i=0;
            std::size_t id = 0;
            for(std::vector<State>::const_iterator cit = lastStatesComputed_.begin(); cit != lastStatesComputed_.end(); ++cit, ++id)
            {
                std::cout << "ID " << id;
                cit->print();
                const core::Configuration_t config = cit->configuration_;
                _CORBA_ULong size = (_CORBA_ULong) config.size ();
                double* dofArray = hpp::floatSeq::allocbuf(size);
                hpp::floatSeq floats (size, size, dofArray, true);
                //convert the config in dofseq
                for (model::size_type j=0 ; j < config.size() ; ++j) {
                  dofArray[j] = config [j];
                }
                (*res) [(_CORBA_ULong)i] = floats;
                ++i;
            }
            return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeqSeq* contactCone(RbPrmFullBodyPtr_t fullBody, State& state, const double friction)
    {
        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        std::pair<stability::MatrixXX, stability::VectorX> cone = stability::ComputeCentroidalCone(fullBody, state, friction);
        res->length ((_CORBA_ULong)cone.first.rows());
        _CORBA_ULong size = (_CORBA_ULong) cone.first.cols()+1;
        for(int i=0; i < cone.first.rows(); ++i)
        {
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the row in dofseq
            for (int j=0 ; j < cone.first.cols() ; ++j)
            {
                dofArray[j] = cone.first(i,j);
            }
            dofArray[size-1] =cone.second[i];
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    hpp::floatSeqSeq* RbprmBuilder::getContactCone(unsigned short stateId, double friction) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            }
            return contactCone(fullBody(), lastStatesComputed_[stateId],friction);
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    State intermediary(const State& firstState, const State& thirdState, unsigned short& cId, bool& success)
    {
        success = false;
        std::vector<std::string> breaks;
        thirdState.contactBreaks(firstState, breaks);
        if(breaks.size() > 1)
        {
            throw std::runtime_error ("too many contact breaks between states" + std::string(""+cId) + "and " + std::string(""+(cId + 1)));
        }
        if(breaks.size() == 1)
        {
            State intermediary(firstState);
            intermediary.RemoveContact(*breaks.begin());
            success = true;
            return intermediary;
        }
        std::cout << "no contact break during intermediary phase " << std::endl;
        return firstState;
    }

    hpp::floatSeqSeq* RbprmBuilder::getContactIntermediateCone(unsigned short stateId, double friction) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId+1)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId+1)));
            }
            bool success;
            State intermediaryState = intermediary(lastStatesComputed_[stateId], lastStatesComputed_[stateId+1],stateId,success);
            if(!success)
            {
                throw std::runtime_error ("No contact breaks, hence no intermediate state from state " + std::string(""+(stateId)));
            }
            return contactCone(fullBody(),intermediaryState, friction);
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    Names_t* RbprmBuilder::getEffectorsTrajectoriesNames(unsigned short pathId)throw (hpp::Error){
        try{
            if(!fullBodyLoaded_)
                throw std::runtime_error ("No Fullbody loaded");
            EffectorTrajectoriesMap_t map;
            if(! fullBody()->getEffectorsTrajectories(pathId,map))
                //throw std::runtime_error ("Error while retrieving the End effector map for pathId = "+ std::string(""+pathId));
                return new Names_t(); // Return an empty list or throw an error ??
            std::vector<std::string> names;
            for(EffectorTrajectoriesMap_t::const_iterator it = map.begin() ; it != map.end() ; ++it){
                names.push_back(it->first);
            }
            // convert names (vector of string) to corba Names_t
            CORBA::ULong size = (CORBA::ULong) names.size ();
            char** nameList = Names_t::allocbuf(size);
            Names_t *limbsNames = new Names_t (size,size,nameList);
            for (std::size_t i = 0 ; i < names.size() ; ++i){
              nameList[i] = (char*) malloc (sizeof(char)*(names[i].length ()+1));
              strcpy (nameList [i], names[i].c_str ());
            }
            return limbsNames;
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    hpp::floatSeqSeqSeq* RbprmBuilder::getEffectorTrajectoryWaypoints(unsigned short pathId,const char* effectorName)throw (hpp::Error){
        try{
            if(!fullBodyLoaded_)
                throw std::runtime_error ("No Fullbody loaded");
            std::vector<bezier_Ptr> curvesVector;
            if(! fullBody()->getEffectorTrajectory(pathId,effectorName,curvesVector))
                throw std::runtime_error ("There is no trajectory stored for path Id = "+ boost::lexical_cast<std::string>(pathId) +" and end effector name = "+std::string(effectorName));
            // 3 dimensionnal array : first index is the curve, second index is the wp of the curve, third index is the coordinate of each wp
            hpp::floatSeqSeqSeq *res;
            res = new hpp::floatSeqSeqSeq();
            res->length((_CORBA_ULong) curvesVector.size());
            size_t curveId = 0;
            for(std::vector<bezier_Ptr>::const_iterator cit=curvesVector.begin() ; cit != curvesVector.end();++cit,curveId++){
                const bezier_t::t_point_t waypoints = (*cit)->waypoints();
                // build a floatSeqSeq from the waypoints :
                hpp::floatSeqSeq *curveWp;
                curveWp = new hpp::floatSeqSeq ();
                _CORBA_ULong size = (_CORBA_ULong)waypoints.size()+1;
                curveWp->length (size); // +1 because the first value is the length (time)
                { // add the time at the first index :
                    double* dofArray = hpp::floatSeq::allocbuf(1);
                    hpp::floatSeq floats (1, 1, dofArray, true);
                    dofArray[0] = (*cit)->max();
                    (*curveWp) [(_CORBA_ULong)0] = floats; // Always assume the curve start at 0. There isn't any ways to create it otherwise in python
                }
                // now add the waypoints :
                std::size_t i=1;
                for(bezier_t::t_point_t::const_iterator wit = waypoints.begin(); wit != waypoints.end(); ++wit,++i)
                {

                    const bezier_t::point_t position = *wit;
                    (*curveWp) [(_CORBA_ULong)i] = vectorToFloatseq(position);
                }
                (*res)[(_CORBA_ULong)curveId] = (*curveWp);
            }
            return res;
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }




    core::PathPtr_t makePath(DevicePtr_t device,
                             const ProblemPtr_t& problem,
                             model::ConfigurationIn_t q1,
                             model::ConfigurationIn_t q2)
    {
        // TODO DT
        return problem->steeringMethod()->operator ()(q1,q2);
    }

    model::Configuration_t addRotation(CIT_Configuration& pit, const model::value_type& u,
                              model::ConfigurationIn_t q1, model::ConfigurationIn_t q2,
                              model::ConfigurationIn_t ref)
    {
        model::Configuration_t res = ref;
        res.head(3) = *pit;
        res.segment<4>(3) = tools::interpolate(q1, q2, u);
        return res;
    }

    core::PathVectorPtr_t addRotations(const T_Configuration& positions,
                                       model::ConfigurationIn_t q1, model::ConfigurationIn_t q2,
                                       model::ConfigurationIn_t ref, DevicePtr_t device,
                                       const ProblemPtr_t& problem)
    {
        core::PathVectorPtr_t res = core::PathVector::create(device->configSize(), device->numberDof());
        model::value_type size_step = 1 /(model::value_type)(positions.size());
        model::value_type u = 0.;
        CIT_Configuration pit = positions.begin();
        model::Configuration_t previous = addRotation(pit, 0., q1, q2, ref), current;
        ++pit;
        for(;pit != positions.end()-1; ++pit, u+=size_step)
        {
            current = addRotation(pit, u, q1, q2, ref);
            res->appendPath(makePath(device,problem, previous, current));
            previous = current;
        }
        // last configuration is exactly q2
        current = addRotation(pit, 1., q1, q2, ref);
        res->appendPath(makePath(device,problem, previous, current));
        return res;
    }

   core::PathVectorPtr_t generateTrunkPath(RbPrmFullBodyPtr_t fullBody, core::ProblemSolverPtr_t problemSolver, const hpp::floatSeqSeq& rootPositions,
                          const model::Configuration_t q1, const model::Configuration_t q2) throw (hpp::Error)
    {
        try
        {
            T_Configuration positions = doubleDofArrayToConfig(3, rootPositions);
            if(positions.size() <2)
            {
                throw std::runtime_error("generateTrunkPath requires at least 2 configurations to generate path");
            }
            return addRotations(positions,q1,q2,fullBody->device_->currentConfiguration(),
                                                 fullBody->device_,problemSolver->problem());
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    CORBA::Short RbprmBuilder::generateRootPath(const hpp::floatSeqSeq& rootPositions,
                          const hpp::floatSeq& q1Seq, const hpp::floatSeq& q2Seq) throw (hpp::Error)
    {
        try
        {
            model::Configuration_t q1 = dofArrayToConfig(4, q1Seq), q2 = dofArrayToConfig(4, q2Seq);
            return problemSolver()->addPath(generateTrunkPath(fullBody(), problemSolver(), rootPositions, q1, q2));
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::straightPath(const hpp::floatSeqSeq& positions) throw (hpp::Error)
     {
         try
         {
             T_Configuration c = doubleDofArrayToConfig(3, positions);
             if(c.size() <2)
             {
                 throw std::runtime_error("straightPath requires at least 2 configurations to generate path");
             }
             core::PathVectorPtr_t res = core::PathVector::create(3, 3);
             CIT_Configuration cit = c.begin(); ++cit;
             int i = 0;
             model::vector3_t zero (0.,0.,0.);
             for(;cit != c.end(); ++cit, ++i)
             {
                 model::vector3_t speed = (*cit) -  *(cit-1);
                 res->appendPath(interpolation::ComTrajectory::create(*(cit-1),*cit,speed,zero,1.));
             }
             return problemSolver()->addPath(res);
         }
         catch(std::runtime_error& e)
         {
             throw Error(e.what());
         }
     }


    CORBA::Short RbprmBuilder::generateCurveTraj(const hpp::floatSeqSeq& positions) throw (hpp::Error)
     {
         try
         {
             T_Configuration c = doubleDofArrayToConfig(3, positions);
             bezier* curve = new bezier(c.begin(), c.end());
             hpp::rbprm::interpolation::PolynomPtr_t curvePtr (curve);
             hpp::rbprm::interpolation::PolynomTrajectoryPtr_t path = hpp::rbprm::interpolation::PolynomTrajectory::create(curvePtr);
             core::PathVectorPtr_t res = core::PathVector::create(3, 3);
             res->appendPath(path);
             return problemSolver()->addPath(res);
         }
         catch(std::runtime_error& e)
         {
             throw Error(e.what());
         }
     }

    CORBA::Short RbprmBuilder::generateCurveTrajParts(const hpp::floatSeqSeq& positions, const hpp::floatSeq& partitions) throw (hpp::Error)
     {
         try
         {
             model::Configuration_t config = dofArrayToConfig ((std::size_t)partitions.length(), partitions);
             T_Configuration c = doubleDofArrayToConfig(3, positions);
             bezier* curve = new bezier(c.begin(), c.end());
             hpp::rbprm::interpolation::PolynomPtr_t curvePtr (curve);
             hpp::rbprm::interpolation::PolynomTrajectoryPtr_t path = hpp::rbprm::interpolation::PolynomTrajectory::create(curvePtr);
             core::PathVectorPtr_t res = core::PathVector::create(3, 3);
             res->appendPath(path);
             std::size_t returned_pathId =problemSolver()->addPath(res);
             for (int i = 1; i < config.rows(); ++i)
             {
                core::PathPtr_t cutPath = path->extract(interval_t (config(i-1), config(i)));
                res = core::PathVector::create(3, 3);
                res->appendPath(cutPath);
                problemSolver()->addPath(res);
             }
             return returned_pathId;
         }
         catch(std::runtime_error& e)
         {
             throw Error(e.what());
         }
     }

    CORBA::Short RbprmBuilder::generateComTraj(const hpp::floatSeqSeq& positions, const hpp::floatSeqSeq& velocities,
                                          const hpp::floatSeqSeq& accelerations,
                                          const double dt) throw (hpp::Error)
     {
         try
         {
             T_Configuration c = doubleDofArrayToConfig(3, positions);
             T_Configuration cd = doubleDofArrayToConfig(3, velocities);
             T_Configuration cdd = doubleDofArrayToConfig(3, accelerations);
             if(c.size() <2)
             {
                 throw std::runtime_error("generateComTraj requires at least 2 configurations to generate path");
             }
             core::PathVectorPtr_t res = core::PathVector::create(3, 3);
             if(cdd.size() != c.size()-1 || cdd.size() != cd.size())
             {
                 std::cout << c.size() << " " << cd.size() << " " << cdd.size() << std::endl;
                 throw std::runtime_error("in generateComTraj, positions and accelerations vector should have the same size");
             }
             CIT_Configuration cit = c.begin(); ++cit;
             CIT_Configuration cdit = cd.begin();
             CIT_Configuration cddit = cdd.begin();
             int i = 0;
             for(;cit != c.end(); ++cit, ++cdit, ++cddit, ++i)
             {
                 res->appendPath(interpolation::ComTrajectory::create(*(cit-1),*cit,*cdit,*cddit,dt));
             }
             return problemSolver()->addPath(res);
         }
         catch(std::runtime_error& e)
         {
             throw Error(e.what());
         }
     }


    floatSeqSeq* RbprmBuilder::computeContactPoints(unsigned short cId) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + 1)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId+1)));
        }
        const State& firstState = lastStatesComputed_[cId], thirdState = lastStatesComputed_[cId+1];
        std::vector<std::vector<fcl::Vec3f> > allStates;
        allStates.push_back(computeRectangleContact(fullBody(), firstState));
        std::vector<std::string> creations;
        bool success(false);
        State intermediaryState = intermediary(firstState, thirdState, cId, success);
        if(success)
        {
            allStates.push_back(computeRectangleContact(fullBody(), intermediaryState));
        }
        thirdState.contactCreations(firstState, creations);
        if(creations.size() == 1)
        {
            allStates.push_back(computeRectangleContact(fullBody(), thirdState));
        }
        if(creations.size() > 1)
        {
            throw std::runtime_error ("too many contact creations between states" + std::string(""+cId) + "and " + std::string(""+(cId + 1)));
        }

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        // compute array of contact positions

        res->length ((_CORBA_ULong)allStates.size());
        std::size_t i=0;
        for(std::vector<std::vector<fcl::Vec3f> >::const_iterator cit = allStates.begin();
                    cit != allStates.end(); ++cit, ++i)
        {
            const std::vector<fcl::Vec3f>& positions = *cit;
            _CORBA_ULong size = (_CORBA_ULong) positions.size () * 3;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    dofArray[j] = positions[h][k];
                }
            }
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::computeContactPointsAtState(unsigned short cId, unsigned short isIntermediate) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + isIntermediate)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId)));
        }
        State state = lastStatesComputed_[cId];
        if(isIntermediate > 0)
        {
            const State&  thirdState = lastStatesComputed_[cId+1];
            bool success(false);
            State interm = intermediary(state, thirdState, cId, success);
            if(success)
                state = interm;
        }
        std::vector<std::vector<fcl::Vec3f> > allStates;
        allStates.push_back(computeRectangleContact(fullBody(), state));

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        // compute array of contact positions

        res->length ((_CORBA_ULong)allStates.size());
        std::size_t i=0;
        for(std::vector<std::vector<fcl::Vec3f> >::const_iterator cit = allStates.begin();
                    cit != allStates.end(); ++cit, ++i)
        {
            const std::vector<fcl::Vec3f>& positions = *cit;
            _CORBA_ULong size = (_CORBA_ULong) positions.size () * 3;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    dofArray[j] = positions[h][k];
                }
            }
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }


    floatSeqSeq* RbprmBuilder::computeContactPointsForLimb(unsigned short cId, const char *limbName) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + 1)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId+1)));
        }
        std::string limb(limbName);
        const State& firstState = lastStatesComputed_[cId], thirdState = lastStatesComputed_[cId+1];
        std::vector<std::vector<fcl::Vec3f> > allStates;
        allStates.push_back(computeRectangleContactLocalTr(fullBody(), firstState, limb));
        std::vector<std::string> creations;
        bool success(false);
        State intermediaryState = intermediary(firstState, thirdState, cId, success);
        if(success)
        {
            allStates.push_back(computeRectangleContactLocalTr(fullBody(), intermediaryState, limb));
        }
        thirdState.contactCreations(firstState, creations);
        if(creations.size() == 1)
        {
            allStates.push_back(computeRectangleContactLocalTr(fullBody(), thirdState, limb));
        }
        if(creations.size() > 1)
        {
            throw std::runtime_error ("too many contact creations between states" + std::string(""+cId) + "and " + std::string(""+(cId + 1)));
        }

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        // compute array of contact positions

        res->length ((_CORBA_ULong)allStates.size());
        std::size_t i=0;
        for(std::vector<std::vector<fcl::Vec3f> >::const_iterator cit = allStates.begin();
                    cit != allStates.end(); ++cit, ++i)
        {
            const std::vector<fcl::Vec3f>& positions = *cit;
            _CORBA_ULong size = (_CORBA_ULong) positions.size () * 3;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    dofArray[j] = positions[h][k];
                }
            }
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::computeContactPointsAtStateForLimb(unsigned short cId, unsigned short isIntermediate, const char *limbName) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + isIntermediate)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId)));
        }
        std::string limb(limbName);
        State state = lastStatesComputed_[cId];
        if(isIntermediate > 0)
        {
            const State&  thirdState = lastStatesComputed_[cId+1];
            bool success(false);
            State interm = intermediary(state, thirdState, cId, success);
            if(success)
                state = interm;
        }
        std::vector<std::vector<fcl::Vec3f> > allStates;
        std::vector<std::string> limbs ; limbs.push_back(limb);
        allStates.push_back(computeRectangleContact(fullBody(), state,limbs));

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        // compute array of contact positions

        res->length ((_CORBA_ULong)allStates.size());
        std::size_t i=0;
        for(std::vector<std::vector<fcl::Vec3f> >::const_iterator cit = allStates.begin();
                    cit != allStates.end(); ++cit, ++i)
        {
            const std::vector<fcl::Vec3f>& positions = *cit;
            _CORBA_ULong size = (_CORBA_ULong) positions.size () * 3;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for(std::size_t h = 0; h<positions.size(); ++h)
            {
                for(std::size_t k =0; k<3; ++k)
                {
                    model::size_type j (h*3 + k);
                    dofArray[j] = positions[h][k];
                }
            }
            (*res) [(_CORBA_ULong)i] = floats;
        }
        return res;
    }

    floatSeqSeq* RbprmBuilder::computeCenterOfContactAtStateForLimb(unsigned short cId, unsigned short isIntermediate, const char *limbName) throw (hpp::Error)
    {
        if(lastStatesComputed_.size() <= cId + isIntermediate)
        {
            throw std::runtime_error ("Unexisting state " + std::string(""+(cId)));
        }
        std::string limb(limbName);
        State state = lastStatesComputed_[cId];
        if(isIntermediate > 0)
        {
            const State&  thirdState = lastStatesComputed_[cId+1];
            bool success(false);
            State interm = intermediary(state, thirdState, cId, success);
            if(success)
                state = interm;
        }
        if(state.contacts_.find(limb) == state.contacts_.end()){
            throw std::runtime_error ("Limb : "+limb+" is not in contact at state "+std::string(""+(cId)));
        }


        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();


        res->length ((_CORBA_ULong)2);

        fcl::Vec3f& position = state.contactPositions_.at(limbName);
        position += fullBody()->GetLimb(limb)->offset_;
        _CORBA_ULong size = (_CORBA_ULong) 3;
        double* dofArray = hpp::floatSeq::allocbuf(size);
        hpp::floatSeq floats (size, size, dofArray, true);
        //convert the config in dofseq
        for(std::size_t k =0; k<3; ++k)
        {
            dofArray[k] = position[k];
        }
        (*res) [(_CORBA_ULong)0] = floats;

        const fcl::Vec3f& normal = state.contactNormals_.at(limbName);
        double* dofArray_n = hpp::floatSeq::allocbuf(size);
        hpp::floatSeq floats_n (size, size, dofArray_n, true);
        //convert the config in dofseq
        for(std::size_t k =0; k<3; ++k)
        {
            dofArray_n[k] = normal[k];
        }
        (*res) [(_CORBA_ULong)1] = floats_n;

        return res;
    }


    floatSeqSeq* RbprmBuilder::interpolate(double timestep, double path, double robustnessTreshold, unsigned short filterStates) throw (hpp::Error)
    {
        hppDout(notice,"### Begin interpolate");
        try
        {
        unsigned int pathId = int(path);
        if(startState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("Start state not initialized, cannot interpolate ");
        }
        if(endState_.configuration_.rows() == 0)
        {
            throw std::runtime_error ("End state not initialized, cannot interpolate ");
        }

        if(problemSolver()->paths().size() <= pathId)
        {
            throw std::runtime_error ("No path computed, cannot interpolate ");
        }
        const affMap_t &affMap = problemSolver()->map
					<std::vector<boost::shared_ptr<model::CollisionObject> > > ();
        if (affMap.empty ())
        {
            throw hpp::Error ("No affordances found. Unable to interpolate.");
        }

        hpp::rbprm::interpolation::RbPrmInterpolationPtr_t interpolator = 
                    rbprm::interpolation::RbPrmInterpolation::create(fullBody(),startState_,endState_,problemSolver()->paths()[pathId]);
        lastStatesComputedTime_ = interpolator->Interpolate(affMap, bindShooter_.affFilter_,
                    timestep,robustnessTreshold, filterStates != 0);
		lastStatesComputed_ = TimeStatesToStates(lastStatesComputedTime_);

        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();

        res->length ((_CORBA_ULong)lastStatesComputed_.size ());
        std::size_t i=0;
        std::size_t id = 0;
        for(std::vector<State>::const_iterator cit = lastStatesComputed_.begin();
					cit != lastStatesComputed_.end(); ++cit, ++id)
        {
            /*std::cout << "ID " << id;
            cit->print();*/
            const core::Configuration_t config = cit->configuration_;
            _CORBA_ULong size = (_CORBA_ULong) config.size ();
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for (model::size_type j=0 ; j < config.size() ; ++j) {
              dofArray[j] = config [j];
            }
            (*res) [(_CORBA_ULong)i] = floats;
            ++i;
        }
        return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short AddPath(core::PathPtr_t path, core::ProblemSolverPtr_t ps)
    {
        core::PathVectorPtr_t resPath = core::PathVector::create(path->outputSize(), path->outputDerivativeSize());
        resPath->appendPath(path);
        return ps->addPath(resPath);
    }

    CORBA::Short RbprmBuilder::limbRRT(double state1, double state2, unsigned short numOptimizations) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            //create helper
//            /interpolation::TimeConstraintHelper helper(fullBody(), problemSolver()->problem());
            core::PathPtr_t path = interpolation::limbRRT(fullBody(),problemSolver()->problem(),
                                                                          lastStatesComputed_.begin()+s1,lastStatesComputed_.begin()+s2, numOptimizations);
            return AddPath(path,problemSolver());
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::limbRRTFromRootPath(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            unsigned int pathId = (unsigned int)(path);
            if(problemSolver()->paths().size() <= pathId)
            {
                throw std::runtime_error ("No path computed, cannot interpolate ");
            }
            core::PathPtr_t path = interpolation::limbRRTFromPath(fullBody(),problemSolver()->problem(), problemSolver()->paths()[pathId],
                                                                          lastStatesComputedTime_.begin()+s1,lastStatesComputedTime_.begin()+s2, numOptimizations);
            return AddPath(path,problemSolver());
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::configToPath(const hpp::floatSeqSeq& configs) throw (hpp::Error)
    {
        T_Configuration positions =  doubleDofArrayToConfig(fullBody()->device_, configs);
        core::PathVectorPtr_t res = core::PathVector::create(fullBody()->device_->configSize(),
                                                             fullBody()->device_->numberDof());
        //for(CIT_Configuration pit = positions.begin();pit != positions.end()-1; ++pit)
        for(CIT_Configuration pit = positions.begin();pit != positions.end()-200; ++pit)
        {
            res->appendPath(makePath(fullBody()->device_,problemSolver()->problem(), *pit,*(pit+1)));
        }
        return problemSolver()->addPath(res);
    }

    CORBA::Short RbprmBuilder::comRRT(double state1, double state2, unsigned short path, unsigned short numOptimizations) throw (hpp::Error)
    {
        try
        {
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);            
// temp
assert(s2 == s1 +1);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            unsigned int pathId = (unsigned int)(path);
            if(problemSolver()->paths().size() <= pathId)
            {
                throw std::runtime_error ("No path computed, cannot interpolate ");
            }
            core::PathPtr_t path = interpolation::comRRT(fullBody(),problemSolver()->problem(), problemSolver()->paths()[pathId],
                                                                          *(lastStatesComputed_.begin()+s1),*(lastStatesComputed_.begin()+s2), numOptimizations);
            return AddPath(path,problemSolver());
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    core::Configuration_t project_or_throw(rbprm::RbPrmFullBodyPtr_t fulllBody, const State& state, const fcl::Vec3f& targetCom, const bool checkCollision = false)
    {
        rbprm::projection::ProjectionReport rep;
        if(checkCollision)
            rep =rbprm::projection::projectToColFreeComPosition(fulllBody, targetCom, state);
        else
            rep= rbprm::projection::projectToComPosition(fulllBody, targetCom, state);
        core::Configuration_t res = rep.result_.configuration_;
        if(!rep.success_)
        {
            std::cout << "projection failed in project or throw " << std::endl;
            throw std::runtime_error("could not project state on COM constraint");
        }
        return res;
    }

    hpp::floatSeq* RbprmBuilder::rrt(t_rrt functor,  double state1, double state2,
                                    unsigned short cT1, unsigned short cT2, unsigned short cT3,
                                    unsigned short numOptimizations)  throw (hpp::Error)
    {
        hppDout(notice,"########## begin rrt for state "<<state1<<" ###########");
        unsigned int seed =  (unsigned int) (time(NULL)) ;
        std::cout<<"seed rrt = "<<seed<<std::endl;
        hppDout(notice,"seed rrt = "<<seed);
        srand (seed);

        try
        {
            std::vector<CORBA::Short> pathsIds;
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            hppDout(notice,"index first state = "<<s1<<" ; index second state : "<<s2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            const core::PathVectors_t& paths = problemSolver()->paths();
            if(paths.size() -1 < std::max(cT1, std::max(cT2, cT3)))
            {
                throw std::runtime_error("in comRRTFromPos, at least one com trajectory is not present in problem solver");
            }
            State& state1=lastStatesComputed_[s1], state2=lastStatesComputed_[s2];
            hppDout(notice,"start comRRTFromPos");

            State s1Bis(state1);
            hppDout(notice,"state1 = "<<model::displayConfig(state1.configuration_));
            hppDout(notice,"proj to CoM position : "<<paths[cT1]->end().head<3>());
            s1Bis.configuration_ = project_or_throw(fullBody(),s1Bis,paths[cT1]->end().head<3>(),true);
            hppDout(notice,"state1 after projection= "<<model::displayConfig(s1Bis.configuration_));

            for(std::map<std::string,bool>::const_iterator cit = s1Bis.contacts_.begin();cit!=s1Bis.contacts_.end(); ++ cit)
            {
              hppDout(notice,"contact : "<<cit->first<<" = "<<cit->second);
            }

            State s2Bis(state2);
            hppDout(notice,"state2 = "<<model::displayConfig(state2.configuration_));
            hppDout(notice,"proj to CoM position : "<<paths[cT2]->end().head<3>());
            s2Bis.configuration_ = project_or_throw(fullBody(), s2Bis,paths[cT2]->end().head<3>(), true);
            hppDout(notice,"state2 after projection= "<<model::displayConfig(s2Bis.configuration_));

            for(std::map<std::string,bool>::const_iterator cit = s2Bis.contacts_.begin();cit!=s2Bis.contacts_.end(); ++ cit)
            {
              hppDout(notice,"contact : "<<cit->first<<" = "<<cit->second);
            }

            ValidationReportPtr_t rport (ValidationReportPtr_t(new CollisionValidationReport));
            BasicConfigurationShooterPtr_t shooter = BasicConfigurationShooter::create(fullBody()->device_);
            bool found = false;
            for (int i = 0; i< 100 && !found;  ++i)
            {
                fullBody()->device_->currentConfiguration(s1Bis.configuration_);
                found =problemSolver()->problem()->configValidations()->validate(s1Bis.configuration_, rport);
                if(!found)
                {
	                std::cout << "could not project s1Bis without collision at state " << s1  << std::endl;
                    std::cout << "collission " << *rport << std::endl;
                    s1Bis.configuration_ = *shooter->shoot();
                    s1Bis.configuration_ = project_or_throw(fullBody(), s1Bis,paths[cT1]->end().head<3>());
                    std::cout << "projection succeedded " << paths[cT1]->end().head<3>() << std::endl;
                }
            }
            if(found)
                std::cout << "got out ! " << std::endl;
            bool found2 = false;
            for (int i = 0; i< 100 && found && !found2; ++i)
            {
                fullBody()->device_->currentConfiguration(s2Bis.configuration_);
                found2 =problemSolver()->problem()->configValidations()->validate(s2Bis.configuration_, rport);
                if(!found2)
                {
					std::cout << "could not project s2Bis without collision at state " << s1  << std::endl;
                    std::cout << "collission " << *rport << std::endl;
                    s2Bis.configuration_ = *shooter->shoot();
                    s2Bis.configuration_ = project_or_throw(fullBody(), s2Bis,paths[cT2]->end().head<3>());
                    std::cout << "projection succeedded " << paths[cT2]->end().head<3>() << std::endl;
                }
            }
            if(!found || !found2)
            {
                std::cout << "could not project without collision at state " << s1  << std::endl;
                throw std::runtime_error ("could not project without collision at state " + s1 );
            }

            core::PathVectorPtr_t resPath = core::PathVector::create(fullBody()->device_->configSize(), fullBody()->device_->numberDof());
            hppDout(notice,"ComRRT : projections done. begin rrt");
            try{
                hppDout(notice,"begin comRRT states 1 and 1bis");
                core::PathPtr_t p1 = interpolation::comRRT(fullBody(),problemSolver()->problem(), paths[cT1],
                        state1,s1Bis, numOptimizations,true);
                hppDout(notice,"end comRRT");
                // reduce path to remove extradof
                core::SizeInterval_t interval(0, p1->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(p1,intervals);
                resPath->appendPath(reducedPath);
                pathsIds.push_back(AddPath(p1,problemSolver()));
            }
            catch(std::runtime_error& e)
            {
                throw Error(e.what());
            }


            try{
                hppDout(notice,"begin comRRT between statebis 1 and 2");
                core::PathPtr_t p2 =(*functor)(fullBody(),problemSolver(), paths[cT2],
                    s1Bis,s2Bis, numOptimizations,true);
                if(!p2)
                    throw std::runtime_error("effectorRRT did not converge, no valid path found" );
                hppDout(notice,"end comRRT");
                pathsIds.push_back(AddPath(p2,problemSolver()));
                // reduce path to remove extradof
                core::SizeInterval_t interval(0, p2->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(p2,intervals);
                resPath->appendPath(reducedPath);
            }
            catch(std::runtime_error& e)
            {
                throw Error(e.what());
            }

            //if(s2Bis.configuration_ != state2.configuration_)
            try{
                hppDout(notice,"begin comRRT states 2bis and 2");
                core::PathPtr_t p3= interpolation::comRRT(fullBody(),problemSolver()->problem(), paths[cT3],
                        s2Bis,state2, numOptimizations,true);
                hppDout(notice,"end comRRT");
                // reduce path to remove extradof
                core::SizeInterval_t interval(0, p3->initial().rows()-1);
                core::SizeIntervals_t intervals;
                intervals.push_back(interval);
                PathPtr_t reducedPath = core::SubchainPath::create(p3,intervals);
                resPath->appendPath(reducedPath);
                pathsIds.push_back(AddPath(p3,problemSolver()));
                std::cout << "PATH 3 OK " << std::endl;
            }
            catch(std::runtime_error& e)
            {
                throw Error(e.what());
            }
            pathsIds.push_back(AddPath(resPath,problemSolver()));

            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(pathsIds.size());
            for(std::size_t i=0; i< pathsIds.size(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = pathsIds[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::comRRTFromPos(double state1,
                                               unsigned short cT1,
                                               unsigned short cT2,
                                               unsigned short cT3,
                                               unsigned short numOptimizations) throw (hpp::Error)
    {
        return rrt(&interpolation::comRRT, state1,state1+1, cT1, cT2, cT3, numOptimizations);

    }

    hpp::floatSeq* RbprmBuilder::comRRTFromPosBetweenState(double state1, double state2,
                                               unsigned short cT1,
                                               unsigned short cT2,
                                               unsigned short cT3,
                                               unsigned short numOptimizations) throw (hpp::Error)
    {
        return rrt(&interpolation::comRRT, state1,state2, cT1, cT2, cT3, numOptimizations);

    }

    hpp::floatSeq* RbprmBuilder::effectorRRTFromPosBetweenState(double state1, double state2,
                                               unsigned short cT1,
                                               unsigned short cT2,
                                               unsigned short cT3,
                                               unsigned short numOptimizations) throw (hpp::Error)
    {
        return rrt(&interpolation::effectorRRT, state1,state2, cT1, cT2, cT3, numOptimizations);

    }

    hpp::floatSeq* RbprmBuilder::effectorRRT(double state1,
                                             unsigned short cT1,
                                             unsigned short cT2,
                                             unsigned short cT3,
                                             unsigned short numOptimizations) throw (hpp::Error)
    {
        return rrt(&interpolation::effectorRRT, state1, state1+1, cT1, cT2, cT3, numOptimizations);
    }

    hpp::floatSeq* RbprmBuilder::effectorRRTFromPath(double state1,
                                                    unsigned short refpath, double path_from, double path_to,
                                                    unsigned short cT1,
                                                    unsigned short cT2,
                                                    unsigned short cT3,
                                                    unsigned short numOptimizations,
                                                    const Names_t &trackedEffector) throw (hpp::Error)
    {
        try
        {
            std::vector<CORBA::Short> pathsIds;
            std::size_t s1((std::size_t)state1), s2((std::size_t)state1+1);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            const core::PathVectors_t& paths = problemSolver()->paths();
            if(paths.size() -1 < std::max(cT1, std::max(cT2, cT3)))
            {
                throw std::runtime_error("in comRRTFromPos, at least one com trajectory is not present in problem solver");
            }
            const State& state1=lastStatesComputed_[s1], state2=lastStatesComputed_[s2];

            core::PathVectorPtr_t comPath = core::PathVector::create(3,3);
            comPath->appendPath(paths[cT1]);
            comPath->appendPath(paths[cT2]);
            comPath->appendPath(paths[cT3]);
            std::vector<std::string> trackedEffectorNames = stringConversion(trackedEffector);
            core::PathPtr_t refFullBody = problemSolver()->paths()[refpath]->extract(std::make_pair(path_from, path_to));
            core::PathPtr_t p2 =interpolation::effectorRRTFromPath(fullBody(),problemSolver(), comPath,
                    state1,state2, numOptimizations,true, refFullBody, trackedEffectorNames);
            if(!p2)
                throw std::runtime_error("effectorRRT did not converge, no valid path found between states "+std::string(""+s1) + ", " + std::string(""+s2));

            pathsIds.push_back(AddPath(p2,problemSolver()));

            // reduce path to remove extradof
            core::SizeInterval_t interval(0, p2->initial().rows()-1);
            core::SizeIntervals_t intervals;
            intervals.push_back(interval);
            PathPtr_t reducedPath = core::SubchainPath::create(p2,intervals);
            core::PathVectorPtr_t resPath = core::PathVector::create(fullBody()->device_->configSize(), fullBody()->device_->numberDof());
            resPath->appendPath(reducedPath);

            pathsIds.push_back(AddPath(resPath,problemSolver()));
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(pathsIds.size());
            for(std::size_t i=0; i< pathsIds.size(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = pathsIds[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::generateEndEffectorBezier(double state1, double state2,
    unsigned short cT)throw (hpp::Error){
        try
        {
            hppDout(notice,"Begin generateEndEffectorBezier");
            std::size_t s1((std::size_t)state1), s2((std::size_t)state2);
            hppDout(notice,"index first state = "<<s1<<" ; index second state : "<<s2);
            if(lastStatesComputed_.size () < s1 || lastStatesComputed_.size () < s2 )
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s1) + ", " + std::string(""+s2));
            }
            const core::PathVectors_t& paths = problemSolver()->paths();
            if(paths.size() -1 < cT)
            {
                throw std::runtime_error("in generateEndEffectorBezier, at least one com trajectory is not present in problem solver");
            }
            State& state1=lastStatesComputed_[s1], state2=lastStatesComputed_[s2];
            hppDout(notice,"start generateEndEffectorBezier");
            interpolation::generateEndEffectorBezier(fullBody(),problemSolver(),paths[cT],state1,state2);
            return problemSolver()->paths().size()-1;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::projectToCom(double state, const hpp::floatSeq& targetCom, unsigned short max_num_sample) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size () < state)
            {
                throw std::runtime_error ("did not find a states at indicated index: " + std::string(""+(std::size_t)(state)));
            }
            model::Configuration_t config = dofArrayToConfig (std::size_t(3), targetCom);
            fcl::Vec3f comTarget; for(int i =0; i<3; ++i) comTarget[i] = config[i];
            model::Configuration_t  res = project_or_throw(fullBody(), lastStatesComputed_[state],comTarget);
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(res.rows());
            for(std::size_t i=0; i< res.rows(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = res[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    double RbprmBuilder::setConfigAtState(unsigned short state, const hpp::floatSeq& q) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size () < state)
            {
                throw std::runtime_error ("did not find a states at indicated index: " + std::string(""+(std::size_t)(state)));
            }
            model::Configuration_t res = dofArrayToConfig (fullBody()->device_, q);
            if(lastStatesComputed_.size() <= state)
            {
                throw std::runtime_error ("Unexisting state in setConfigAtstate");
            }
            else
            {
                lastStatesComputed_[state].configuration_ = res;
                lastStatesComputedTime_[state].second.configuration_ = res;
                return 1.;
            }
            return 0.;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::getConfigAtState(unsigned short state) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size () < state)
            {
                throw std::runtime_error ("did not find a state at indicated index: " + std::string(""+(std::size_t)(state)));
            }
            model::Configuration_t res = lastStatesComputed_[state].configuration_;
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(res.rows());
            for(std::size_t i=0; i< res.rows(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = res[i];
            }
            return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::saveComputedStates(const char* outfilename) throw (hpp::Error)
    {
        std::stringstream ss;
        ss << lastStatesComputed_.size()-2 << "\n";
        std::vector<rbprm::State>::iterator cit = lastStatesComputed_.begin()+1;
        int i = 0;
        ss << i++ << " ";
        cit->print(ss);
        for(std::vector<rbprm::State>::iterator cit2 = lastStatesComputed_.begin()+2;
            cit2 != lastStatesComputed_.end()-1; ++cit2, ++cit, ++i)
        {
            cit2->robustness = stability::IsStable(this->fullBody(), *cit2);
            ss << i<< " ";
            cit2->print(ss,*cit);
        }
        std::ofstream outfile;
        outfile.open(outfilename);
        if (outfile.is_open())
        {
            outfile << ss.rdbuf();
            outfile.close();
        }
        else
        {
            std::string error("Cannot open outfile " + std::string(outfilename));
            throw Error(error.c_str());
        }
    }

    void RbprmBuilder::saveLimbDatabase(const char* limbname,const char* filepath) throw (hpp::Error)
    {
        try
        {
        std::string limbName(limbname);
        std::ofstream fout;
        fout.open(filepath, std::fstream::out | std::fstream::app);
        rbprm::saveLimbInfoAndDatabase(fullBody()->GetLimbs().at(limbName),fout);
        //sampling::saveLimbDatabase(fullBody()->GetLimbs().at(limbName)->sampleContainer_,fout);
        fout.close();
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeqSeq* RbprmBuilder::getOctreeBoxes(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try
        {
        model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
        model::Configuration_t save = fullBody()->device_->currentConfiguration();
        fullBody()->device_->currentConfiguration(config);
        fullBody()->device_->computeForwardKinematics();
        const std::map<std::size_t, fcl::CollisionObject*>& boxes =
                fullBody()->GetLimbs().at(std::string(limbName))->sampleContainer_.boxes_;
        const double resolution = fullBody()->GetLimbs().at(std::string(limbName))->sampleContainer_.resolution_;
        std::size_t i =0;
        hpp::floatSeqSeq *res;
        res = new hpp::floatSeqSeq ();
        res->length ((_CORBA_ULong)boxes.size ());
        for(std::map<std::size_t, fcl::CollisionObject*>::const_iterator cit = boxes.begin();
            cit != boxes.end();++cit,++i)
        {
            fcl::Vec3f position = (*cit->second).getTranslation();
            _CORBA_ULong size = (_CORBA_ULong) 4;
            double* dofArray = hpp::floatSeq::allocbuf(size);
            hpp::floatSeq floats (size, size, dofArray, true);
            //convert the config in dofseq
            for (model::size_type j=0 ; j < 3 ; ++j) {
                dofArray[j] = position[j];
            }
            dofArray[3] = resolution;
            (*res) [(_CORBA_ULong)i] = floats;
        }
        fullBody()->device_->currentConfiguration(save);
        fullBody()->device_->computeForwardKinematics();
        return res;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::getOctreeBox(const char* limbName, double octreeNodeId) throw (hpp::Error)
    {
        try
        {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        long ocId ((long)octreeNodeId);
        const T_Limb& limbs = fullBody()->GetLimbs();
        T_Limb::const_iterator lit = limbs.find(std::string(limbName));
        if(lit == limbs.end())
        {
            std::string err("No limb " + std::string(limbName) + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        const std::map<std::size_t, fcl::CollisionObject*>& boxes =
                fullBody()->GetLimbs().at(std::string(limbName))->sampleContainer_.boxes_;
        std::map<std::size_t, fcl::CollisionObject*>::const_iterator cit = boxes.find(ocId);
        if(cit == boxes.end())
        {
            std::stringstream ss; ss << ocId;
            std::string err("No octree node with id " + ss.str() + "was defined for robot" + fullBody()->device_->name());
            throw Error (err.c_str());
        }
        const fcl::CollisionObject* box = cit->second;
        const fcl::Vec3f& pos = box->getTransform().getTranslation();
        hpp::floatSeq* dofArray = new hpp::floatSeq();
        dofArray->length(4);
        for(std::size_t i=0; i< 3; ++i)
        {
          (*dofArray)[(_CORBA_ULong)i] = pos[i];
        }
        (*dofArray)[(_CORBA_ULong)3] = fullBody()->GetLimbs().at(std::string(limbName))->sampleContainer_.resolution_;
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    CORBA::Short RbprmBuilder::isLimbInContact(const char* limbName, double state1) throw (hpp::Error)
    {
        try
        {
            std::size_t s((std::size_t)state1);
            if(lastStatesComputed_.size () < s)
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s));
            }
            const std::map<std::string, fcl::Vec3f> & contacts = lastStatesComputed_[s].contactPositions_;
            if(contacts.find(std::string(limbName))!= contacts.end())
                return 1;
            return 0;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::isLimbInContactIntermediary(const char* limbName, double state1) throw (hpp::Error)
    {
        try
        {
            std::size_t s((std::size_t)state1);
            if(lastStatesComputed_.size () < s+1)
            {
                throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s));
            }
            const State& state1 = lastStatesComputed_[s];
            const State& state2 = lastStatesComputed_[s+1];
            bool unused;
            short unsigned cId = s;
            const State state = intermediary(state1, state2,cId,unused);
            const std::map<std::string, fcl::Vec3f> & contacts = state.contactPositions_;
            if(contacts.find(std::string(limbName))!= contacts.end())
                return 1;
            return 0;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }


    CORBA::Short  RbprmBuilder::computeIntermediary(unsigned short stateFrom, unsigned short stateTo) throw (hpp::Error)
    try
    {
        std::size_t s((std::size_t)stateFrom);
        std::size_t s2((std::size_t)stateTo);
        if(lastStatesComputed_.size () < s+1 || lastStatesComputed_.size () < s2+1)
        {
            throw std::runtime_error ("did not find a states at indicated indices: " + std::string(""+s));
        }
        const State& state1 = lastStatesComputed_[s];
        const State& state2 = lastStatesComputed_[s2];
        bool unused;
        short unsigned cId = s;
        const State state = intermediary(state1, state2,cId,unused);
        lastStatesComputed_.push_back(state);
        lastStatesComputedTime_.push_back(std::make_pair(-1., state));
        return lastStatesComputed_.size() -1;
    }
    catch(std::runtime_error& e)
    {
        throw Error(e.what());
    }

    hpp::floatSeq* RbprmBuilder::getOctreeTransform(const char* limbName, const hpp::floatSeq& configuration) throw (hpp::Error)
    {
        try{
        model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
        model::Configuration_t save = fullBody()->device_->currentConfiguration();
        fullBody()->device_->currentConfiguration(config);
        fullBody()->device_->computeForwardKinematics();
        const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody()->GetLimbs().at(std::string(limbName));
        const fcl::Transform3f transform = limb->octreeRoot();
        const fcl::Quaternion3f& quat = transform.getQuatRotation();
        const fcl::Vec3f& position = transform.getTranslation();
        hpp::floatSeq *dofArray;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(7));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        for(std::size_t i=0; i< 4; i++)
          (*dofArray)[(_CORBA_ULong)i+3] = quat [i];
        fullBody()->device_->currentConfiguration(save);
        fullBody()->device_->computeForwardKinematics();
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::computeTargetTransform(const char* limbName, const hpp::floatSeq& configuration,
                                                        const hpp::floatSeq& p_a, const hpp::floatSeq& n_a) throw (hpp::Error)
    {
        try{
        model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
        model::Configuration_t vec_conf = dofArrayToConfig (std::size_t(3), p_a);
        fcl::Vec3f p; for(int i =0; i<3; ++i) p[i] = vec_conf[i];
        vec_conf = dofArrayToConfig (std::size_t(3), n_a);
        fcl::Vec3f n; for(int i =0; i<3; ++i) n[i] = vec_conf[i];
        const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody()->GetLimbs().at(std::string(limbName));

        const fcl::Transform3f transform = projection::computeProjectionMatrix(fullBody(), limb, config, n,p);
        const fcl::Quaternion3f& quat = transform.getQuatRotation();
        const fcl::Vec3f& position = transform.getTranslation();
        hpp::floatSeq *dofArray;
        dofArray = new hpp::floatSeq();
        dofArray->length(_CORBA_ULong(7));
        for(std::size_t i=0; i< 3; i++)
          (*dofArray)[(_CORBA_ULong)i] = position [i];
        for(std::size_t i=0; i< 4; i++)
          (*dofArray)[(_CORBA_ULong)i+3] = quat [i];
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::isConfigBalanced(const hpp::floatSeq& configuration, const hpp::Names_t& contactLimbs, double robustnessTreshold) throw (hpp::Error)
    {
        try{
        rbprm::State testedState;
        model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
        model::Configuration_t save = fullBody()->device_->currentConfiguration();
        std::vector<std::string> names = stringConversion(contactLimbs);
        fullBody()->device_->currentConfiguration(config);
        fullBody()->device_->computeForwardKinematics();
        for(std::vector<std::string>::const_iterator cit = names.begin(); cit != names.end();++cit)
        {
            const hpp::rbprm::RbPrmLimbPtr_t limb =fullBody()->GetLimbs().at(std::string(*cit));
            testedState.contacts_[*cit] = true;
            testedState.contactPositions_[*cit] = limb->effector_->currentTransformation().getTranslation();
            testedState.contactRotation_[*cit] = limb->effector_->currentTransformation().getRotation();
            // normal given by effector normal
            const fcl::Vec3f normal = limb->effector_->currentTransformation().getRotation() * limb->normal_;
            testedState.contactNormals_[*cit] = normal;
            testedState.configuration_ = config;
            ++testedState.nbContacts;
        }
        fullBody()->device_->currentConfiguration(save);
        fullBody()->device_->computeForwardKinematics();
        if (stability::IsStable(fullBody(), testedState) >= robustnessTreshold)
        {
            return (CORBA::Short)(1);
        }
        else
        {
            return (CORBA::Short)(0);
        }
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    double RbprmBuilder::isStateBalanced(unsigned short stateId) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
            {
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            }
            return stability::IsStable(fullBody(),lastStatesComputed_[stateId]);
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }


    void RbprmBuilder::runSampleAnalysis(const char* analysis, double isstatic) throw (hpp::Error)
    {
        try
        {
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        std::string eval(analysis);
        if (eval == "all")
        {
            for(sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.begin();
                analysisit != analysisFactory_->evaluate_.end(); ++ analysisit)
            {
                for(T_Limb::const_iterator cit = fullBody()->GetLimbs().begin(); cit !=fullBody()->GetLimbs().end();++cit)
                {
                    sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (cit->second->sampleContainer_);
                    sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
                }
            }
        }
        else
        {
            sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.find(std::string(eval));
            if(analysisit == analysisFactory_->evaluate_.end())
            {
                std::string err("No analysis named  " + eval + "was defined for analyzing database sample");
                throw Error (err.c_str());
            }
            for(T_Limb::const_iterator cit = fullBody()->GetLimbs().begin(); cit !=fullBody()->GetLimbs().end();++cit)
            {
                sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (cit->second->sampleContainer_);
                sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
            }
        }
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::runLimbSampleAnalysis(const char* limbname, const char* analysis, double isstatic) throw (hpp::Error)
    {
        try
        {
        rbprm::sampling::ValueBound bounds;
        if(!fullBodyLoaded_)
            throw Error ("No full body robot was loaded");
        rbprm::RbPrmLimbPtr_t limb = fullBody()->GetLimb(limbname);
        std::string eval(analysis);
        if (eval == "all")
        {
            for(sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.begin();
                analysisit != analysisFactory_->evaluate_.end(); ++ analysisit)
            {
                sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (limb->sampleContainer_);
                sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
                bounds = sampleDB.valueBounds_[analysisit->first];
            }
        }
        else
        {
            sampling::T_evaluate::const_iterator analysisit = analysisFactory_->evaluate_.find(std::string(eval));
            if(analysisit == analysisFactory_->evaluate_.end())
            {
                std::string err("No analysis named  " + eval + "was defined for analyzing database sample");
                throw Error (err.c_str());
            }
            sampling::SampleDB & sampleDB =const_cast<sampling::SampleDB &> (limb->sampleContainer_);
            sampling::addValue(sampleDB, analysisit->first, analysisit->second, isstatic > 0.5, isstatic > 0.5);
            bounds = sampleDB.valueBounds_[analysisit->first];
        }
        hpp::floatSeq* dofArray = new hpp::floatSeq();
        dofArray->length(2);
        (*dofArray)[0] = bounds.first;
        (*dofArray)[1] = bounds.second;
        return dofArray;
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    hpp::floatSeq* RbprmBuilder::evaluateConfig(const hpp::floatSeq& configuration,const hpp::floatSeq& direction) throw (hpp::Error){
      if(!fullBodyLoaded_)
          throw Error ("No full body robot was loaded");
      if(fullBody()->GetLimbs().size()<= 0)
          throw Error ("No Limbs defined for this robot");
      fcl::Vec3f dir;
      for(std::size_t i =0; i <3; ++i)
      {
          dir[i] = direction[(_CORBA_ULong)i];
      }
      dir = dir.normalize();


      hpp::floatSeq* dofArray = new hpp::floatSeq();
      dofArray->length(fullBody()->GetLimbs().size());
      size_t id = 0;
      model::Configuration_t config = dofArrayToConfig (fullBody()->device_, configuration);
      for(T_Limb::const_iterator lit = fullBody()->GetLimbs().begin() ; lit != fullBody()->GetLimbs().end() ; ++lit){
        sampling::Sample sample(lit->second->limb_, lit->second->effector_, config,  lit->second->offset_,lit->second->limbOffset_, 0);
        (*dofArray)[(_CORBA_ULong)id] = lit->second->evaluate_(sample,dir,lit->second->normal_,sampling::HeuristicParam());
        hppDout(notice,"Evaluate for limb : "<<lit->second->effector_->name()<<" = "<<(*dofArray)[(_CORBA_ULong)id]);
        hppDout(notice,"eff position = "<<sample.effectorPosition_);
        hppDout(notice,"limb frame   = "<<sample.effectorPositionInLimbFrame_);
        hppDout(notice,"direction    = "<<dir);
        id++;
      }
      return dofArray;
    }



    CORBA::Short RbprmBuilder::addNewContact(unsigned short stateId, const char* limbName,
                                        const hpp::floatSeq& position, const hpp::floatSeq& normal, unsigned short max_num_sample) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            State ns = lastStatesComputed_[stateId];
            const std::string limb(limbName);
            model::Configuration_t config = dofArrayToConfig (std::size_t(3), position);
            fcl::Vec3f p; for(int i =0; i<3; ++i) p[i] = config[i];
            config = dofArrayToConfig (std::size_t(3), normal);
            fcl::Vec3f n; for(int i =0; i<3; ++i) n[i] = config[i];

            projection::ProjectionReport rep = projection::projectStateToObstacle(fullBody(),limb, fullBody()->GetLimbs().at(limb), ns, n,p);
            ValidationReportPtr_t rport (ValidationReportPtr_t(new CollisionValidationReport));
            CollisionValidationPtr_t val = fullBody()->GetCollisionValidation();
            rep.success_ =  rep.success_ &&  val->validate(rep.result_.configuration_,rport);
            size_t extraDofSize = fullBody()->device_->extraConfigSpace().dimension();
            Configuration_t extraDof = ns.configuration_.tail(extraDofSize);
            if (!rep.success_ && max_num_sample > 0)
            {
                BasicConfigurationShooterPtr_t shooter = BasicConfigurationShooter::create(fullBody()->device_);
                Configuration_t head = ns.configuration_.head<7>();
                for(std::size_t i =0; !rep.success_ && i< max_num_sample; ++i)
                {
                    ns.configuration_ = *shooter->shoot();
                    ns.configuration_.head<7>() = head;
                    rep = projection::projectStateToObstacle(fullBody(),limb, fullBody()->GetLimbs().at(limb), ns, n,p);
                    rep.success_ = rep.success_ && val->validate(rep.result_.configuration_,rport);
                }
            }
            if(rep.success_)
            {
                rep.result_.configuration_.tail(extraDofSize) = extraDof;
                rep.result_.nbContacts= rep.result_.contactNormals_.size();
                lastStatesComputed_.push_back(rep.result_);
                lastStatesComputedTime_.push_back(std::make_pair(-1., rep.result_));
                return lastStatesComputed_.size() -1;
            }
            else
                return -1;
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    CORBA::Short RbprmBuilder::removeContact(unsigned short stateId, const char* limbName) throw (hpp::Error)
    {
        try
        {
            if(lastStatesComputed_.size() <= stateId)
                throw std::runtime_error ("Unexisting state " + std::string(""+(stateId)));
            State ns = lastStatesComputed_[stateId];
            const std::string limb(limbName);
            ns.RemoveContact(limb);
            lastStatesComputed_.push_back(ns);
            lastStatesComputedTime_.push_back(std::make_pair(-1., ns));
            return lastStatesComputed_.size() -1;
        }
        catch(std::runtime_error& e)
        {
            std::cout << "ERROR " << e.what() << std::endl;
            throw Error(e.what());
        }
    }

    void RbprmBuilder::dumpProfile(const char* logFile) throw (hpp::Error)
    {
        try
        {
            #ifdef PROFILE
                RbPrmProfiler& watch = getRbPrmProfiler();
                std::ofstream fout;
                fout.open(logFile, std::fstream::out | std::fstream::app);
                std::ostream* fp = &fout;
                watch.report_all_and_count(2,*fp);
                fout.close();
            #else
                throw(std::runtime_error("PROFILE PREPOC variable undefined, cannot dump profile"));
            #endif
        }
        catch(std::runtime_error& e)
        {
            throw Error(e.what());
        }
    }

    void RbprmBuilder::SetProblemSolverMap (hpp::corbaServer::ProblemSolverMapPtr_t psMap)
    {
        psMap_ = psMap;
        //bind shooter creator to hide problem as a parameter and respect signature
        //initNewProblemSolver();
    }

    void RbprmBuilder::initNewProblemSolver()
    {
        //bind shooter creator to hide problem as a parameter and respect signature

        // add rbprmshooter
        bindShooter_.problemSolver_ = problemSolver();
        problemSolver()->add<core::ConfigurationShooterBuilder_t>("RbprmShooter",
                                                   boost::bind(&BindShooter::create, boost::ref(bindShooter_), _1));
        problemSolver()->add<core::PathValidationBuilder_t>("RbprmPathValidation",
                                                   boost::bind(&BindShooter::createPathValidation, boost::ref(bindShooter_), _1, _2));
        problemSolver()->add<core::PathValidationBuilder_t>("RbprmDynamicPathValidation",
                                                   boost::bind(&BindShooter::createDynamicPathValidation, boost::ref(bindShooter_), _1, _2));
        problemSolver()->add<core::PathPlannerBuilder_t>("DynamicPlanner",DynamicPlanner::createWithRoadmap);
        problemSolver()->add <core::SteeringMethodBuilder_t> ("RBPRMKinodynamic", SteeringMethodKinodynamic::create);
        problemSolver()->add <core::PathOptimizerBuilder_t> ("RandomShortcutDynamic", RandomShortcutDynamic::create);
        problemSolver()->add <core::PathOptimizerBuilder_t> ("OrientedPathOptimizer", OrientedPathOptimizer::create);

    }



    Names_t* RbprmBuilder::getAllLimbsNames()throw (hpp::Error)
    {
      if(!fullBodyLoaded_){
        throw std::runtime_error ("fullBody not loaded");
      }
      std::vector<std::string> names = rbprm::interpolation::extractEffectorsName(fullBody()->GetLimbs());
      CORBA::ULong size = (CORBA::ULong) names.size ();
      char** nameList = Names_t::allocbuf(size);
      Names_t *limbsNames = new Names_t (size,size,nameList);
      for (std::size_t i = 0 ; i < names.size() ; ++i){
        nameList[i] = (char*) malloc (sizeof(char)*(names[i].length ()+1));
        strcpy (nameList [i], names[i].c_str ());
      }
      return limbsNames;
    }

    bool RbprmBuilder::areKinematicsConstraintsVerified(const hpp::floatSeq &point)throw (hpp::Error){
        if(!fullBodyLoaded_){
          throw std::runtime_error ("fullBody not loaded");
        }
        Configuration_t pt_config = dofArrayToConfig(3,point);
        fcl::Vec3f pt(pt_config[0],pt_config[1],pt_config[2]);
        bool success(true);
        bool successLimb;
        hppDout(notice,"Test kinematic constraint for point : "<<pt);
        for(CIT_Limb lit = fullBody()->GetLimbs().begin() ; lit != fullBody()->GetLimbs().end() ; ++lit){
            hppDout(notice,"for limb : "<<lit->first);
            if(lit->second->kinematicConstraints_.first.size()==0){
                hppDout(notice,"Kinematics constraints not initialized");
            }else{
                successLimb = rbprm::reachability::verifyKinematicConstraints(lit->second->kinematicConstraints_,lit->second->effector_->currentTransformation(),pt);
                hppDout(notice,"kinematic constraints verified : "<<successLimb);
                success = success && successLimb;
            }
        }
        return success;
    }

    bool RbprmBuilder::areKinematicsConstraintsVerifiedForState(unsigned short stateId,const hpp::floatSeq &point)throw (hpp::Error){
        if(!fullBodyLoaded_){
          throw std::runtime_error ("fullBody not loaded");
        }
        if(stateId >= lastStatesComputed_.size()){
            throw std::runtime_error ("Unexisting state ID");
        }
        Configuration_t pt_config = dofArrayToConfig(3,point);
        fcl::Vec3f pt(pt_config[0],pt_config[1],pt_config[2]);
        return reachability::verifyKinematicConstraints(fullBody(),lastStatesComputed_[stateId],pt);
    }


    bool RbprmBuilder::isReachableFromState(unsigned short stateFrom,unsigned short stateTo)throw (hpp::Error){
        if(!fullBodyLoaded_){
          throw std::runtime_error ("fullBody not loaded");
        }
        if(stateTo >= lastStatesComputed_.size() || stateFrom >= lastStatesComputed_.size()){
            throw std::runtime_error ("Unexisting state ID");
        }
        reachability::Result res = reachability::isReachable(fullBody(),lastStatesComputed_[stateFrom],lastStatesComputed_[stateTo]);
        return (res.success());
    }



    hpp::floatSeq* RbprmBuilder::isDynamicallyReachableFromState(unsigned short stateFrom, unsigned short stateTo,bool addPathPerPhase, const hpp::floatSeq &timings, unsigned short numPointPerPhase )throw (hpp::Error){
        if(!fullBodyLoaded_){
          throw std::runtime_error ("fullBody not loaded");
        }
        if(stateTo >= lastStatesComputed_.size() || stateFrom >= lastStatesComputed_.size()){
            throw std::runtime_error ("Unexisting state ID");
        }
        reachability::Result res;
        if(timings.length()>0){
            std::vector<double> Ts;

            Configuration_t t_config = dofArrayToConfig(timings.length(),timings);
            for(size_t i = 0 ; i < timings.length() ; ++i)
                Ts.push_back(t_config[i]);
            res = reachability::isReachableDynamic(fullBody(),lastStatesComputed_[stateFrom],lastStatesComputed_[stateTo],false,Ts,numPointPerPhase);
        }else{
            res = reachability::isReachableDynamic(fullBody(),lastStatesComputed_[stateFrom],lastStatesComputed_[stateTo],false);
        }
        if (res.success()){
            std::vector<int> ids;
            core::PathVectorPtr_t pathVector_full = core::PathVector::create(res.path_->outputSize(),res.path_->outputDerivativeSize());
            pathVector_full->appendPath(res.path_);
            ids.push_back(problemSolver()->addPath(pathVector_full));
            if(addPathPerPhase){
                for(size_t i = 0 ; i < res.timings_.size() ; ++i){
                    core::PathVectorPtr_t pathVector = core::PathVector::create(res.path_->outputSize(),res.path_->outputDerivativeSize());
                    pathVector->appendPath(res.pathPerPhases_[i]);
                    ids.push_back(problemSolver()->addPath(pathVector));
                }
            }
            // convert vector of int to floatSeq :
            hpp::floatSeq* dofArray = new hpp::floatSeq();
            dofArray->length(ids.size());
            for(std::size_t i=0; i< ids.size(); ++i)
            {
              (*dofArray)[(_CORBA_ULong)i] = ids[i];
            }
            return dofArray;
        }else
            return new hpp::floatSeq();
    }


    } // namespace impl
  } // namespace rbprm
} // namespace hpp
