/*
 *  Copyright 2007 LAAS-CNRS
 *
 *  Authors: Florent Lamiraux, Luis Delgado
 */

#include <cerrno>
#include <iostream>

#include <boost/foreach.hpp>

#include <KineoModel/kppFreeFlyerJointComponent.h>
#include <KineoModel/kppAnchorJointComponent.h>
#include <KineoModel/kppRotationJointComponent.h>
#include <KineoModel/kppTranslationJointComponent.h>
#include <KineoModel/kppSolidComponentRef.h>
#include <KineoModel/kppSteeringMethodComponent.h>

#include <hpp/kwsio/configuration.hh>
#include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/util/debug.hh>

#include "hpp/model/device.hh"
#include "hpp/model/exception.hh"
#include "hpp/model/joint.hh"
#include <hpp/model/body-distance.hh>

namespace hpp {
  namespace model {

    impl::ObjectFactory Device::objectFactory_;

    Device::Device()
      : impl::DynamicRobot(objectFactory ()),
	CkppDeviceComponent (),
	bodyDistances_ (),
	weakPtr_ ()
    {
      CkitNotificator::defaultNotificator()->subscribe<Device>
	(CkppComponent::DID_INSERT_CHILD, this,
	 &Device::componentDidInsertChild);
      CkitNotificator::defaultNotificator()->subscribe<Device>
	(CkppComponent::WILL_INSERT_CHILD, this,
	 &Device::componentWillInsertChild);
    }


    // ========================================================================

    Device::~Device()
    {
    }

    // ========================================================================

    impl::ObjectFactory* Device::objectFactory()
    {
      return &Device::objectFactory_;
    }

    // ========================================================================

    DeviceShPtr Device::create(std::string name)
    {
      Device *hppDevice = new Device();
      DeviceShPtr hppDeviceShPtr(hppDevice);

      if (hppDevice->init(hppDeviceShPtr, name) != KD_OK) {
	hppDeviceShPtr.reset();
      }
      CkwsValidatorSetConstShPtr validators
	(hppDevice->directPathValidators ());
      std::size_t n = validators->count ();
      hppDout (info, "Nb direct path validators: " << n);
      for (std::size_t i=0; i<n; ++i) {
	CkwsValidatorShPtr validator (validators->at (i));
	hppDout (info, "Direct path validator: " << validator->name ());
      }
      return hppDeviceShPtr;
    }

    // ========================================================================

    DeviceShPtr Device::createCopy(const DeviceShPtr& device)
    {
      Device* ptr = new Device(*device);
      DeviceShPtr deviceShPtr(ptr);

      if(KD_OK != ptr->init(deviceShPtr, device))	{
	deviceShPtr.reset();
      }

      return deviceShPtr;
    }

    // ========================================================================

    CkwsDeviceShPtr Device::clone() const
    {
      return Device::createCopy(weakPtr_.lock());
    }

    // ========================================================================

    CkppComponentShPtr Device::cloneComponent() const
    {
      return Device::createCopy(weakPtr_.lock());
    }

    // ========================================================================

    bool Device::isComponentClonable() const
    {
      return true;
    }

    // ========================================================================

    ktStatus Device::init(const DeviceWkPtr& weakPtr,
			  const std::string &name)
    {
      ktStatus success = CkppDeviceComponent::init(weakPtr, name);

      if(KD_OK == success) {
	hppDout (info, "CkppDeviceComponent::init succeeded.");
	weakPtr_ = weakPtr;
      } else {
	hppDout(error, "failed to initialize CkppDeviceComponent.");
      }
      return success;
    }

    // ========================================================================

    ktStatus Device::init(const DeviceWkPtr& weakPtr,
			  const DeviceShPtr& device)
    {
      ktStatus  success = CkppDeviceComponent::init(weakPtr, device);

      if(KD_OK == success) {
	weakPtr_ = weakPtr;
      }

      return success;
    }

    // ========================================================================

    bool Device::initialize ()
    {
      JointShPtr rootJoint = getRootJoint();
      initializeKinematicChain(rootJoint);
      impl::DynamicRobot::rootJoint(*(rootJoint->jrlJoint()));
      if (!impl::DynamicRobot::initialize()) {
	throw Exception("Failed to initialize impl::DynamicRobot");
      }
      return true;
    }

    // ========================================================================

    void Device::initializeKinematicChain(JointShPtr joint)
    {
      joint->createDynamicPart();
      for (unsigned int iChild=0; iChild < joint->countChildJoints();
	   iChild++) {
	JointShPtr child = joint->childJoint(iChild);
	initializeKinematicChain(child);
	joint->jrlJoint()->addChildJoint(*(child->jrlJoint()));
      }
    }

    // ========================================================================
    ktStatus
    Device::axisAlignedBoundingBox (double& xMin, double& yMin, double& zMin,
				    double& xMax, double& yMax, double& zMax)
      const
    {

      TBodyVector bodyVector;
      this->getBodyVector(bodyVector);
      unsigned int j=bodyVector.size();

      xMin=9999999;
      yMin=9999999;
      zMin=9999999;
      xMax=-9999999;
      yMax=-9999999;
      zMax=-9999999;

      for(unsigned int i=0; i<j; i++)
	{
	  /*Dynamic cast*/
	  CkwsKCDBodyAdvancedShPtr a;
	  a=KIT_DYNAMIC_PTR_CAST(CkwsKCDBodyAdvanced, bodyVector[i]);
	  if(!a)
	    {
	      hppDout(error, ":axisAlignedBoundingBox: Error, "
		      "the CkwsBody not of type CkwsKCDBodyAdvanced");
	      return KD_ERROR;
	    }
	  computeBodyBoundingBox(a,xMin,yMin,zMin,xMax,yMax,zMax);
	}
      return KD_OK;
    }

    // ========================================================================

    ktStatus Device::ignoreDeviceForCollision (DeviceShPtr device )
    {
      ktStatus status = KD_OK ;

      //
      // Get body vector of device
      //
      CkwsDevice::TBodyVector deviceBodyVector;
      device->getBodyVector(deviceBodyVector) ;

      //
      // For each body of device,
      //
      for (unsigned int bodyId = 0 ;  bodyId < deviceBodyVector.size();
	   bodyId++)  {
	CkwsKCDBodyAdvancedShPtr kcdBody;
	if (kcdBody = KIT_DYNAMIC_PTR_CAST(CkwsKCDBodyAdvanced,
					   deviceBodyVector[bodyId])) {
	  //
	  // get the inner object list of the body
	  //
	  std::vector< CkcdObjectShPtr > kcdBodyInnerObjects =
	    kcdBody->mobileObjects() ;

	  //
	  // and deactivate each object for collision analysis. 
	  //
	  for (unsigned int objectId =0 ; objectId < kcdBodyInnerObjects.size() ;
	       objectId ++) {
	    CkcdObjectShPtr kcdObject = kcdBodyInnerObjects[objectId] ;
	    kcdObject->activation (false) ;
	  }
	}
	else {
	  hppDout(error, ":ignoreDeviceForCollision : body is not KCD body.");
	  return KD_ERROR ;
	}
      }

      return status ;
    }

    // ========================================================================

    void Device::computeBodyBoundingBox(const CkwsKCDBodyAdvancedShPtr& body,
					double& xMin, double& yMin,
					double& zMin, double& xMax,
					double& yMax, double& zMax) const
    {
      std::vector<CkcdObjectShPtr> listObject = body->mobileObjects();
      unsigned int j= listObject.size();
      for(unsigned int i=0; i<j; i++)
	{
	  ckcdObjectBoundingBox(listObject[i],xMin,yMin,zMin,xMax,yMax,zMax);
	}
    }

    // ========================================================================

    void Device::ckcdObjectBoundingBox(const CkcdObjectShPtr& object,
				       double& xMin, double& yMin,
				       double& zMin, double& xMax,
				       double& yMax, double& zMax) const
    {
      kcdReal x,y,z;

      // If the object has no bounding box, ignore it
      if (!object->boundingBox()) {
	return;
      }
      object->boundingBox()->getHalfLengths(x, y, z) ;

      /*Matrices absolute et relative*/
      CkcdMat4 matrixAbsolutePosition;
      CkcdMat4 matrixRelativePosition;
      object->getAbsolutePosition(matrixAbsolutePosition);
      object->boundingBox()->getRelativePosition(matrixRelativePosition);

      /*Creer les points et change position points*/
      CkcdMat4 matrixChangePosition =
	matrixAbsolutePosition*matrixRelativePosition;

      CkcdPoint position[8];

      position[0]=matrixChangePosition*CkcdPoint( x, y, z);
      position[1]=matrixChangePosition*CkcdPoint( x, y,-z);
      position[2]=matrixChangePosition*CkcdPoint( x,-y, z);
      position[3]=matrixChangePosition*CkcdPoint(-x, y, z);
      position[4]=matrixChangePosition*CkcdPoint( x,-y,-z);
      position[5]=matrixChangePosition*CkcdPoint(-x,-y, z);
      position[6]=matrixChangePosition*CkcdPoint(-x, y,-z);
      position[7]=matrixChangePosition*CkcdPoint(-x,-y,-z);

      for(int i=0; i<8; i++)
	{
	  if((position[i])[0]<xMin)
	    {
	      xMin=(position[i])[0];
	    }
	  if((position[i])[1]<yMin)
	    {
	      yMin=(position[i])[1];
	    }
	  if((position[i])[2]<zMin)
	    {
	      zMin=(position[i])[2];
	    }
	  if((position[i])[0]>xMax)
	    {
	      xMax=(position[i])[0];
	    }
	  if((position[i])[1]>yMax)
	    {
	      yMax=(position[i])[1];
	    }
	  if((position[i])[2]>zMax)
	    {
	      zMax=(position[i])[2];
	    }
	}

    }

    // ========================================================================

    ktStatus Device::addObstacle(const CkcdObjectShPtr& object,
				 bool distanceComputation)
    {
      // Loop over bodies of robot.
      BOOST_FOREACH(BodyDistanceShPtr bodyDistance, bodyDistances_)
	{
	  bodyDistance->addOuterObject(object, distanceComputation);
	}
      return KD_OK;
    }

    // ========================================================================

    void Device::setRootJoint(JointShPtr joint)
    {
      hppDout(info, "Root joint = " << 
	      KIT_DYNAMIC_PTR_CAST(CkppJointComponent, joint)->name());
      /*
	Set joint as Kineo root joint
      */
      CkppDeviceComponent::rootJointComponent(joint->kppJoint());

      /*
	Set joint as robotDynamics root joint
      */
      impl::DynamicRobot::rootJoint(*(joint->jrlJoint()));
    }

    // ========================================================================

    JointShPtr Device::getRootJoint()
    {
      /*
	Get CkppJointComponent root joint
      */
      CkppJointComponentShPtr kppJointComponent = rootJointComponent();
      JointShPtr joint = KIT_DYNAMIC_PTR_CAST(Joint, kppJointComponent);
      return joint;
    }

    // ========================================================================

    const std::vector<BodyDistanceShPtr>& Device::bodyDistances () const
    {
      return bodyDistances_;
    }

    // ========================================================================

    ktStatus Device::addBodyDistance (const BodyDistanceShPtr& bodyDistance)
    {
      if (!bodyDistance)
	{
	  hppDout (error, "Null pointer to body distance object.");
	  return KD_ERROR;
	}
      else
	{
	  bodyDistances_.push_back (bodyDistance);
	  return KD_OK;
	}
    }

    // ========================================================================

    bool Device::kwsToJrlDynamicsDofValues(const std::vector<double>&
					   kwsDofVector,
					   vectorN& outJrlDynamicsDofVector)
    {
      // Count the number of extra dofs of the CkppDeviceComponent
      // since the first degrees of freedom of kwsDofVector correspond
      // to these extra-dofs.
      unsigned int rankInCkwsConfig =
	CkwsDevice::rootJoint ()->customSubspace ()->size ();
      std::vector< CkppJointComponentShPtr > kppJointVector;
      getJointComponentVector(kppJointVector);

      // Output vectors should be of right size
      KWS_PRECONDITION(outJrlDynamicsDofVector.size() == numberDof());

      /// Loop over CkppDeviceComponent joints
      for (unsigned int iKppJoint=0; iKppJoint < kppJointVector.size();
	   iKppJoint++) {
	CkppJointComponentShPtr kppJoint = kppJointVector[iKppJoint];
	JointShPtr joint = KIT_DYNAMIC_PTR_CAST(Joint, kppJoint);
	KIT_ASSERT(joint);
	/// Get associated CjrlJoint
	CjrlJoint* jrlJoint = joint->jrlJoint();

	// Do nothing if it is an anchor joint.
	if (jrlJoint->numberDof () == 0)
	  continue;

	unsigned int jrlRankInConfig = jrlJoint->rankInConfiguration();

	hppDout(info, "iKppJoint=" << kppJoint->name()
		<< " jrlRankInConfig=" << jrlRankInConfig);

#ifdef HPP_DEBUG
	///Check rank in configuration wrt  dimension.
	unsigned int jointDim = kppJoint->countDofComponents();
	if (jrlRankInConfig+jointDim > kwsDofVector.size()) {
	  hppDout(error, "rank in configuration is "
		  "more than configuration dimension(rank = "
		  << jrlRankInConfig << ", dof = " << jointDim << ")."
		  << std::endl <<
		  ":kwsToJrlDynamicsDofValues:   vectorN: "
		  << outJrlDynamicsDofVector);
	  throw Exception("Error in configuration conversion.");
	}
#endif
	/*
	  Cast joint into one of the possible types
	*/
	if (CkppFreeFlyerJointComponentShPtr jointFF =
	    KIT_DYNAMIC_PTR_CAST(CkppFreeFlyerJointComponent,
				 kppJoint)) {
	  // Translations along x, y, z
	  outJrlDynamicsDofVector[jrlRankInConfig] =
	    kwsDofVector[rankInCkwsConfig];
	  outJrlDynamicsDofVector[jrlRankInConfig+1] =
	    kwsDofVector[rankInCkwsConfig+1];
	  outJrlDynamicsDofVector[jrlRankInConfig+2] =
	    kwsDofVector[rankInCkwsConfig+2];
	  double rx = kwsDofVector[rankInCkwsConfig+3];
	  double ry = kwsDofVector[rankInCkwsConfig+4];
	  double rz = kwsDofVector[rankInCkwsConfig+5];
	  // Convert KineoWorks rotation angles to roll, pitch, yaw
	  double roll, pitch, yaw;
	  YawPitchRollToRollPitchYaw(rx, ry, rz, roll, pitch, yaw);
	  outJrlDynamicsDofVector[jrlRankInConfig+3] = roll;
	  outJrlDynamicsDofVector[jrlRankInConfig+4] = pitch;
	  outJrlDynamicsDofVector[jrlRankInConfig+5] = yaw;
	  hppDout(info, "Joint value: "
		  << outJrlDynamicsDofVector[jrlRankInConfig]
		  << ", "
		  << outJrlDynamicsDofVector[jrlRankInConfig+1] << ", "
		  << outJrlDynamicsDofVector[jrlRankInConfig+2] << ", "
		  << outJrlDynamicsDofVector[jrlRankInConfig+3] << ", "
		  << outJrlDynamicsDofVector[jrlRankInConfig+4] << ", "
		  << outJrlDynamicsDofVector[jrlRankInConfig+5]);
	  rankInCkwsConfig += 6;
	}
	else if(CkppRotationJointComponentShPtr jointRot =
		KIT_DYNAMIC_PTR_CAST(CkppRotationJointComponent,
				     kppJoint)) {
	  outJrlDynamicsDofVector[jrlRankInConfig] =
	    kwsDofVector[rankInCkwsConfig];
	  hppDout(info, "Joint value: " <<
		  outJrlDynamicsDofVector[jrlRankInConfig]);
	  rankInCkwsConfig ++;
	}
	else if(CkppTranslationJointComponentShPtr jointTrans =
		KIT_DYNAMIC_PTR_CAST(CkppTranslationJointComponent,
				     kppJoint)) {
	  outJrlDynamicsDofVector[jrlRankInConfig] =
	    kwsDofVector[rankInCkwsConfig];
	  hppDout(info, "Joint value: " <<
		  outJrlDynamicsDofVector[jrlRankInConfig]);
	  rankInCkwsConfig ++;
	}
	else if(CkppAnchorJointComponentShPtr jointAnchor =
		KIT_DYNAMIC_PTR_CAST(CkppAnchorJointComponent,
				     kppJoint)) {
	  // do nothing
	}
	else {
	  hppDout(error, "unknown type of joint.");
	  hppDout(error, "  vectorN: " <<
		  outJrlDynamicsDofVector[jrlRankInConfig]);
	  throw Exception("unknow joint type");
	}
      }

      hppDout(info, "hppSetCurrentConfig: outJrlDynamicsDofVector = "
	      << outJrlDynamicsDofVector);
      return true;
    }


    // ========================================================================

    bool
    Device::jrlDynamicsToKwsDofValues(const vectorN& inJrlDynamicsDofVector,
				      std::vector<double>& outKwsDofVector)
    {
      /// Count the number of extra dofs of the CkppDeviceComponent.
      unsigned int rankInDofValues =
	CkwsDevice::rootJoint ()->customSubspace ()->size ();

      std::vector< CkppJointComponentShPtr > kppJointVector;
      getJointComponentVector(kppJointVector);

      /// Output vectors should be of right size
      KWS_PRECONDITION(outKwsDofVector.size() == countDofs());

      /// Loop over CkppDeviceComponent joints
      for (unsigned int iKppJoint=0; iKppJoint < kppJointVector.size();
	   iKppJoint++) {
	CkppJointComponentShPtr kppJoint = kppJointVector[iKppJoint];
#ifdef HPP_DEBUG
	unsigned int jointDim = kppJoint->countDofComponents();
#endif
	JointShPtr joint = KIT_DYNAMIC_PTR_CAST(Joint, kppJoint);
	KIT_ASSERT(joint);
	/// Get associated CjrlJoint
	CjrlJoint* jrlJoint = joint->jrlJoint();

	// Do nothing if it is an anchor joint.
	if (jrlJoint->numberDof () == 0)
	  continue;

	unsigned int jrlRankInConfig = jrlJoint->rankInConfiguration();

	hppDout(info, ":jrlDynamicsToKwsDofValues: iKppJoint="
		<< kppJoint->name()
		<< " jrlRankInConfig= " << jrlRankInConfig);

#ifdef HPP_DEBUG
	/// Check rank in configuration wrt  dimension.
	if (jointDim > 0 &&
	    jrlRankInConfig+jointDim > inJrlDynamicsDofVector.size()) {
	  hppDout(error, "rank in configuration is more than configuration "
		  "dimension(rank = "
		  << jrlRankInConfig << ", dof = " << jointDim << ").");
	  throw Exception("rank in configuration is more than configuration");
	}
#endif
	/*
	  Cast joint into one of the possible types
	*/
	if (CkppFreeFlyerJointComponentShPtr jointFF =
	    KIT_DYNAMIC_PTR_CAST(CkppFreeFlyerJointComponent,
				 kppJoint)) {
	  // Translations along x, y, z
	  outKwsDofVector[rankInDofValues  ] =
	    inJrlDynamicsDofVector[jrlRankInConfig  ];
	  outKwsDofVector[rankInDofValues+1] =
	    inJrlDynamicsDofVector[jrlRankInConfig+1];
	  outKwsDofVector[rankInDofValues+2] =
	    inJrlDynamicsDofVector[jrlRankInConfig+2];

	  double roll = inJrlDynamicsDofVector[jrlRankInConfig+3];
	  double pitch = inJrlDynamicsDofVector[jrlRankInConfig+4];
	  double yaw = inJrlDynamicsDofVector[jrlRankInConfig+5];

	  // Convert KineoWorks rotation angles to roll, pitch, yaw
	  double rx, ry, rz;
	  RollPitchYawToYawPitchRoll(roll, pitch, yaw, rx, ry, rz);

	  outKwsDofVector[rankInDofValues+3] = rx;
	  outKwsDofVector[rankInDofValues+4] = ry;
	  outKwsDofVector[rankInDofValues+5] = rz;

	  rankInDofValues+= 6;
	}
	else if(CkppRotationJointComponentShPtr jointRot =
		KIT_DYNAMIC_PTR_CAST(CkppRotationJointComponent,
				     kppJoint)) {
	  outKwsDofVector[rankInDofValues] =
	    inJrlDynamicsDofVector[jrlRankInConfig];
	  rankInDofValues++;
	}
	else if(CkppTranslationJointComponentShPtr jointTrans =
		KIT_DYNAMIC_PTR_CAST(CkppTranslationJointComponent,
				     kppJoint)) {
	  outKwsDofVector[rankInDofValues] =
	    inJrlDynamicsDofVector[jrlRankInConfig];
	  rankInDofValues++;
	}
	else if(CkppAnchorJointComponentShPtr jointAnchor =
		KIT_DYNAMIC_PTR_CAST(CkppAnchorJointComponent, kppJoint)){
	  // do nothing
	}
	else {
	  hppDout(error, "unknown type of joint.");
	  hppDout(error, "  vectorN: " <<
		  inJrlDynamicsDofVector);
	  throw Exception("unknow joint type");
	}
      }
      return true;
    }

    // ========================================================================

    bool Device::hppSetCurrentConfig(const CkwsConfig& config,
				     EwhichPart updateWhat)
    {
      bool updateGeom = (updateWhat == GEOMETRIC || updateWhat == BOTH);
      bool updateDynamic = (updateWhat == DYNAMIC || updateWhat == BOTH);

      if (updateGeom) {
	hppDout(info, "updating geometric part: config = " << config);


	if (CkppDeviceComponent::setCurrentConfig(config) != KD_OK) {
	  hppDout(error, "failed to set configuration of geometric part.");
	  throw("failed to set configuration of geometric part.");
	}
      }
      if (updateDynamic) {
	// Allocate a vector for CjrldynamicRobot configuration.
	// Dimension of vector is size of dynamic part of robot.
	MAL_VECTOR_DIM(jrlConfig, double, numberDof());
	std::vector<double> kwsDofVector;
	config.getDofValues(kwsDofVector);
	kwsToJrlDynamicsDofValues(kwsDofVector, jrlConfig);

	if (!currentConfiguration(jrlConfig)) {
	  hppDout(error, "failed to set configuration of dynamic part.");
	  throw Exception("failed to set configuration of dynamic part.");
	}
	if (!computeForwardKinematics()) {
	  hppDout(error, "failed to compute forward kinematics.");
	  throw Exception("failed to compute forward kinematics.");
	}
      }
      return true;
    }

    // ========================================================================

    bool Device::hppSetCurrentConfig(const vectorN& config,
				     EwhichPart updateWhat)
    {
      bool updateGeom = (updateWhat == GEOMETRIC || updateWhat == BOTH);
      bool updateDynamic = (updateWhat == DYNAMIC || updateWhat == BOTH);

      if (updateDynamic) {
	hppDout(info, "updating dynamic part: config = " << config);
	if (!currentConfiguration(config)) {
	  throw Exception("failed to set configuration of dynamic part.");
	}
	if (!computeForwardKinematics()) {
	  throw Exception("failed to compute forward kinematics.");
	}
      }
      if (updateGeom) {
	std::vector<double> dofValues(countDofs());
	this->getCurrentDofValues(dofValues);
	jrlDynamicsToKwsDofValues(config, dofValues);

	if (CkppDeviceComponent::setCurrentDofValues(dofValues) != KD_OK) {
	  throw("failed to set configuration of geometric part.");
	}
      }
      return true;
    }

    void
    Device::RollPitchYawToYawPitchRoll(const double& inRx, const double& inRy,
				       const double& inRz, double& outRx,
				       double& outRy, double& outRz)
    {
      const double cRx=cos(inRx);
      const double sRx=sin(inRx);
      const double cRy=cos(inRy);
      const double sRy=sin(inRy);
      const double cRz=cos(inRz);
      const double sRz=sin(inRz);
      const double r00 = cRy*cRz;
      const double r01 = sRx*sRy*cRz - cRx*sRz;
      double r02 = cRx*sRy*cRz + sRx*sRz;
      const double r10 = cRy*sRz;
      const double r11 = sRx*sRy*sRz + cRx*cRz;
      const double r12 = cRx*sRy*sRz - sRx*cRz;
      const double r22 = cRx*cRy;

      // make sure that r02 is in [-1,1]
      if (r02 < -1.)
	r02 = -1.;
      else if (r02 > 1.)
	r02 = 1.;

      errno=0;
      outRy = asin(r02);
      // Check that asin call was successful.
      assert(errno == 0);

      double cosOutRy = cos(outRy);

      if(fabs(cosOutRy) > 1e-6) {
	double cosOutRx =  r22 / cosOutRy;
	double sinOutRx = -r12 / cosOutRy;

	outRx = atan2(sinOutRx, cosOutRx);

	double cosOutRz =  r00 / cosOutRy;
	double sinOutRz = -r01 / cosOutRy;
	outRz = atan2(sinOutRz, cosOutRz);
      }
      else {
	outRx = 0.;

	double cosOutRz = r11;
	double sinOutRz = r10;
	outRz = atan2(sinOutRz, cosOutRz);
      }
    }

    void
    Device::YawPitchRollToRollPitchYaw(const double& inRx, const double& inRy,
				       const double& inRz, double& outRx,
				       double& outRy, double& outRz)
    {
      const double cRx = cos(inRx);
      const double sRx = sin(inRx);
      const double cRy = cos(inRy);
      const double sRy = sin(inRy);
      const double cRz  =cos(inRz);
      const double sRz = sin(inRz);
      const double r00 = cRz * cRy;
      const double r01 = -sRz * cRy;
      const double r10 = cRz * sRy * sRx + sRz * cRx;
      const double r11 = cRz * cRx - sRz * sRy * sRx;
      double r20 = sRz * sRx - cRz * sRy * cRx;
      const double r21 = sRz * sRy * cRx + cRz * sRx;
      const double r22 = cRx * cRy;

      // make sure all values are in [-1,1]
      // as trigonometric functions would
      // fail when given values such as 1.00000001
      if (r20 < -1.)
	r20 = -1.;
      else if (r20 > 1.)
	r20 = 1.;

      errno=0;
      outRy = -asin(r20);
      // Check that asin call was successful.
      assert(errno == 0);

      double cosOutRy = cos(outRy);

      if (fabs(cosOutRy) > 1e-6) {
	double sinOutRx = r21 / cosOutRy;
	double cosOutRx = r22 / cosOutRy;
	outRx = atan2(sinOutRx, cosOutRx);

	double cosOutRz = r00 / cosOutRy;
	double sinOutRz = r10 / cosOutRy;
	outRz = atan2(sinOutRz, cosOutRz);
      } else {
	outRx = 0.;

	double cosOutRz = r11;
	double sinOutRz = -r01;
	outRz = atan2(sinOutRz, cosOutRz);
      }
    }

    // ======================================================================

    void Device::
    componentWillInsertChild(const CkitNotificationConstShPtr& notification)
    {
      JointShPtr parentJoint, childJoint;
      CkppSolidComponentShPtr childSolid;
      CkppSolidComponentRefShPtr childSolidRef;
      DeviceShPtr device;

      CkppComponentShPtr parent(notification->objectShPtr<CkppComponent>());
      CkppComponentShPtr child(notification->shPtrValue<CkppComponent>
			       (CkppComponent::CHILD_KEY));
      // Detect insertion of a solid component
      if ((parentJoint = KIT_DYNAMIC_PTR_CAST(Joint, parent)) &&
	  (childSolid = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, child))) {
	parentJoint->insertBody();
	hppDout(info, "WILL_INSERT_SOLID: parent joint = "
		<< parent->name() << ", solid component = " << child->name());
      }
      // Detect insertion of a reference to a solid component
      else if ((parentJoint = KIT_DYNAMIC_PTR_CAST(Joint, parent)) &&
	       (childSolidRef = KIT_DYNAMIC_PTR_CAST(CkppSolidComponentRef,
						     child))) {
	parentJoint->insertBody();
	hppDout(info, "WILL_INSERT_SOLID_REF: parent joint = "
		<< parent->name() << ", solid component ref = "
		<< child->name());
      } 
      else if (childJoint = KIT_DYNAMIC_PTR_CAST(Joint, child)) {
	// detect insertion of root joint
	if (device = KIT_DYNAMIC_PTR_CAST(Device, parent)) {
	  device->impl::DynamicRobot::rootJoint(*(childJoint->jrlJoint()));
	  hppDout(info, "WILL_INSERT_JOINT: parent device = "
		  << parent->name() << ", joint = " << child->name());
	} 
	// Detect insertion of a child joint to a joint
	else if (parentJoint = KIT_DYNAMIC_PTR_CAST(Joint, parent)) {
	  hppDout(info, "WILL_INSERT_JOINT: parent joint = "
		  << parent->name() << ", child joint = " << child->name());
	  insertDynamicPart(parentJoint, childJoint);
	}
      }
    }

    // ======================================================================

    void Device::
    componentDidInsertChild(const CkitNotificationConstShPtr&)
    {
    }

    // ======================================================================

    void Device::insertDynamicPart(JointShPtr parent, JointShPtr child)
    {
      hppDout(info, "");
      if (parent->jrlJoint()) {
	parent->jrlJoint()->addChildJoint(*(child->jrlJoint()));
      }
    }


  } // namespace model
} // namespace hpp

std::ostream& operator<<(std::ostream& os, hpp::model::Device& device)
{
  os << "Device: " << device.name() << std::endl;
  os << std::endl;
  os << "  Current configuration: " << device.currentConfiguration()
     << std::endl;
  os << "  Steering method component: "
     << device.steeringMethodComponent ()->name () << std::endl;
  os << std::endl;
  os << std::endl;
  os << "  Writing kinematic chain" << std::endl;

  //
  // Go through joints and output each joint
  //
  hpp::model::JointShPtr hppJoint = device.getRootJoint();

  if (hppJoint) {
    os << *hppJoint << std::endl;
  }
  // Get position of center of mass
  MAL_S3_VECTOR(com, double);
  com = device.positionCenterOfMass();

  //debug
  os <<"total mass "<<device.mass() <<", COM: " << MAL_S3_VECTOR_ACCESS(com, 0)
     <<", "<< MAL_S3_VECTOR_ACCESS(com, 1) <<", "<< MAL_S3_VECTOR_ACCESS(com, 2)
     <<std::endl;
  return os;
}
