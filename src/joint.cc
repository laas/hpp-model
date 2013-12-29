///
/// Copyright (c) 2011 CNRS
/// Authors: Florent Lamiraux
///
///
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#include <iostream>
#include <sstream>
#include <string>

#include <KineoModel/kppDoubleProperty.h>
#include <KineoModel/kppJointComponent.h>
#include <KineoModel/kppSolidComponentRef.h>
#include <KineoWorks2/kwsJoint.h>

#include <hpp/kwsio/mat.hh>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/joint.hh>
#include <hpp/util/debug.hh>

#include "hpp/model/joint.hh"
#include "hpp/model/device.hh"
#include "hpp/model/exception.hh"

namespace hpp {
  namespace model {
    std::map <CjrlJoint*, JointShPtr> Joint::jointMap_;
    // Mass
    const CkppProperty::TPropertyID
    Joint::MASS_ID(CkppProperty::makeID());
    const std::string Joint::MASS_STRING_ID("MASS");
    const CkppProperty::TPropertyID

    // Center of mass
    // X coordinate
    Joint::COM_X_ID(CkppProperty::makeID());
    const std::string Joint::COM_X_STRING_ID("COM_X");
    // Y coordinate
    const CkppProperty::TPropertyID
    Joint::COM_Y_ID(CkppProperty::makeID());
    const std::string Joint::COM_Y_STRING_ID("COM_Y");
    // Z coordinate
    const CkppProperty::TPropertyID
    Joint::COM_Z_ID(CkppProperty::makeID());
    const std::string Joint::COM_Z_STRING_ID("COM_Z");

    // Inertia matrix
    // XX coefficient
    const CkppProperty::TPropertyID
    Joint::INERTIA_MATRIX_XX_ID(CkppProperty::makeID());
    const std::string
    Joint::INERTIA_MATRIX_XX_STRING_ID("INERTIA_MATRIX_XX");
    // YY coefficient
    const CkppProperty::TPropertyID
    Joint::INERTIA_MATRIX_YY_ID(CkppProperty::makeID());
    const std::string
    Joint::INERTIA_MATRIX_YY_STRING_ID("INERTIA_MATRIX_YY");
    // ZZ coefficient
    const CkppProperty::TPropertyID
    Joint::INERTIA_MATRIX_ZZ_ID(CkppProperty::makeID());
    const std::string
    Joint::INERTIA_MATRIX_ZZ_STRING_ID("INERTIA_MATRIX_ZZ");
    // XY coefficient
    const CkppProperty::TPropertyID
    Joint::INERTIA_MATRIX_XY_ID(CkppProperty::makeID());
    const std::string
    Joint::INERTIA_MATRIX_XY_STRING_ID("INERTIA_MATRIX_XY");
    // XZ coefficient
    const CkppProperty::TPropertyID
    Joint::INERTIA_MATRIX_XZ_ID(CkppProperty::makeID());
    const std::string
    Joint::INERTIA_MATRIX_XZ_STRING_ID("INERTIA_MATRIX_XZ");
    // YZ coefficient
    const CkppProperty::TPropertyID
    Joint::INERTIA_MATRIX_YZ_ID(CkppProperty::makeID());
    const std::string
    Joint::INERTIA_MATRIX_YZ_STRING_ID("INERTIA_MATRIX_YZ");

    CkppJointComponentShPtr Joint::kppJoint() const
    {
      return KIT_DYNAMIC_PTR_CAST(CkppJointComponent, weakPtr_.lock());
    }

    CjrlJoint* Joint::jrlJoint()
    {
      return dynamicJoint_;
    }

    const CjrlJoint* Joint::jrlJoint() const
    {
      return dynamicJoint_;
    }

    Joint::Joint(CjrlJoint* joint) : dynamicJoint_(joint)
    {
    }

    Joint::~Joint() 
    {
      hppDout(info, "hpp::model::Joint::~Joint(): this = "
	      << this << std::endl);
    }

    JointShPtr Joint::parentJoint() const
    {
      JointShPtr parent =
	KIT_DYNAMIC_PTR_CAST(Joint, kppJoint()->kwsJoint()->parentJoint());
      if (!parent) {
	std::ostringstream oss;
	oss << "Joint " << kppJoint()->name() << " has no parent.";
	throw Exception(oss.str());
      }
      return parent;
    }

    JointShPtr Joint::childJoint(unsigned int rank) const
    {
      JointShPtr child = KIT_DYNAMIC_PTR_CAST
	(Joint, kppJoint()->kwsJoint()->childJoint(rank));

      if (!child) {
	std::ostringstream oss;
	oss << "Joint " << kppJoint()->name() << " has no child at rank "
	    << rank << ".";
	throw Exception(oss.str());
      }
      return child;
    }

    void Joint::addChildJoint(JointShPtr childJoint)
    {
      if (kppJoint()->addChildJointComponent(childJoint->kppJoint()) != KD_OK) {
	std::ostringstream oss;
	oss << "Failed to add a child joint to CkppFreeFlyerJointComponent "
	    << kppJoint()->name() << ".";
	throw Exception(oss.str());
      }
      if (!jrlJoint()->addChildJoint(*(childJoint->jrlJoint()))) {
	std::ostringstream oss;
	oss << "Failed to add " << childJoint->kppJoint()->name()
	    << " as child joint to dynamicJRLJapan::Joint "
	    << kppJoint()->name() << ".";
	throw Exception(oss.str());
      }
    }

    unsigned int Joint::countChildJoints() const
    {
      return kppJoint()->countChildJointComponents();
    }

    void Joint::isBounded(unsigned int dofRank, bool bounded)
    {
      kppJoint()->kwsJoint()->dof(dofRank)->isBounded(bounded);
    }

    bool Joint::isBounded(unsigned int inDofRank) const
    {
      return kppJoint()->kwsJoint()->dof(inDofRank)->isBounded();
    }


    double Joint::lowerBound(unsigned int dofRank) const
    {
      return kppJoint()->kwsJoint()->dof(dofRank)->vmin();
    }

    double Joint::upperBound(unsigned int dofRank) const
    {
      return kppJoint()->kwsJoint()->dof(dofRank)->vmax();
    }

    void Joint::lowerBound(unsigned int dofRank, double lowerBound)
    {
      // KPP side
      if (kppJoint()->kwsJoint()->dof(dofRank)->vmin(lowerBound) != KD_OK) {
	std::ostringstream oss;
	oss << "Failed to set lower bound of CkwsJoint "
	    << kppJoint()->name() << ": dof rank: " << dofRank << ".";
	throw Exception(oss.str());
      }
      // CjrlJoint side
      jrlJoint()->lowerBound(dofRank, lowerBound);
    }

    void Joint::upperBound(unsigned int dofRank, double upperBound)
    {
      // KPP side
      if (kppJoint()->kwsJoint()->dof(dofRank)->vmax(upperBound) != KD_OK) {
	std::ostringstream oss;
	oss << "Failed to set upper bound of CkwsJoint "
	    << kppJoint()->name() << ": dof rank: " << dofRank << ".";
	throw Exception(oss.str());
      }
      // CjrlJoint side
      jrlJoint()->upperBound(dofRank, upperBound);
    }

    void Joint::bounds(unsigned int dofRank, const double& lowerBound,
		       const double& upperBound)
    {
      // KPP side
      if (kppJoint()->kwsJoint()->dof(dofRank)->
	  bounds(lowerBound, upperBound) != KD_OK) {
	std::ostringstream oss;
	oss << "Failed to set bounds of CkwsJoint "
	    << kppJoint()->name() << ": dof rank: " << dofRank << ".";
	throw Exception(oss.str());
      }
      // CjrlJoint side
      jrlJoint()->lowerBound(dofRank, lowerBound);
      jrlJoint()->upperBound(dofRank, upperBound);
    }

    void Joint::velocityBounds(unsigned int dofRank,
			       const double& lowerVelocityBound,
			       const double& upperVelocityBound)
    {
      // KPP side
      // CjrlJoint side
      jrlJoint()->lowerVelocityBound(dofRank, lowerVelocityBound);
      jrlJoint()->upperVelocityBound(dofRank, upperVelocityBound);
    }

    void Joint::torqueBounds(unsigned int dofRank,
			     const double& lowerTorqueBound,
			     const double& upperTorqueBound)
    {
      // KPP side
      // CjrlJoint side
      jrlJoint()->lowerTorqueBound(dofRank, lowerTorqueBound);
      jrlJoint()->upperTorqueBound(dofRank, upperTorqueBound);
    }

    ktStatus Joint::init(const JointWkPtr& weakPtr)
    {
      weakPtr_ = weakPtr;
      CkppJointComponentShPtr component = kppJoint();
      mass_ = CkppDoubleProperty::create("MASS", component,
					 MASS_ID , MASS_STRING_ID);
      if (!mass_) return KD_ERROR;

      comX_ = CkppDoubleProperty::create("COM_X", component,
					 COM_X_ID, COM_X_STRING_ID);
      if (!comX_) return KD_ERROR;

      comY_ = CkppDoubleProperty::create("COM_Y", component,
					 COM_Y_ID, COM_Y_STRING_ID);
      if (!comY_) return KD_ERROR;

      comZ_ = CkppDoubleProperty::create("COM_Z", component,
					 COM_Z_ID, COM_Z_STRING_ID);
      if (!comZ_) return KD_ERROR;

      inertiaMatrixXX_ =
	CkppDoubleProperty::create("INERTIA_MATRIX_XX", component,
				   INERTIA_MATRIX_XX_ID,
				   INERTIA_MATRIX_XX_STRING_ID);
      if (!inertiaMatrixXX_) return KD_ERROR;

      inertiaMatrixYY_ =
	CkppDoubleProperty::create("INERTIA_MATRIX_YY", component,
				   INERTIA_MATRIX_YY_ID,
				   INERTIA_MATRIX_YY_STRING_ID);
      if (!inertiaMatrixYY_) return KD_ERROR;

      inertiaMatrixZZ_ =
	CkppDoubleProperty::create("INERTIA_MATRIX_ZZ", component,
				   INERTIA_MATRIX_ZZ_ID,
				   INERTIA_MATRIX_ZZ_STRING_ID);
      if (!inertiaMatrixZZ_) return KD_ERROR;

      inertiaMatrixXY_ =
	CkppDoubleProperty::create("INERTIA_MATRIX_XY", component,
				   INERTIA_MATRIX_XY_ID,
				   INERTIA_MATRIX_XY_STRING_ID);
      if (!inertiaMatrixXY_) return KD_ERROR;

      inertiaMatrixXZ_ =
	CkppDoubleProperty::create("INERTIA_MATRIX_XZ", component,
				   INERTIA_MATRIX_XZ_ID,
				   INERTIA_MATRIX_XZ_STRING_ID);
      if (!inertiaMatrixXZ_) return KD_ERROR;

      inertiaMatrixYZ_ =
	CkppDoubleProperty::create("INERTIA_MATRIX_YZ", component,
				   INERTIA_MATRIX_YZ_ID,
				   INERTIA_MATRIX_YZ_STRING_ID);
      if (!inertiaMatrixYZ_) return KD_ERROR;

      if (dynamicJoint_) {
	jointMap_[dynamicJoint_] = weakPtr_.lock();
	hppDout(info, "jointMap_[" << dynamicJoint_ << "] = " <<
		jointMap_[dynamicJoint_]);
      }
      return KD_OK;
    }

    void Joint::
    fillPropertyVector(std::vector<CkppPropertyShPtr>& outPropertyVector) const
    {
      outPropertyVector.push_back(mass_);
      outPropertyVector.push_back(comX_);
      outPropertyVector.push_back(comY_);
      outPropertyVector.push_back(comZ_);
      outPropertyVector.push_back(inertiaMatrixXX_);
      outPropertyVector.push_back(inertiaMatrixYY_);
      outPropertyVector.push_back(inertiaMatrixZZ_);
      outPropertyVector.push_back(inertiaMatrixXY_);
      outPropertyVector.push_back(inertiaMatrixXZ_);
      outPropertyVector.push_back(inertiaMatrixYZ_);
    }

    // ======================================================================

    matrix4d Joint::abstractMatrixFromCkitMat4(const CkitMat4& matrix)
    {
      hppDout(info, matrix);
      MAL_S4x4_MATRIX(abstractMatrix, double);
      for (unsigned int iRow=0; iRow<4; iRow++) {
	for (unsigned int iCol=0; iCol<4; iCol++) {
	  MAL_S4x4_MATRIX_ACCESS_I_J(abstractMatrix, iRow, iCol) =
	    matrix(iRow, iCol);
	}
      }
      hppDout(info, abstractMatrix);
      return abstractMatrix;
    }

    // ======================================================================

    CkitMat4 Joint::CkitMat4MatrixFromAbstract(const matrix4d& matrix)
    {
      CkitMat4 kitMat4;
      for (unsigned int iRow=0; iRow<4; iRow++) {
	for (unsigned int iCol=0; iCol<4; iCol++) {
	  kitMat4(iRow, iCol) = MAL_S4x4_MATRIX_ACCESS_I_J(matrix, iRow, iCol);
	}
      }
      return kitMat4;
    }

    // ======================================================================

    JointShPtr Joint::fromJrlJoint(CjrlJoint* joint)
    {
      return jointMap_[joint];
    }


    // ======================================================================

    void Joint::insertBody()
    {
      JointShPtr joint = weakPtr_.lock();
      const std::string name = KIT_DYNAMIC_PTR_CAST(CkppComponent, joint)
	->name();

      if (jrlJoint() && !jrlJoint()->linkedBody()) {
	CjrlBody* jrlBody = Device::objectFactory_.createBody();
	// Set mass
	jrlBody->mass(mass_->value());
	// Set local center of mass
	vector3d com;
	com[0] = comX_->value();
	com[1] = comY_->value();
	com[2] = comZ_->value();
	jrlBody->localCenterOfMass(com);
	// Set inertia matrix
	matrix3d inertia;
	inertia(0,0) = inertiaMatrixXX_->value();
	inertia(1,1) = inertiaMatrixYY_->value();
	inertia(2,2) = inertiaMatrixZZ_->value();
	inertia(0,1) = inertia(1,0) = inertiaMatrixXY_->value();
	inertia(0,2) = inertia(2,0) = inertiaMatrixXZ_->value();
	inertia(1,2) = inertia(2,1) = inertiaMatrixYZ_->value();
	jrlBody->inertiaMatrix(inertia);
	jrlJoint()->setLinkedBody(*jrlBody);
      }
    }

    // ======================================================================

    void Joint::createDynamicPart()
    {
      if (jointFactory_) {
	CkitMat4 initialPos = kppJoint()->kwsJoint()->initialPosition();
	dynamicJoint_ =
	  jointFactory_(&Device::objectFactory_,
			Joint::abstractMatrixFromCkitMat4(initialPos));
	jointMap_[dynamicJoint_] = weakPtr_.lock();
	hppDout(info, "jointMap_[" << dynamicJoint_ << "] = " <<
		jointMap_[dynamicJoint_]);
	insertBody();
      }
    }
  } // namespace model
} // namespace hpp

std::ostream& operator<<(std::ostream& os, const hpp::model::Joint& joint)
{
  os << "Joint: " << joint.kppJoint()->name() << std::endl;
  if (joint.jrlJoint()->numberDof () != 0)
    os << "Rank in configuration dynamic part: "
       << joint.jrlJoint()->rankInConfiguration()
       << std::endl;
  else
    os << "Anchor joint" << std::endl;
  os << "Current transformation dynamic part:" << std::endl;
  os << joint.jrlJoint()->currentTransformation() << std:: endl;
  os << std::endl;
  os << "Current transformation geometric part:" << std::endl;
  os << joint.kppJoint()->kwsJoint()->currentPosition() << std:: endl;

  const CjrlJoint* jrlJoint = joint.jrlJoint();
  if (jrlJoint) {
    CjrlBody* hppBody = jrlJoint->linkedBody();
    if (hppBody) {
      os << "Attached body:" << std::endl;
      os << "Mass of the attached body: " << hppBody->mass() << std::endl;
      os << "Local center of mass:" << hppBody->localCenterOfMass()
	 << std::endl;
      os << "Inertia matrix:" << std::endl;
      os << hppBody->inertiaMatrix() << std::endl;
    } else {
      os << "No attached body" << std::endl;
    }
  } else {
    os << "No dynamic part" << std::endl;
  }
  std::vector<CkppSolidComponentRefShPtr> solidComps;
  joint.kppJoint ()->getSolidComponentRefVector (solidComps);
  CkitMat4 position;
  os << "solid components" << std::endl;
  for (std::vector<CkppSolidComponentRefShPtr>::iterator it =
	 solidComps.begin(); it != solidComps.end (); it++) {
    CkppSolidComponentShPtr solidComp ((*it)->referencedSolidComponent ());
    solidComp->getAbsolutePosition (position);
    os << "name: " << solidComp->name () << std::endl;
    os << "position :" << std::endl;
    os << position << std::endl;
  }
  for (unsigned int iChild=0; iChild < joint.countChildJoints();
       iChild++) {
    os << *(joint.childJoint(iChild)) << std::endl;
    os <<std::endl;
  }

  return os;
}
