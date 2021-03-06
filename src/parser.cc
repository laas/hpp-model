//
// Copyright (c) 2010, 2011 CNRS
// Authors: Florent Lamiraux
//
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
#include <typeinfo>
#include <stdexcept>

#include <boost/tokenizer.hpp>

#include <kprParserXML/kprParserManager.h>
#include <KineoModuleManager/kppModuleManager.h>
#include <KineoModuleManager/kppModule.h>

#include <hpp/util/debug.hh>
#include "hpp/model/humanoid-robot.hh"
#include "hpp/model/anchor-joint.hh"
#include "hpp/model/freeflyer-joint.hh"
#include "hpp/model/rotation-joint.hh"
#include "hpp/model/translation-joint.hh"
#include "hpp/model/parser.hh"

#include "module-dir.hh"

namespace hpp {
  namespace model {
    Parser::Parser(bool addon)
    {
      if (addon) {
	// Initialize module manager.
	CkppModuleManagerShPtr moduleManager = CkppModuleManager::create ();
	moduleManager->addModuleFile (HPP_MODEL_KINEODEVICEPARSING_SO);
	moduleManager->addModuleFile (HPP_MODEL_KINEODEVICEBASE_SO);
	moduleManager->addModuleFile (HPP_MODEL_KINEODEVICE_SO);

	moduleManager->initializeModules ();

	if (moduleManager->countModules () == 0) {
	  hppDout (warning, "No module loaded. "
		   "Are you sure you LD_LIBRARY_PATH is correctly set?");
	} else {
	  for (unsigned int i=0; i < moduleManager->countModules (); i++) {
	    hppDout (info, "Module " << i << ": "
		     << moduleManager->module (i)->name ());
	  }
	}
      CkprParserManager::defaultManager()->moduleManager (moduleManager);
      }

#ifdef HPP_DEBUG
      CkitLogManager::createInstance("/tmp/kite.log");
#endif

      ktStatus status = KD_ERROR;
      // Write humanoid robot
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeHumanoidRobot);
      // Read humanoid robot
      status =CkprParserManager::defaultManager()->addXMLTagBuilderMethod
	<Parser>("HPP_HUMANOID_ROBOT", this, &Parser::buildHumanoidRobot);
      if (status != KD_OK)
	throw std::runtime_error
	  ("Could not register HPP_HUMANOID_ROBOT tag");
      hppDout(info, "register HPP_HUMANOID_ROBOT tag");
      // Write freeflyer joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeFreeflyerJoint);
      // Read freeflyer joint
      status =CkprParserManager::defaultManager()->addXMLTagBuilderMethod
	<Parser>("HPP_FREEFLYER_JOINT", this, &Parser::buildFreeflyerJoint);
      if (status != KD_OK)
	throw std::runtime_error
	  ("Could not register HPP_FREEFLYER_JOINT tag");
      hppDout(info, "register HPP_FREEFLYER_JOINT tag");
      // Write rotation joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeRotationJoint);
      // Read rotation joint
      status =
	CkprParserManager::defaultManager()->addXMLTagBuilderMethod < Parser >
	("HPP_ROTATION_JOINT", this, &Parser::buildRotationJoint);
      if (status != KD_OK)
	throw std::runtime_error
	  ("Could not register HPP_ROTATION_JOINT tag");
      hppDout(info, "register HPP_ROTATION_JOINT tag");
      // Write translation joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeTranslationJoint);
      // Read translation joint
      status =
	CkprParserManager::defaultManager()->addXMLTagBuilderMethod < Parser >
	("HPP_TRANSLATION_JOINT", this, &Parser::buildTranslationJoint);
      if (status != KD_OK)
	throw std::runtime_error
	  ("Could not register HPP_TRANSLATION_JOINT tag");
      hppDout(info, "register HPP_TRANSLATION_JOINT tag");
      // Write anchor joint
      CkprParserManager::defaultManager()->addXMLWriterMethod < Parser >
	(this, &Parser::writeAnchorJoint);
      // Read anchor joint
      status =
	CkprParserManager::defaultManager()->addXMLTagBuilderMethod < Parser >
	("HPP_ANCHOR_JOINT", this, &Parser::buildAnchorJoint);
      if (status != KD_OK)
	throw std::runtime_error
	  ("Could not register HPP_ANCHOR_JOINT tag");
      hppDout(info, "register HPP_ANCHOR_JOINT tag");
    }

    Parser::~Parser()
    {
    }

    ktStatus Parser::writeHumanoidRobot
    (const CkppComponentConstShPtr& inComponent,
     CkprXMLWriterShPtr&,
     CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const HumanoidRobot, inComponent)) {
	inOutTag->name("HPP_HUMANOID_ROBOT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::buildHumanoidRobot
    (const CkprXMLTagConstShPtr&,
     const CkppComponentShPtr&,
     std::vector< CkppComponentShPtr >&,
     CkprXMLBuildingContextShPtr&,
     CkppComponentShPtr& outComponent)
    {
      hppDout(info, "building HumanoidRobot.");
      outComponent = HumanoidRobot::create("Humanoid Robot");
      if (!outComponent) {
	hppDout(error, "failed to create HumanoidRobot");
      }
      return KD_OK;
    }

    ktStatus Parser::
    writeFreeflyerJoint(const CkppComponentConstShPtr& inComponent,
			CkprXMLWriterShPtr&,
			CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const FreeflyerJoint, inComponent)) {
	inOutTag->name("HPP_FREEFLYER_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildFreeflyerJoint(const CkprXMLTagConstShPtr&,
			const CkppComponentShPtr&,
			std::vector< CkppComponentShPtr >&,
			CkprXMLBuildingContextShPtr&,
			CkppComponentShPtr& outComponent)
    {
      hppDout(info, "building FreeFlyerJoint.");
      outComponent = FreeflyerJoint::create("FREEFLYER");
      if (!outComponent) {
	hppDout(error, "failed to create FreeFlyerJoint");
      }
      return KD_OK;
    }

    ktStatus Parser::
    writeRotationJoint(const CkppComponentConstShPtr& inComponent,
		       CkprXMLWriterShPtr&,
		       CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const RotationJoint, inComponent)) {
	inOutTag->name("HPP_ROTATION_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildRotationJoint(const CkprXMLTagConstShPtr&,
		       const CkppComponentShPtr&,
		       std::vector< CkppComponentShPtr >&,
		       CkprXMLBuildingContextShPtr&,
		       CkppComponentShPtr& outComponent)
    {
      hppDout(info, "building RotationJoint.");
      outComponent = RotationJoint::create("ROTATION");
      if (!outComponent) {
	hppDout(error, "failed to create RotationJoint");
      }
      return KD_OK;
    }

    ktStatus Parser::
    writeTranslationJoint(const CkppComponentConstShPtr& inComponent,
			  CkprXMLWriterShPtr&,
			  CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const TranslationJoint, inComponent)) {
	inOutTag->name("HPP_TRANSLATION_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildTranslationJoint(const CkprXMLTagConstShPtr&,
			  const CkppComponentShPtr&,
			  std::vector< CkppComponentShPtr >&,
			  CkprXMLBuildingContextShPtr&,
			  CkppComponentShPtr& outComponent)
    {
      hppDout(info, "building TranslationJoint.");
      outComponent = TranslationJoint::create("TRANSLATION");
      if (!outComponent) {
	hppDout(error, "failed to create TranslationJoint");
      }
      return KD_OK;
    }

    ktStatus Parser::
    writeAnchorJoint(const CkppComponentConstShPtr& inComponent,
		     CkprXMLWriterShPtr&,
		     CkprXMLTagShPtr& inOutTag)
    {
      if (KIT_DYNAMIC_PTR_CAST(const AnchorJoint, inComponent)) {
	inOutTag->name("HPP_ANCHOR_JOINT");
	return KD_OK;
      }
      return KD_ERROR;
    }

    ktStatus Parser::
    buildAnchorJoint(const CkprXMLTagConstShPtr&,
		     const CkppComponentShPtr&,
		     std::vector< CkppComponentShPtr >&,
		     CkprXMLBuildingContextShPtr&,
		     CkppComponentShPtr& outComponent)
    {
      hppDout(info, "building AnchorJoint.");
      outComponent = AnchorJoint::create("ANCHOR");
      if (!outComponent) {
	hppDout(error, "failed to create AnchorJoint");
      }
      return KD_OK;
    }

  } // namespace model
} // namespace hpp
