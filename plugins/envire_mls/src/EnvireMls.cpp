/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * \file EnvireMls.cpp
 * \author Raul (Raul.Dominguez@dfki.de)
 * \brief Provides
 *
 * Version 0.1
 */


#include "EnvireMls.hpp"
#include <envire_collider_mls/MLSCollision.hpp>


namespace mars {
  namespace plugins {
    namespace envire_mls {

      using namespace mars::utils;
      using namespace mars::interfaces;

      EnvireMls::EnvireMls(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "EnvireMls") {
		
        envire::collision::MLSCollision* mls_collision = envire::collision::MLSCollision::getInstance();
      }
  
      void EnvireMls::init() {
      }

      void EnvireMls::reset() {
      }

      EnvireMls::~EnvireMls() {
      }


      void EnvireMls::update(sReal time_ms) {
      }

      void EnvireMls::addMLS(envire::core::FrameId center, const std::string & mlsPath){

      }


    } // end of namespace envire_mls
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::envire_mls::EnvireMls);
CREATE_LIB(mars::plugins::envire_mls::EnvireMls);
