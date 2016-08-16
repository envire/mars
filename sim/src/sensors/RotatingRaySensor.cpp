/*
 *  Copyright 2011, 2012, 2014, DFKI GmbH Robotics Innovation Center
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

/*
 *  RotatingRaySensor.cpp
 *
 *  Created by Stefan Haase, Kai von Szadkowski, Malte Langosz
 *
 */

#include "RotatingRaySensor.h"
#include <SimNode.h>

#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/LoadCenter.h>
#include <mars/interfaces/sim/NodeInterface.h>

#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <mars/utils/MutexLocker.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace mars {
  namespace sim {

    using namespace utils;
    using namespace configmaps;
    using namespace interfaces;

    BaseSensor* RotatingRaySensor::instanciate(ControlCenter *control, BaseConfig *config ){
      RotatingRayConfig *cfg = dynamic_cast<RotatingRayConfig*>(config);
      assert(cfg);
      return new RotatingRaySensor(control,*cfg);
    }

    RotatingRaySensor::RotatingRaySensor(ControlCenter *control, RotatingRayConfig config):
    BasePolarIntersectionSensor(config.id, config.name,
                                config.bands*config.lasers,
                                1, 
                                config.opening_width,
                                config.opening_height),
    SensorInterface(control),
    config(config){
      setFixedParameters();
      validateConfigVals();
      setConfigBasedParameters();
      dataBrokerSetup();
      setSensorPos();
      computeRaysDirectionsAndPrepareDraw();
      closeThread = false;
      this->start();
    }

    // Auxiliar methods for the constructor
    void RotatingRaySensor::setFixedParameters(){
      orientation.setIdentity();
      full_scan = false;
      current_pose.setIdentity();
      num_points = 0;
      toCloud = &pointcloud1;
      toDepthMap = &partialDepthMaps1;
      convertPointCloud = false;
      nextCloud = 2;
      update_available = false;
      for(int i = 0; i < 3; ++i)
        positionIndices[i] = -1;
      for(int i = 0; i < 4; ++i)
        rotationIndices[i] = -1;
    }

    void RotatingRaySensor::setConfigBasedParameters(){
      frame = config.frame;
      updateRate = config.updateRate;
      maxDistance = config.maxDistance;
      this->attached_node = config.attached_node;
      // All bands will be turned from 'turning_offset' to 'turning_end_fullscan' in 'turning_step steps'.
      turning_offset = 0;
      turning_end_fullscan = config.opening_width / config.bands;
      turning_step = config.horizontal_resolution; 
      vertical_resolution = config.lasers <= 1 ? 0 : config.opening_height/(config.lasers -1);
      // Initialize DepthMap 
      partialDepthMaps1 = std::vector<base::samples::DepthMap>(config.bands);
      partialDepthMaps2 = std::vector<base::samples::DepthMap>(config.bands);
      finalDepthMap = base::samples::DepthMap();
      double limitVAngle = config.lasers <= 1 ? 0.0 : config.opening_height/2.0;
      std::vector<double> vertical_interval;
      vertical_interval.push_back(-limitVAngle + config.vertical_offset);
      vertical_interval.push_back(limitVAngle + config.vertical_offset);
      finalDepthMap.vertical_interval = vertical_interval;
      std::vector<double> horizontal_interval;
      horizontal_interval.push_back(-M_PI);
      horizontal_interval.push_back(M_PI);
      finalDepthMap.horizontal_interval = horizontal_interval;
      finalDepthMap.vertical_size = config.lasers;
      finalDepthMap.horizontal_size = (2.0*M_PI)/config.horizontal_resolution;
      LOG_DEBUG("Horizontal size from config is: %d", finalDepthMap.horizontal_size );
      LOG_DEBUG("Horizontal by vertical size: %d", finalDepthMap.horizontal_size*finalDepthMap.vertical_size );
    }

   void RotatingRaySensor::setSensorPos(){
     envire::core::Transform sensorTf = control->graph->getTransform("center", frame); //FIXME Standarize the center of the graph name
     position = sensorTf.transform.translation;
     orientation = sensorTf.transform.orientation;
     LOG_DEBUG("[RotatingRaySensor] Position and orientation obtained ");
     orientation_offset.setIdentity();
   }

   void RotatingRaySensor::dataBrokerSetup(){
     std::string groupName, dataName;
     using SimNodeItem = envire::core::Item<std::shared_ptr<SimNode>>;
     using Iterator = envire::core::EnvireGraph::ItemIterator<SimNodeItem>;
     std::shared_ptr<SimNode> simNodePtr;
     Iterator begin, end;
     boost::tie(begin, end) = control->graph->getItems<SimNodeItem>(frame);
     if (begin != end){
       simNodePtr = begin->getData(); 
       simNodePtr->getDataBrokerNames(&groupName, &dataName);
     }
     else
     {
       LOG_DEBUG("[RotatingRaySensor] The SimNode for the data broker to link to the sensor is NOT found");
     }
     if(control->dataBroker->registerTimedReceiver(this, groupName, dataName,"mars_sim/simTimer",updateRate)) {
     }
   }

   void RotatingRaySensor::validateConfigVals(){
        // Fills the direction array.
        if(config.bands < 1) {
          std::cerr << "Number of bands too low("<< config.bands <<"),will be set to 1" << std::endl;
          config.bands = 1;
        }
        if(config.lasers < 1) {
          std::cerr << "Number of lasers too low("<< config.lasers <<"),will be set to 1" << std::endl;
          config.lasers = 1;
        }
   }

   void RotatingRaySensor::computeRaysDirectionsAndPrepareDraw(){
     double vAngle = config.lasers <= 1 ? config.opening_height/2.0 : config.opening_height/(config.lasers-1);
     double hAngle = config.bands <= 1 ? 0 : config.opening_width/config.bands;
     double h_angle_cur = 0.0;
     double v_angle_cur = 0.0;
     utils::Vector tmp;
     drawStruct draw;
     for(int b=0; b<config.bands; ++b) {
       h_angle_cur = b*hAngle - config.opening_width / 2.0 + config.horizontal_offset;
       for(int l=0; l<config.lasers; ++l) {
         v_angle_cur = l*vAngle - config.opening_height / 2.0 + config.vertical_offset;
         tmp = Eigen::AngleAxisd(h_angle_cur, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(v_angle_cur, Eigen::Vector3d::UnitY()) *
           Vector(1,0,0);
         directions.push_back(tmp);
         // Add a drawing item for each ray regarding the initial sensor orientation.
         if(config.draw_rays) {
           pushDrawRayItem(tmp, draw);
         }
       }
     }
     // GraphicsManager crashes if default constructor drawStruct is passed.
     if(config.draw_rays) {
       if(control->graphics) {
         control->graphics->addDrawItems(&draw);
       }
     }
   }
   
   void RotatingRaySensor::pushDrawRayItem(const Vector &tmp, drawStruct &draw){
     draw_item item;
     draw.ptr_draw = (DrawInterface*)this;
     item.id = 0;
     item.type = DRAW_LINE;
     item.draw_state = DRAW_STATE_CREATE;
     item.point_size = 1;
     item.myColor.r = 1;
     item.myColor.g = 0;
     item.myColor.b = 0;
     item.myColor.a = 1;
     item.texture = "";
     item.t_width = item.t_height = 0;
     item.get_light = 0.0;
     // Initial vector length is set to 1.0
     item.start = position;
     item.end = orientation * tmp;
     draw.drawItems.push_back(item);
   }

    RotatingRaySensor::~RotatingRaySensor(void) {
      control->graphics->removeDrawItems((DrawInterface*)this);
      control->dataBroker->unregisterTimedReceiver(this, "*", "*", "mars_sim/simTimer");
      closeThread = true;
      this->wait();
    }

    // This is the method that is used in the orogen task.
    // We may implement a similar one, or extend it to return also a DepthMap
    bool RotatingRaySensor::getPointcloud(std::vector<utils::Vector>& pcloud) {
      mars::utils::MutexLocker lock(&mutex_pointcloud);
      if(full_scan) {
        full_scan = false;
        pcloud.swap(pointcloud_full);
        return true;
      } else {
          return false;
      }
    }


    bool RotatingRaySensor::getDepthMap(base::samples::DepthMap &depthMap) {
      mars::utils::MutexLocker lock(&mutex_pointcloud);
      if(full_scan) {
        full_scan = false;
        bool valid = true;
        unsigned int i = 0;
        int count_invalids = 0;
        depthMap.time = finalDepthMap.time;
        depthMap.vertical_projection = finalDepthMap.vertical_projection;
        depthMap.horizontal_projection = finalDepthMap.horizontal_projection;
        depthMap.horizontal_interval= finalDepthMap.horizontal_interval;
        depthMap.vertical_interval = finalDepthMap.vertical_interval;
        depthMap.vertical_size = finalDepthMap.vertical_size;
        depthMap.horizontal_size = finalDepthMap.horizontal_size;
        depthMap.timestamps.swap(finalDepthMap.timestamps);
        depthMap.distances.swap(finalDepthMap.distances);
        depthMap.remissions.swap(finalDepthMap.remissions);
        finalDepthMap.distances.clear();
        finalDepthMap.timestamps.clear();
        finalDepthMap.remissions.clear();
        return true;
      } else {
          return false;
      }
    }
    
    // Copies into data the full pointcloud. 
    // I guess here what has to be copied now is the DepthMap
    // TODO Ask Malte what this method is for
    // Seems not to be executed in our use case. The trace is never shown
    int RotatingRaySensor::getSensorData(double** data_) const {
      LOG_DEBUG("Executing getSensorData");       
      mars::utils::MutexLocker lock(&mutex_pointcloud);
      *data_ = (double*)malloc(pointcloud_full.size()*3*sizeof(double));
      return pointcloud_full.size();
    }
      
    void RotatingRaySensor::getCurrentPose(const data_broker::DataPackage &package){
      if(positionIndices[0] == -1) {
        positionIndices[0] = package.getIndexByName("position/x");
        positionIndices[1] = package.getIndexByName("position/y");
        positionIndices[2] = package.getIndexByName("position/z");
        rotationIndices[0] = package.getIndexByName("rotation/x");
        rotationIndices[1] = package.getIndexByName("rotation/y");
        rotationIndices[2] = package.getIndexByName("rotation/z");
        rotationIndices[3] = package.getIndexByName("rotation/w");
      }
      package.get(positionIndices[0], &position.x());
      package.get(positionIndices[1], &position.y());
      package.get(positionIndices[2], &position.z());
      package.get(rotationIndices[0], &orientation.x());
      package.get(rotationIndices[1], &orientation.y());
      package.get(rotationIndices[2], &orientation.z());
      package.get(rotationIndices[3], &orientation.w());
      poseMutex.lock();
      current_pose.setIdentity();
      current_pose.rotate(orientation);
      current_pose.translation() = position;
      poseMutex.unlock();
    }

    // Receives the measured distances in package. Method is called by the
    // DataBroker as soon as the registered event occurs.
    void RotatingRaySensor::receiveData(const data_broker::DataInfo &info,
                                const data_broker::DataPackage &package,
                                int callbackParam) {
      CPP_UNUSED(info);
      CPP_UNUSED(callbackParam);
      std::vector<base::samples::DepthMap> &partialDepthMaps = *toDepthMap; 
      long id;
      package.get(0, &id);
      getCurrentPose(package);
      // data[] contains all the measured distances according to the define directions.
      assert((int)data.size() == config.bands * config.lasers);
      int i = 0; // data_counter
      utils::Vector local_ray, tmpvec;
      double local_dist;
      base::Time timestamp = base::Time::Time::now(); 
      for(int b=0; b<config.bands; ++b) {
        partialDepthMaps[b].timestamps.push_back(timestamp);
        for(int col=0; col<config.lasers; col++, ++i){
          local_ray = orientation_offset * directions[i] * data[i];
          partialDepthMaps[b].distances.push_back(local_ray.norm());
        }
      }
      num_points += data.size();
      update_available = true;
    }

    void RotatingRaySensor::update(std::vector<draw_item>* drawItems) {

      unsigned int i;

      if(update_available) {
        control->nodes->updateRay(attached_node);
        update_available = false;
      }
      if(config.draw_rays) {
        if(!(*drawItems)[0].draw_state) {
          for(i=0; i<data.size(); i++) {
              (*drawItems)[i].draw_state = DRAW_STATE_UPDATE;
              // Updates the rays using the current sensor pose. 
              (*drawItems)[i].start = position;
              (*drawItems)[i].end = (orientation * orientation_offset * directions[i]);
              (*drawItems)[i].end *= data[i];
              (*drawItems)[i].end += (*drawItems)[i].start;
          }
        }
      }
    }

    utils::Quaternion RotatingRaySensor::turn() {  
      
      // If the scan is full the pointcloud will be copied.
      mutex_pointcloud.lock();
      turning_offset += turning_step;
      if(turning_offset >= turning_end_fullscan) {
        while(convertPointCloud) msleep(1);
        fromCloud = toCloud;
        fromDepthMap = toDepthMap;
        if(nextCloud == 2) {
          toCloud = &pointcloud2;
          toDepthMap = &partialDepthMaps2;
          nextCloud = 1;
        }
        else {
          toCloud = &pointcloud1;
          toDepthMap = &partialDepthMaps1;
          nextCloud = 2;
        }
        convertPointCloud = true;
        turning_offset = 0;
      }
      orientation_offset = utils::angleAxisToQuaternion(turning_offset, utils::Vector(0.0, 0.0, 1.0));
      mutex_pointcloud.unlock();
      
      return orientation_offset;
    }

    int RotatingRaySensor::getNumberRays() {
      return config.bands * config.lasers;
    }

    void RotatingRaySensor::prepareFinalPointcloud(){
      // Copies current full pointcloud to pointcloud_full.
      poseMutex.lock();
      Eigen::Affine3d current_pose2 = current_pose;
      Eigen::Affine3d rot;
      rot.setIdentity();
      rot.rotate(config.transf_sensor_rot_to_sensor);
      poseMutex.unlock();
      std::list<utils::Vector>::iterator it = fromCloud->begin();
      pointcloud_full.clear();
      pointcloud_full.reserve(fromCloud->size());
      base::Vector3d vec_local;
      for(int i=0; it != fromCloud->end(); it++, i++) {
        // Transforms the pointcloud back from world to current node (see receiveDate()).
        // In addition 'transf_sensor_rot_to_sensor' is applied which describes
        // the orientation of the sensor in the unturned sensor frame.
        vec_local = rot * current_pose2.inverse() * (*it);
        pointcloud_full.push_back(vec_local);
      }
      fromCloud->clear();
    }

    void RotatingRaySensor::prepareFinalDepthMap(){
      std::vector<base::samples::DepthMap> &partialDepthMaps = *fromDepthMap; 

      for(int b=0; b<config.bands; b++){
        //LOG_DEBUG("Num samples in partial depthmap %d: %d", b, partialDepthMaps[b].timestamps.size());
        for(unsigned int sample=0; sample < partialDepthMaps[b].timestamps.size(); sample++){
          finalDepthMap.timestamps.push_back(partialDepthMaps[b].timestamps[sample]);
        }
      }
      int partial_row_size = config.lasers;
      for (int laser_i=0; laser_i<config.lasers; laser_i++){
        for(int b=0; b<config.bands; b++){
          int partial_num_samples = partialDepthMaps[b].timestamps.size();
          //LOG_DEBUG("Partial DepthMap: %d ", b);
          //LOG_DEBUG("Partial num samples: %d", partial_num_samples);
          //LOG_DEBUG("Distances size: %d", partialDepthMaps[b].distances.size());
          //LOG_DEBUG("partialDepthMaps1[b] timestamps size: %d", partialDepthMaps1[b].timestamps.size() );
          //LOG_DEBUG("partialDepthMaps2[b] timestamps size: %d", partialDepthMaps2[b].timestamps.size() );
          for (int sample_i=0; sample_i<partial_num_samples; sample_i++){
            // We have to invert the coordinates
            finalDepthMap.distances.push_back(partialDepthMaps[b].distances[sample_i*partial_row_size + laser_i]);
            finalDepthMap.remissions.push_back(1.0);
          }
        }
      }
      for(int b=0; b<config.bands; b++){
        partialDepthMaps[b].distances.clear();
        partialDepthMaps[b].timestamps.clear();
      }
      //LOG_DEBUG("The final depthMap has a distances size of: %d", finalDepthMap.distances.size());
      //LOG_DEBUG("The final depthMap has %d timestamps", finalDepthMap.timestamps.size());
      //LOG_DEBUG("The final depthMap has %d vertical size", finalDepthMap.vertical_size);
      //LOG_DEBUG("The final depthMap timestamps multiplied by the number laser should be equal to the number of distances: %d", finalDepthMap.timestamps.size()*finalDepthMap.vertical_size);
      finalDepthMap.time = base::Time::Time::now();
      finalDepthMap.horizontal_size = finalDepthMap.timestamps.size();
    }

    void RotatingRaySensor::run() {
      while(!closeThread) {
        if(convertPointCloud) { 
          prepareFinalPointcloud();
          prepareFinalDepthMap();
          convertPointCloud = false;
          full_scan = true;
        }
        else msleep(2);
      }
    }

    BaseConfig* RotatingRaySensor::parseConfig(ControlCenter *control,
                                       ConfigMap *config) {
      RotatingRayConfig *cfg = new RotatingRayConfig;
      unsigned int mapIndex = (*config)["mapIndex"];
      unsigned long attachedNodeID = (*config)["attached_node"];
      if(mapIndex) {
        attachedNodeID = control->loadCenter->getMappedID(attachedNodeID,
                                                          interfaces::MAP_TYPE_NODE,
                                                          mapIndex);
      }

      ConfigMap::iterator it;
      if((it = config->find("bands")) != config->end())
        cfg->bands = it->second;
      if((it = config->find("lasers")) != config->end())
        cfg->lasers = it->second;
      if((it = config->find("opening_width")) != config->end())
        cfg->opening_width = it->second;
      if((it = config->find("opening_height")) != config->end())
        cfg->opening_height = it->second;
      if((it = config->find("max_distance")) != config->end())
        cfg->maxDistance = it->second;
      if((it = config->find("min_distance")) != config->end())
        cfg->minDistance = it->second;
      if((it = config->find("draw_rays")) != config->end())
        cfg->draw_rays = it->second;
      if((it = config->find("horizontal_offset")) != config->end())
        cfg->horizontal_offset = it->second;
      if((it = config->find("vertical_offset")) != config->end())
        cfg->vertical_offset = it->second;
      if((it = config->find("rate")) != config->end())
        cfg->updateRate = it->second;
      if((it = config->find("horizontal_resolution")) != config->end())
        cfg->horizontal_resolution = it->second;
      cfg->attached_node = attachedNodeID;
      
      ConfigMap::iterator it2;
      if((it = config->find("rotation_offset")) != config->end()) {
        if(it->second.hasKey("yaw")) {
          Vector euler;
          euler.x() = it->second["roll"];
          euler.y() = it->second["pitch"];
          euler.z() = it->second["yaw"];
          cfg->transf_sensor_rot_to_sensor = eulerToQuaternion(euler);
        }
        else {
          Quaternion q;
          q.x() = it->second["x"];
          q.y() = it->second["y"];
          q.z() = it->second["z"];
          q.w() = it->second["w"];
          cfg->transf_sensor_rot_to_sensor = q;
        }
      }

      return cfg;
    }

    ConfigMap RotatingRaySensor::createConfig() const {
      ConfigMap cfg;
      cfg["name"] = config.name;
      cfg["id"] = config.id;
      cfg["type"] = "RaySensor";
      cfg["attached_node"] = config.attached_node;
      cfg["bands"] = config.bands;
      cfg["lasers"] = config.lasers;
      cfg["opening_width"] = config.opening_width;
      cfg["opening_height"] = config.opening_height;
      cfg["max_distance"] = config.maxDistance;
      cfg["min_distance"] = config.minDistance;
      cfg["draw_rays"] = config.draw_rays;
      cfg["vertical_offset"] = config.vertical_offset;
      cfg["horizontal_offset"] = config.horizontal_offset;
      cfg["rate"] = config.updateRate;
      cfg["horizontal_resolution"] = config.horizontal_resolution;
      //cfg["rotation_offset"] = config.transf_sensor_rot_to_sensor;
      /*
        cfg["stepX"] = config.stepX;
        cfg["stepY"] = config.stepY;
        cfg["cols"] = config.cols;
        cfg["rows"] = config.rows;
      */
      return cfg;
    }

    const RotatingRayConfig& RotatingRaySensor::getConfig() const {
      return config;
    }

  } // end of namespace sim
} // end of namespace mars

