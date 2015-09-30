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
 * \file test.cpp
 * \author Arne Böckmann
 * \brief Plugin
 */


#include "test.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/interfaces/graphics/GraphicsManagerInterface.h>
#include <envire_core/items/Transform.hpp>
#include <envire_core/graph/TransformGraph.hpp>
#include <envire_core/events/ItemAddedEvent.hpp>
#include <envire_core/events/TransformAddedEvent.hpp>
#include <envire_core/events/TransformModifiedEvent.hpp>
#include <envire_core/graph/TransformGraphExceptions.hpp>
#include <mars/sim/ConfigMapItem.h>
#include <base/TransformWithCovariance.hpp>
#include <stdlib.h>
#include <iostream>
#include <algorithm>

using namespace mars::plugins::test_graphics;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;
using namespace mars::sim;
using namespace std;

Test::Test(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "Test"), originId(""){
}

void Test::init() {
  
  if(control->graph != nullptr)
  {
    control->graph->subscribe(std::shared_ptr<GraphEventDispatcher>(this));
  }
}

void Test::reset() {
}

Test::~Test() {
  if(control->graph != nullptr)
  {
    control->graph->unsubscribe(std::shared_ptr<GraphEventDispatcher>(this));
  }
}

void Test::transformAdded(const envire::core::TransformAddedEvent& e)
{
  if(tree.empty())
  {
    //use the first frame we get as originId
    changeOrigin(e.origin);
  }
  else
  {
    //update the tree structure
    //find the vertex that was added
    const vertex_descriptor target = control->graph->vertex(e.target);
    const vertex_descriptor origin = control->graph->vertex(e.origin);
    if(tree.find(origin) == tree.end() &&
      tree.find(target) != tree.end())
    {
      //origin was added
      tree[target].push_back(origin);
    }
    else if(tree.find(target) == tree.end() &&
            tree.find(origin) != tree.end())
    {
      //target was added
      tree[origin].push_back(target);
    }
    else
    {
      //loop detected. No need to do anything
    }
  }
}

void Test::transformRemoved(const envire::core::TransformRemovedEvent& e)
{
  //TODO figure out what to do when the origin frame gets removed
}

void Test::transformModified(const envire::core::TransformModifiedEvent& e)
{
  const vertex_descriptor target = control->graph->vertex(e.target);
  const vertex_descriptor origin = control->graph->vertex(e.origin);
  vertex_descriptor parent;
  vertex_descriptor child;
  
  if(isParent(target, origin))
  {
    parent = target;
    child = origin;
  }
  else if(isParent(origin, target))
  {
    parent = origin;
    child = target;
  }
  else
  {
    //this cannot happen, either target should be the parent of origin or the
    //other way around.
    assert(false);
  }
  
  //update all children
  vector<vertex_descriptor> queue;
  queue.push_back(child);
  while(!queue.empty())
  {
    const vertex_descriptor vertex = queue.back();
    queue.pop_back();
    
    updatePosition(vertex);
    
    //if the vertex has children, queue them
    if(tree.find(vertex) != tree.end())
    {
      for(const vertex_descriptor vd : tree[vertex])
      {
        queue.emplace_back(vd);
      }
    }
  }
}

void Test::itemAdded(const envire::core::ItemAddedEvent& e)
{
  boost::intrusive_ptr<ConfigMapItem> pItem;
  if(pItem = boost::dynamic_pointer_cast<ConfigMapItem>(e.item))
  {
    assert(uuidToGraphicsId.find(pItem->getID()) == uuidToGraphicsId.end());
    try
    {         
      NodeData node;
      node.fromConfigMap(&pItem->getData(), "");
      Transform fromOrigin = control->graph->getTransform(originId, e.frame); 
      node.pos = fromOrigin.transform.translation;
      node.rot = fromOrigin.transform.orientation;
      uuidToGraphicsId[pItem->getID()] = control->graphics->addDrawObject(node);
    }
    catch(const UnknownTransformException& ex)
    {
      cerr << ex.what() << endl;
    }
  }
}

bool Test::isParent(vertex_descriptor a, vertex_descriptor b) const
{
  if(tree.find(a) != tree.end())
  {
    //a is a parent, now check if b is the child of a
    return std::find(tree.at(a).begin(), tree.at(a).end(), b) != tree.at(a).end();
  }
  return false;
}

void Test::update(sReal time_ms) {

}

void Test::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
}

void Test::changeOrigin(const FrameId& origin)
{
  originId = origin;
  tree = control->graph->getTree(control->graph->vertex(origin));
}

/**Updates the drawing position of @p vertex */              
void Test::updatePosition(const vertex_descriptor vertex) const
{
  const FrameId& frameId = control->graph->getFrameId(vertex);
  const vector<ItemBase::Ptr>& items = control->graph->getItems(vertex);
  const Transform tf = control->graph->getTransform(originId, frameId);
  for(const ItemBase::Ptr item : items)
  {
    const int graphicsId = uuidToGraphicsId.at(item->getID());
    control->graphics->setDrawObjectPos(graphicsId, tf.transform.translation);
    control->graphics->setDrawObjectRot(graphicsId, tf.transform.orientation);
  }
}

DESTROY_LIB(mars::plugins::test_graphics::Test);
CREATE_LIB(mars::plugins::test_graphics::Test);
