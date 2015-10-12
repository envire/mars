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

#include "graphviz.h"
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

using namespace mars::plugins::graph_viz_plugin;
using namespace mars::utils;
using namespace mars::interfaces;
using namespace envire::core;
using namespace mars::sim;
using namespace std;

GraphViz::GraphViz(lib_manager::LibManager *theManager)
  : MarsPluginTemplate(theManager, "GraphViz"), originId(""){
}

void GraphViz::init() {
  
  if(control->graph != nullptr)
  {
    control->graph->subscribe(std::shared_ptr<GraphEventDispatcher>(this));
  }
}

void GraphViz::reset() {
}

GraphViz::~GraphViz() {
  if(control->graph != nullptr)
  {
    control->graph->unsubscribe(std::shared_ptr<GraphEventDispatcher>(this));
  }
}

void GraphViz::transformAdded(const envire::core::TransformAddedEvent& e)
{
  const vertex_descriptor target = control->graph->vertex(e.target);
  const vertex_descriptor origin = control->graph->vertex(e.origin);
  if(tree.empty())
  {
    //use the first frame we get as originId
    changeOrigin(e.origin);
    tree[origin].insert(target);
  }
  else
  {
    //update the tree structure
    //find the vertex that was added
    if(tree.find(origin) != tree.end() &&
       tree.find(target) == tree.end())
    {
      //origin already is a parent in the tree and target is not
      tree[origin].insert(target);
    }
    else if(tree.find(origin) == tree.end() &&
            tree.find(target) != tree.end())
    {
      //target already is a parent in the tree and origin is not
      tree[target].insert(origin);
    }
    else
    {
      //loop detected. a new shorter path might be available. refresh the whole tree
      updateTree(originId);
    }
  }
}

void GraphViz::transformRemoved(const envire::core::TransformRemovedEvent& e)
{
  //Removing a transform can lead to non trivial changes in the tree.
  //Instead of thinking about them we just recalculate the tree.
  //This is fast enough for now.
  updateTree(originId);
}

void GraphViz::transformModified(const envire::core::TransformModifiedEvent& e)
{
  const vertex_descriptor target = control->graph->vertex(e.target);
  const vertex_descriptor origin = control->graph->vertex(e.origin);
  vertex_descriptor child;
  
  if(isParent(target, origin))
  {
    child = origin;
  }
  else if(isParent(origin, target))
  {
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

void GraphViz::itemAdded(const envire::core::ItemAddedEvent& e)
{
  boost::shared_ptr<ConfigMapItem> pItem;
  if(pItem = boost::dynamic_pointer_cast<ConfigMapItem>(e.item))
  {
    //assert that this item has not been added before
    assert(uuidToGraphicsId.find(pItem->getID()) == uuidToGraphicsId.end());
    try
    {         
      NodeData node;
      if(node.fromConfigMap(&pItem->getData(), ""))
      {
        Transform fromOrigin = control->graph->getTransform(originId, e.frame); 
        node.pos = fromOrigin.transform.translation;
        node.rot = fromOrigin.transform.orientation;
        uuidToGraphicsId[pItem->getID()] = control->graphics->addDrawObject(node);
      }
    }
    catch(const UnknownTransformException& ex)
    {
      cerr << ex.what() << endl;
    }
  }
}

bool GraphViz::isParent(vertex_descriptor a, vertex_descriptor b) const
{
  if(tree.find(a) != tree.end())
  {
    //a is a parent, now check if b is the child of a
    return std::find(tree.at(a).begin(), tree.at(a).end(), b) != tree.at(a).end();
  }
  return false;
}

void GraphViz::update(sReal time_ms) {
  //uncomment this to randomly change the origin
  /*
  static sReal secondsPassed = 0;
  secondsPassed += time_ms / 1000;
  if(secondsPassed > 500)
  {
    VertexMap::iterator it = tree.begin();
    int randomIndex = rand() % tree.size();
    std::advance(it, randomIndex);
    changeOrigin(control->graph->getFrameId(it->first));
    secondsPassed = 0;
  }
*/
}

void GraphViz::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
}

void GraphViz::changeOrigin(const FrameId& origin)
{
  originId = origin;  
  updateTree(origin);
}

void GraphViz::updateTree(const FrameId& origin)
{
  const vertex_descriptor newOrigin = control->graph->vertex(origin);
  tree = control->graph->getTree(newOrigin);
  //update the origins position
  updatePosition(newOrigin);
  //update all childreen
  for(const auto& it : tree)
  {
    for(const auto& vertex : it.second)
    {
      updatePosition(vertex);
    }
  }
}


/**Updates the drawing position of @p vertex */              
void GraphViz::updatePosition(const vertex_descriptor vertex) const
{
  const FrameId& frameId = control->graph->getFrameId(vertex);
  const vector<ItemBase::Ptr>& items = control->graph->getItems(vertex);
  base::Vector3d translation;
  base::Quaterniond orientation;
  if(originId.compare(frameId) == 0)
  {
    translation << 0, 0, 0;
    orientation.setIdentity();
  }
  else
  {
    const Transform tf = control->graph->getTransform(originId, frameId);
    translation = tf.transform.translation;
    orientation = tf.transform.orientation;
  }
 
  for(const ItemBase::Ptr item : items)
  {
    const int graphicsId = uuidToGraphicsId.at(item->getID());
    control->graphics->setDrawObjectPos(graphicsId, translation);
    control->graphics->setDrawObjectRot(graphicsId, orientation);
  }
}

DESTROY_LIB(mars::plugins::graph_viz_plugin::GraphViz);
CREATE_LIB(mars::plugins::graph_viz_plugin::GraphViz);
