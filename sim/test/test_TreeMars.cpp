/*
 * test_TreeMars.cpp
 *
 *  Created on: Sep 7, 2015
 *      Author: Arne Böckmann
 */
#include <boost/test/unit_test.hpp>
#include "TreeMars.h"
#include <mars/interfaces/sim/ControlCenter.h>
#include <envire_core/GraphViz.hpp>

using namespace envire::core;
using mars::sim::TreeMars;
using mars::interfaces::ControlCenter;
using base::TransformWithCovariance;
using envire::core::GraphViz;
using mars::interfaces::NodeData;


BOOST_AUTO_TEST_CASE(add_objects)
{
    ControlCenter control;
    TreeMars tree(&control);
    NodeData d;
    for(int i = 0; i < 42; ++i)
    {
      TransformWithCovariance tf;
      tf.translation << i, i * 2, i * 3;
      Transform t;
      t.setTransform(tf);
      tree.addObject("test " + boost::lexical_cast<std::string>(i), d, t);
    }
    GraphViz gviz;
    gviz.write(tree, "graphviz_boost_test_tree_mars_add_object.dot");
}



