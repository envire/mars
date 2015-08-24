#include <boost/test/unit_test.hpp>
#include <physics/ItemPhysics.cpp>
#include <envire_core/Item.hpp>
// Visualization includes
#include <osgViz/OsgViz.hpp>
#include <osgViz/plugins/viz/Primitives/PrimitivesFactory.h>
#include <unistd.h>//sleep

using namespace mars::sim;
using namespace osgviz;

BOOST_AUTO_TEST_CASE(contructors)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::PhysicsItem item(&physics);
    std::cout << "Constructor Test" << std::endl;
}

BOOST_AUTO_TEST_CASE(hello_world)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::PhysicsItem item(&physics);
    item.hello();
    std::cout << "Hello World Test" << std::endl;
}

BOOST_AUTO_TEST_CASE(set_get)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::PhysicsItem item(&physics);
    mars::sim::WorldPhysics physics2(NULL);
    mars::sim::NodePhysics node(&physics2);
    item.setData(node);
    std::cout << "Set data done" << std::endl;
    BOOST_ASSERT(item.getData() == node);
    std::cout << "Get data done" << std::endl;
    mars::sim::NodePhysics otherNode(&physics);
    BOOST_ASSERT(item.getData() != otherNode);
}
BOOST_AUTO_TEST_CASE(first_visualization)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::PhysicsItem item(&physics);
    // Somehow visualize
    osgviz::OsgViz *osgViz = osgviz::OsgViz::getInstance();
    /*
    //load lib with some helpful primitives
    osgviz::PrimitivesFactory *primitivesfactory = osgViz->getVisualizerPlugin< osgviz::PrimitivesFactory >("PrimitivesFactory");
    osg::ref_ptr<osgviz::Object> grid = primitivesfactory->createGrid();
    osgViz->addChild(grid);
    osgViz->createWindow();
    osgViz->startThread();
    //or don't start the thread and update manually using osgViz->update();
    while (true){
        osgViz->lockThread();
        //do updates
        osgViz->unlockThread();
        sleep(1);
    }
    delete osgViz;
    */
}
/*
 * This is the goal test 
BOOST_AUTO_TEST_CASE(first_object)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::PhysicsItem item(&physics);
    mars::sim::NodePhysics node = item.getData();
    ode.compute(node); //Or however is done
    //mars::sim::SurfacePhysicsItem item(&physics);
    //mars::sim::SurfacePhysics surface = item.getData();
    //ode.compute(surface);
}
*/
