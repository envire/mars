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
    //load lib with some helpful primitives
    osgviz::PrimitivesFactory *primitivesfactory = osgViz->getVisualizerPlugin< osgviz::PrimitivesFactory >("PrimitivesFactory");
    osg::ref_ptr<osgviz::Object> grid = primitivesfactory->createGrid();
    // Piramid example
    osg::Group* root = new osg::Group();
    osg::Geode* pyramidGeode = new osg::Geode();
    osg::Geometry* pyramidGeometry = new osg::Geometry();
    pyramidGeode->addDrawable(pyramidGeometry); 
    root->addChild(pyramidGeode);
    osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
    pyramidVertices->push_back( osg::Vec3( 0, 0, 0) ); // front left
    pyramidVertices->push_back( osg::Vec3(10, 0, 0) ); // front right
    pyramidVertices->push_back( osg::Vec3(10,10, 0) ); // back right 
    pyramidVertices->push_back( osg::Vec3( 0,10, 0) ); // back left 
    pyramidVertices->push_back( osg::Vec3( 5, 5,10) ); // peak
    pyramidGeometry->setVertexArray( pyramidVertices );
    osg::DrawElementsUInt* pyramidBase = 
        new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    pyramidBase->push_back(3);
    pyramidBase->push_back(2);
    pyramidBase->push_back(1);
    pyramidBase->push_back(0);
    pyramidGeometry->addPrimitiveSet(pyramidBase); 
    osg::DrawElementsUInt* pyramidFaceOne = 
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceOne->push_back(0);
    pyramidFaceOne->push_back(1);
    pyramidFaceOne->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceOne);
    osg::DrawElementsUInt* pyramidFaceTwo = 
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceTwo->push_back(1);
    pyramidFaceTwo->push_back(2);
    pyramidFaceTwo->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceTwo);
    osg::DrawElementsUInt* pyramidFaceThree = 
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceThree->push_back(2);
    pyramidFaceThree->push_back(3);
    pyramidFaceThree->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceThree);
    osg::DrawElementsUInt* pyramidFaceFour = 
        new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    pyramidFaceFour->push_back(3);
    pyramidFaceFour->push_back(0);
    pyramidFaceFour->push_back(4);
    pyramidGeometry->addPrimitiveSet(pyramidFaceFour);
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 0 red
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) ); //index 1 green
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) ); //index 2 blue
    colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) ); //index 3 white 
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) ); //index 4 red
    pyramidGeometry->setColorArray(colors);
    pyramidGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    // Piramid end
    osgViz->addChild(root);
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
