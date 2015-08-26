#include <boost/test/unit_test.hpp>
#include <physics/ItemPhysics.cpp>
#include <envire_core/Item.hpp>
// Visualization includes
#include <osgViz/OsgViz.hpp>
#include <osgViz/plugins/viz/Primitives/PrimitivesFactory.h>
#include <unistd.h>//sleep

using namespace mars::sim;
using namespace osgviz;

BOOST_AUTO_TEST_CASE(first_visualization)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::PhysicsItem item(&physics);
    // Somehow visualize
    osgviz::OsgViz *osgViz = osgviz::OsgViz::getInstance();
    //load lib with some helpful primitives
    osgviz::PrimitivesFactory *primitivesfactory = osgViz->getVisualizerPlugin< osgviz::PrimitivesFactory >("PrimitivesFactory");
    osg::ref_ptr<osgviz::Object> grid = primitivesfactory->createGrid();
    osgViz->addChild(grid);
    /*
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
    osgViz->addChild(root);
    // Piramid end
    */
    /*
    // Box shape example
    //Declare a group to act as root node of a scene:
    osg::Group* root = new osg::Group();
   
    // Declare a box class (derived from shape class) instance
    // This constructor takes an osg::Vec3 to define the center
    // and a float to define the height, width and depth.
    // (an overloaded constructor allows you to specify unique
    // height, width and height values.)
    osg::Box* unitCube = new osg::Box( osg::Vec3(0,0,0), 1.0f);
   
    // Declare an instance of the shape drawable class and initialize 
    // it with the unitCube shape we created above.
    // This class is derived from 'drawable' so instances of this
    // class can be added to Geode instances.
    osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube); // I have to include osg/ShapeDrawable
   
    // Declare a instance of the geode class: 
    osg::Geode* basicShapesGeode = new osg::Geode();
   
    // Add the unit cube drawable to the geode:
    basicShapesGeode->addDrawable(unitCubeDrawable);
   
    // Add the goede to the scene:
    root->addChild(basicShapesGeode);
    osgViz->addChild(root);
    */
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
