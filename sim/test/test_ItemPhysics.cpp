#include <boost/test/unit_test.hpp>
#include <physics/ItemPhysics.cpp>
#include <envire_core/Item.hpp>

using namespace mars::sim;

BOOST_AUTO_TEST_CASE(contructors)
{
 
    mars::sim::PhysicsItem item;
    //mars::sim::MarsItem<mars::sim::NodePhysics> item;
    std::cout << "Constructor Test" << std::endl;
}

BOOST_AUTO_TEST_CASE(hello_world)
{
    mars::sim::MarsItem<mars::sim::NodePhysics> item;
    item.hello();
}

BOOST_AUTO_TEST_CASE(set_get)
{
    mars::sim::MarsItem<mars::sim::NodePhysics> item;
    mars::sim::NodePhysics node(NULL);
    item.setData(node);
    std::cout << "Set data done" << std::endl;
    mars::sim::NodePhysics node_copy = item.getData();
    std::cout << "Get data done" << std::endl;
}
