#include <boost/test/unit_test.hpp>
#include <physics/ItemPhysics.cpp>
#include <envire_core/Item.hpp>

using namespace mars::sim;

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
/*
