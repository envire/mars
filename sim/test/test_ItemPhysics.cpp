#include <boost/test/unit_test.hpp>
#include <physics/ItemPhysics.cpp>
#include <envire_core/Item.hpp>

using namespace mars::sim;

BOOST_AUTO_TEST_CASE(contructors)
{
    mars::sim::ItemPhysics item;
    std::cout << "Constructor Test";
    
}

BOOST_AUTO_TEST_CASE(hello_world)
{
    mars::sim::ItemPhysics item;
    item.hello();
}

BOOST_AUTO_TEST_CASE(dummy_welcome)
{
    mars::sim::ItemPhysics item;
    mars::sim::DummyClass dummy;
    dummy.welcome();
    item.setData(dummy);
    mars::sim::DummyClass dummy2 = item.getData();
    dummy2.welcome();
}

