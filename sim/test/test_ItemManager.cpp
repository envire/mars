#include <boost/test/unit_test.hpp>
#include <mars/interfaces/sim/ControlCenter.h>
#include "ItemManager.h"

using namespace mars::sim;

BOOST_AUTO_TEST_CASE(contructors)
{
    mars::interfaces::ControlCenter* control; // How to initialize?
    mars::sim::ItemManager itemManager(control);
    std::cout << "Constructor Test" << std::endl;
}

BOOST_AUTO_TEST_CASE(showAbox)
{
    mars::interfaces::ControlCenter control;
    mars::sim::ItemManager itemManager(&control);
    itemManager.test();
    std::cout << "Test Item Manager" << std::endl;
}
//
//BOOST_AUTO_TEST_CASE(set_get)
//{
//    mars::sim::WorldPhysics physics(NULL);
//    mars::sim::ItemPhysics item(&physics);
//    mars::sim::WorldPhysics physics2(NULL);
//    mars::sim::NodePhysics node(&physics2);
//    item.setData(node);
//    std::cout << "Set data done" << std::endl;
//    BOOST_ASSERT(item.getData() == node);
//    std::cout << "Get data done" << std::endl;
//    mars::sim::NodePhysics otherNode(&physics);
//    BOOST_ASSERT(item.getData() != otherNode);
//}
/*
 * This is the goal test 
BOOST_AUTO_TEST_CASE(first_object)
{
    mars::sim::WorldPhysics physics(NULL);
    mars::sim::ItemPhysics item(&physics);
    mars::sim::NodePhysics node = item.getData();
    ode.compute(node); //Or however is done
    //mars::sim::SurfaceItemPhysics item(&physics);
    //mars::sim::SurfacePhysics surface = item.getData();
    //ode.compute(surface);
}
*/
