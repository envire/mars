#include <boost/test/unit_test.hpp>
#include <mars/Dummy.hpp>

using namespace mars;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    mars::DummyClass dummy;
    dummy.welcome();
}
