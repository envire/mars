#include "ItemPhysics.h"
#include <iostream>

using namespace std;

void mars::sim::ItemPhysics::hello()
{
    cout << "You successfully Inherited from a class template. Hello!" << endl;
}

void mars::sim::DummyClass::welcome()
{
    cout << "This is a method of the class to which the template was adapted" << endl;
}
