#include "ItemPhysics.h"
#include <iostream>

using namespace std;

template <class T>
void mars::sim::MarsItem<T>::hello()
{
  cout << "You successfully created a templated class from a template. Hello!" << endl;
};

//void mars::sim::MarsItem<mars::sim::NodePhysics>::hello()
//{ 
//  cout << "You successfully created a class to allocate NodePhysics from a template. Hello!" << endl;
//};

