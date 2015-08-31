rock_library(test_ItemPhysics                                         
  DEPS mars_sim                                                          
  DEPS_PKGCONFIG envire_core)                                                

rock_testsuite(test_suite suite.cpp                                             
  test_ItemPhysics.cpp                                                            
  DEPS mars_sim                                                            
  DEPS_PKGCONFIG envire_core)                                                
rock_testsuite(test_ItemManager suite.cpp
  test_ItemManager.cpp
  DEPS mars_sim
  DEPS_PKGCONFIG envire_core)
