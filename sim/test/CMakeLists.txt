rock_library(test_ItemPhysics                                         
             DEPS mars_sim                                                          
             DEPS_PKGCONFIG envire_core)                                                

find_package(Boost COMPONENTS thread filesystem system)
rock_testsuite(test_suite 
               suite.cpp                                             
               test_ItemPhysics.cpp
               test_TreeMars.cpp                                                            
               DEPS mars_sim                                                           
               DEPS_PKGCONFIG envire_core)
               
target_link_libraries(test_suite
                      ${Boost_LIBRARIES})
                 
