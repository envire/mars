rock_library(mars
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(mars_bin Main.cpp
    DEPS mars)

