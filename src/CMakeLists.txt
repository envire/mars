rock_library(mars
  SOURCES EnvireMars.cpp
  HEADERS EnvireMars.h)

rock_executable(mars_bin Main.cpp
    DEPS mars)

