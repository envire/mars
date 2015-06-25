<<<<<<< HEAD
![MARS](doc/src/images/logo_v2_wob.png)

MARS (Machina Arte Robotum Simulans) is a cross-platform simulation and visualisation tool created for robotics research. It runs on (Ubuntu) Linux, Mac and Windows and consists of a core framework containing all main simulation components, a GUI (based on [Qt]()), 3D visualization (using [OSG](http://www.openscenegraph.org)) and a physics core (based on [ODE](http://www.ode.org)).  
MARS is designed in a modular manner and can be used very flexibly, e.g. by running the physics simulation without visualization and GUI.
It is possible to extend MARS writing your own plugins and many plugins introducing various functionality such as HUDs or custom ground reaction forces already exist - and it's easy to write your own.

## Installation

We don't yet provide binaries for MARS, thus installing it requires downloading the source and building it locally. To simplify this task, we have created a set of install scripts that will do all the work of downloading and installing dependencies, cloning the MARS repository and building for you. They have their own repository here on GitHub: [MARS install scripts](https://github.com/rock-simulation/mars_install_scripts)

## Documentation

You can find MARS' documentation on its [GitHub Page](http://rock-simulation.github.io/mars).

## License

MARS is distributed under the [Lesser GNU Public License (LGPL)](https://www.gnu.org/licenses/lgpl.html).
=======
mars
=============
Envire plugin to be used by mars core

The plugin is used to handle the elements that are present in the simulation

License
-------
dummy-license

Installation
------------
The easiest way to build and install this package is to use Rock's build system.
See [this page](http://rock-robotics.org/stable/documentation/installation.html)
on how to install Rock.

However, if you feel that it's too heavy for your needs, Rock aims at having
most of its "library" packages (such as this one) to follow best practices. See
[this page](http://rock-robotics.org/stable/documentation/packages/outside_of_rock.html)
for installation instructions outside of Rock.

Rock CMake Macros
-----------------

This package uses a set of CMake helper shipped as the Rock CMake macros.
Documentations is available on [this page](http://rock-robotics.org/stable/documentation/packages/cmake_macros.html).

Rock Standard Layout
--------------------

This directory structure follows some simple rules, to allow for generic build
processes and simplify reuse of this project. Following these rules ensures that
the Rock CMake macros automatically handle the project's build process and
install setup properly.

STRUCTURE
-- src/ 
	Contains all header (*.h/*.hpp) and source files
-- build/
	The target directory for the build process, temporary content
-- bindings/
	Language bindings for this package, e.g. put into subfolders such as
   |-- ruby/ 
        Ruby language bindings
-- viz/
        Source files for a vizkit plugin / widget related to this library 
-- resources/
	General resources such as images that are needed by the program
-- configuration/
	Configuration files for running the program
-- external/
	When including software that needs a non standard installation process, or one that can be
	easily embedded include the external software directly here
-- doc/
	should contain the existing doxygen file: doxygen.conf
>>>>>>> c4ddd1e6b176253ff9c38a38cad7eb48536f401d
