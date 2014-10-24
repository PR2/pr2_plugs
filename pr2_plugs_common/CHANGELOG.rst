^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_plugs_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added install dependency from checkerboard_pose to find ros_detect.h
* Contributors: TheDash

1.0.20 (2014-10-24)
-------------------
* Moved set(orocos_include_dir to before find_package() so that it actually can find the package
* Contributors: TheDash

1.0.19 (2014-10-24)
-------------------
* Added set(orocos_kdl_DIR) to get around the fail orocos_kdl package, will not work if orocos is being installed from src since it looks in /opt/ros/hydro/share/orocos_kdl
* FIxed orocos_kdl and visual_pose not found by putting them after catkin_package()
* Contributors: TheDash

1.0.18 (2014-10-23)
-------------------
* Fixed orocos_kdl missing in pr2_plugs_common
* Contributors: TheDash

1.0.17 (2014-10-20)
-------------------

1.0.16 (2014-10-20)
-------------------

1.0.15 (2014-10-20)
-------------------

1.0.14 (2014-10-17)
-------------------
* Fixing orocos bug
* fixed orocos-kdl
* Contributors: TheDash

1.0.13 (2014-10-14)
-------------------
* Removed mainpage.dox
* Contributors: TheDash

1.0.12 (2014-10-10)
-------------------

1.0.11 (2014-10-10)
-------------------

1.0.10 (2014-09-18)
-------------------
* pr2_plugs is now in unstable, compiles
* ADded cmake version tag
* Fixed misname in CMake
* Contributors: TheDash, dash

* ADded cmake version tag
* Fixed misname in CMake
* Contributors: TheDash

1.0.9 (2014-09-17)
------------------

1.0.8 (2014-09-11)
------------------

1.0.7 (2014-09-11)
------------------

1.0.6 (2014-09-11)
------------------

1.0.5 (2014-09-08)
------------------

1.0.4 (2014-09-08)
------------------

1.0.3 (2014-09-08)
------------------
* removed kdl
* Contributors: TheDash

1.0.2 (2014-09-08)
------------------

1.0.1 (2014-09-08)
------------------
* Added install targets
* Generate changelogs
* now compiles in hydro
* Added
* Fixes
* fixes
* Fixed
* Catkin changes
* Linker error fix
* Fixed Cmake for linker errors
* Now compiles in hydro
* Remove makefile
* Catkinization fix
* pr2_plugs_common catkinization
* Current updates..
* remove custom controllers for plugs, and start using default controllers
* copy branch into trunk
* Contributors: TheDash, wim

* now compiles in hydro
* Added
* Fixes
* fixes
* Fixed
* Catkin changes
* Linker error fix
* Fixed Cmake for linker errors
* Now compiles in hydro
* Remove makefile
* Catkinization fix
* pr2_plugs_common catkinization
* Current updates..
* remove custom controllers for plugs, and start using default controllers
* copy branch into trunk
* Contributors: TheDash, wim
