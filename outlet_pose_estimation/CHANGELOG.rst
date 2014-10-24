^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package outlet_pose_estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.21 (2014-10-24)
-------------------

1.0.20 (2014-10-24)
-------------------

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
* Linked outlet_pose_estimation to catkin_LIBRARIES
* Contributors: TheDash

1.0.15 (2014-10-20)
-------------------

1.0.14 (2014-10-17)
-------------------

1.0.13 (2014-10-14)
-------------------
* |Compiles now
* Contributors: TheDash

1.0.12 (2014-10-10)
-------------------

1.0.11 (2014-10-10)
-------------------
* Added library install for outlet_pose_estimation
* Contributors: TheDash

1.0.10 (2014-09-18)
-------------------
* Fixed up CMakeLists in outlet_pose
* pr2_plugs is now in unstable, compiles
* things
* Contributors: TheDash, dash

* things
* Contributors: TheDash

1.0.9 (2014-09-17)
------------------

1.0.8 (2014-09-11)
------------------

1.0.7 (2014-09-11)
------------------

1.0.6 (2014-09-11)
------------------
* Added cleaner CMakeLists for stereo_wall_detection and fixed bugs inside of it found using catkin_lint
* Contributors: TheDash

1.0.5 (2014-09-08)
------------------

1.0.4 (2014-09-08)
------------------

1.0.3 (2014-09-08)
------------------

1.0.2 (2014-09-08)
------------------

1.0.1 (2014-09-08)
------------------
* ADded install targets
* Generate changelogs
* Exported libraries
* Exported library
* compiles
* added include
* catkin
* Catkinization
* catkinization of pr2_*
* Add another dep on openCV
* Reverting some changes in the outlet pose estimation that now reflect the current opencv api
* Fixing compiler problems after a change in opencv folder structure
* copy branch into trunk
* Contributors: Austin Hendrix, TheDash, jbohren, relrotciv, wim

* Exported libraries
* Exported library
* compiles
* added include
* catkin
* Catkinization
* catkinization of pr2_*
* Add another dep on openCV
* Reverting some changes in the outlet pose estimation that now reflect the current opencv api
* Fixing compiler problems after a change in opencv folder structure
* copy branch into trunk
* Contributors: Austin Hendrix, TheDash, jbohren, relrotciv, wim
