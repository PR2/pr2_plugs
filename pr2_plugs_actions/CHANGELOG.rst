^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_plugs_actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.2 (2014-09-08)
------------------
* Removed kdl dependency
* Contributors: TheDash

1.0.1 (2014-09-08)
------------------
* Added install targets
* Generate changelogs
* Bug fixes
* bugs
* Added
* Catkinization and hydro compilation
* Removed makefile
* Current updates..
* Removed manifest xml
* fixed InvalidUserCodeError (using ud.gripper_to_plug when it's not in input_keys) bug
* catching various tf2 exceptions and doing one re-try
* Fixed typo "bg8" -> "bgr8"
* use the C++ cv_bridge API (as the C got deprecated)
* fixed turning on of textured projector
* disabled launchfile for base planning - it hasn't been updated to work with fuerte
* Update link flags for OpenCV on Fuerte.
* Don't print so much `#5151 <https://github.com/PR2/pr2_plugs/issues/5151>`_
* fix warnings releated to automatically starting the actionlib server
* rosdep for opencv
* use cmake to find opencv
* merge wall extractor changes from electric-dev
* be a little more patient when waiting for transforms
* Update plugs launch files; fix error in pcl wall extractor
* working with fuerte
* Stow plug error tolerance increase, add max iterations. wg-ros-pkg5180
* Downloading 40MB outlet templates tarball into build directory, wg-ros-pkg 5178
* patch to free base after unplugging, and fix issues with previous patches applied for moving the base closer to the wall
* temporary comment out test with spurious failures
* patch by Kevin to move robot closer to wall during charging, and stow left arm `#5156 <https://github.com/PR2/pr2_plugs/issues/5156>`_
* Correcting head pointing goal frame
* add example launch file
* update to using action interface. `#5135 <https://github.com/PR2/pr2_plugs/issues/5135>`_
* same fixes for unplug
* fixed
* update topic name
* remove local approach pose from list, and add by default to support an empty location list
* move recharge locations out of default launch file
* getting ready for 0.4.2 release
* update comment
* add web adaptor
* recharge application checks if robot is really plugged in by monitorying the ac present status
* select new charge location every time
* recharge application does not look at battery levels any more
* recharge application server automatically plugs in robot, and is only willing to preempt when the robot is fully charged
* Header fix
* provide more detailled output
* rename recharnge monitor to application
* working version
* set state back to original state on failure
* fix scope
* monitor plug in state, and refuse stupid commands
* add plug application launch file
* fixes
* update monitor
* recharge monitor passes through goals to recharger. this prevents race conditions in preemption of app
* Fixing the wrong node name in the plugs launch file
* first version of recharge application monitor
* take image of failing detect plug after stow
* add image snapshotter to stow plug recovery
* recovery for stowing plug
* add copyright header
* add dependency on kdl
* add buffer server to test launch files
* add remap for buffer server
* start buffer server for plugs, to allow release in cturtle
* working version
* first port to tf2, first port away from posestampedmath
* Now calls pr2_move_base
* Adding an outlet location that is on the other side of the narrow hallway
* fix problem with rough align when fails to detect outlet after fine approach. Ticket 4830
* Do sanity check on wall norm
* When stowin plug, give up detecting plug on base after 5 minutes
* projector now works in sim
* move tf_utils from executive_smach to pr2_plugs_actions. Tickets `#4705 <https://github.com/PR2/pr2_plugs/issues/4705>`_ and `#4707 <https://github.com/PR2/pr2_plugs/issues/4707>`_
* update plugs regression tests
* add sim calibration params
* never give up finding plug on base
* give detect plug on base more time
* no debugging by default
* mark as executable
* add debug info
* catch service exceptions of dynamic reconfigure
* Remove Plugin action and old script that used it, and updated launch files
* load plug description
* don't get too close to the wall
* revert offset added to fetch plug
* load robot specific calibration using args to launch files
* major cleanup of userdata in plugin action
* big cleanup in launch files
* clean up launch file
* fixed dropping the plug after failing to plug in
* adjusting grasp for fetching the plug
* fixing the look at wall point to be a function of the distance from the wall and move the robot back from the wall to get more of a viewing angle for finding the plug
* working on robot
* number of bugfixes for plugging in
* removing joint traj state
* Updating pr2_plugs_actions for new smach stack and refactored smach (pending testing)
* Finishing plugs smach 0.3 updates
* Fixing ud key access
* Fixes to the plugs use of SMACH
* Updating plugs to work with new SMACH api
* rename arm ik action
* update to new arm ik api
* Fixing permissions in plugs calibration
* load joint trajectories for test
* Updates to plugs reflecting smach actionserver wrapper changes
* new plug in gripper detection positions to avoid windup of plug cord
* Making the robot stop twisting the cord up
* Updates to plugs, fixing some stuff that used the old SMACH api
* Removed feature.
* Re-adding tfutil instances and other things
* remove custom controllers for plugs, and start using default controllers
* Updating introspection path for plug_in action
* Reverting introspection nesting specification
* Moving plugs SMACH components into actions, cleaning up a bit
* Improvimg unplug behavior, adding some more recovery pathways, fixing a hack in app_unplug
* Moving more stuff into smach, fixing transform calculations
* More iterator testing / failure recovery
* Fixed the twist
* Moving more plugs components over to SMACH
* New expanded smach features in plugs
* Removing tf util, fixing typo
* Fixing some bugs
* New plug_in sm
* Beginning to expand some of the older scripts into smach sm's... experimenting with an iterator container
* Switching plugs actions over to imported smach containers
* Final changes from last API review and updates for plugs
* Applying updates from pr2 launch party demo
* fix a whole bunch of problems to get plugging  in working with latest smach api
* temp fix for tolerance on plug on base pose
* tix syntax
* update to new api
* update state machines to new smach api
* copy branch into trunk
* Contributors: Austin Hendrix, Bhaskara Marthi, Kaijen Hsiao, TheDash, Vincent Rabaud, Wim Meeussen, eitan, hsu, jbinney, jbohren, kevinwwatts, kwc, marioprats, mwise, wim

* Bug fixes
* bugs
* Added
* Catkinization and hydro compilation
* Removed makefile
* Current updates..
* Removed manifest xml
* fixed InvalidUserCodeError (using ud.gripper_to_plug when it's not in input_keys) bug
* catching various tf2 exceptions and doing one re-try
* Fixed typo "bg8" -> "bgr8"
* use the C++ cv_bridge API (as the C got deprecated)
* fixed turning on of textured projector
* disabled launchfile for base planning - it hasn't been updated to work with fuerte
* Update link flags for OpenCV on Fuerte.
* Don't print so much `#5151 <https://github.com/PR2/pr2_plugs/issues/5151>`_
* fix warnings releated to automatically starting the actionlib server
* rosdep for opencv
* use cmake to find opencv
* merge wall extractor changes from electric-dev
* be a little more patient when waiting for transforms
* Update plugs launch files; fix error in pcl wall extractor
* working with fuerte
* Stow plug error tolerance increase, add max iterations. wg-ros-pkg5180
* Downloading 40MB outlet templates tarball into build directory, wg-ros-pkg 5178
* patch to free base after unplugging, and fix issues with previous patches applied for moving the base closer to the wall
* temporary comment out test with spurious failures
* patch by Kevin to move robot closer to wall during charging, and stow left arm `#5156 <https://github.com/PR2/pr2_plugs/issues/5156>`_
* Correcting head pointing goal frame
* add example launch file
* update to using action interface. `#5135 <https://github.com/PR2/pr2_plugs/issues/5135>`_
* same fixes for unplug
* fixed
* update topic name
* remove local approach pose from list, and add by default to support an empty location list
* move recharge locations out of default launch file
* getting ready for 0.4.2 release
* update comment
* add web adaptor
* recharge application checks if robot is really plugged in by monitorying the ac present status
* select new charge location every time
* recharge application does not look at battery levels any more
* recharge application server automatically plugs in robot, and is only willing to preempt when the robot is fully charged
* Header fix
* provide more detailled output
* rename recharnge monitor to application
* working version
* set state back to original state on failure
* fix scope
* monitor plug in state, and refuse stupid commands
* add plug application launch file
* fixes
* update monitor
* recharge monitor passes through goals to recharger. this prevents race conditions in preemption of app
* Fixing the wrong node name in the plugs launch file
* first version of recharge application monitor
* take image of failing detect plug after stow
* add image snapshotter to stow plug recovery
* recovery for stowing plug
* add copyright header
* add dependency on kdl
* add buffer server to test launch files
* add remap for buffer server
* start buffer server for plugs, to allow release in cturtle
* working version
* first port to tf2, first port away from posestampedmath
* Now calls pr2_move_base
* Adding an outlet location that is on the other side of the narrow hallway
* fix problem with rough align when fails to detect outlet after fine approach. Ticket 4830
* Do sanity check on wall norm
* When stowin plug, give up detecting plug on base after 5 minutes
* projector now works in sim
* move tf_utils from executive_smach to pr2_plugs_actions. Tickets `#4705 <https://github.com/PR2/pr2_plugs/issues/4705>`_ and `#4707 <https://github.com/PR2/pr2_plugs/issues/4707>`_
* update plugs regression tests
* add sim calibration params
* never give up finding plug on base
* give detect plug on base more time
* no debugging by default
* mark as executable
* add debug info
* catch service exceptions of dynamic reconfigure
* Remove Plugin action and old script that used it, and updated launch files
* load plug description
* don't get too close to the wall
* revert offset added to fetch plug
* load robot specific calibration using args to launch files
* major cleanup of userdata in plugin action
* big cleanup in launch files
* clean up launch file
* fixed dropping the plug after failing to plug in
* adjusting grasp for fetching the plug
* fixing the look at wall point to be a function of the distance from the wall and move the robot back from the wall to get more of a viewing angle for finding the plug
* working on robot
* number of bugfixes for plugging in
* removing joint traj state
* Updating pr2_plugs_actions for new smach stack and refactored smach (pending testing)
* Finishing plugs smach 0.3 updates
* Fixing ud key access
* Fixes to the plugs use of SMACH
* Updating plugs to work with new SMACH api
* rename arm ik action
* update to new arm ik api
* Fixing permissions in plugs calibration
* load joint trajectories for test
* Updates to plugs reflecting smach actionserver wrapper changes
* new plug in gripper detection positions to avoid windup of plug cord
* Making the robot stop twisting the cord up
* Updates to plugs, fixing some stuff that used the old SMACH api
* Removed feature.
* Re-adding tfutil instances and other things
* remove custom controllers for plugs, and start using default controllers
* Updating introspection path for plug_in action
* Reverting introspection nesting specification
* Moving plugs SMACH components into actions, cleaning up a bit
* Improvimg unplug behavior, adding some more recovery pathways, fixing a hack in app_unplug
* Moving more stuff into smach, fixing transform calculations
* More iterator testing / failure recovery
* Fixed the twist
* Moving more plugs components over to SMACH
* New expanded smach features in plugs
* Removing tf util, fixing typo
* Fixing some bugs
* New plug_in sm
* Beginning to expand some of the older scripts into smach sm's... experimenting with an iterator container
* Switching plugs actions over to imported smach containers
* Final changes from last API review and updates for plugs
* Applying updates from pr2 launch party demo
* fix a whole bunch of problems to get plugging  in working with latest smach api
* temp fix for tolerance on plug on base pose
* tix syntax
* update to new api
* update state machines to new smach api
* copy branch into trunk
* Contributors: Austin Hendrix, Bhaskara Marthi, Kaijen Hsiao, TheDash, Vincent Rabaud, Wim Meeussen, eitan, hsu, jbinney, jbohren, kevinwwatts, kwc, marioprats, mwise, wim
