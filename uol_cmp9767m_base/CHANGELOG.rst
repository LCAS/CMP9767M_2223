^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uol_cmp9767m_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2019-11-18)
------------------
* Merge branch 'master' into mapping
* Contributors: gcielniak

0.5.1 (2019-10-29)
------------------
* added switches for fake localisation (`#42 <https://github.com/LCAS/CMP9767M/issues/42>`_)
  * added switches for fake localisation
  * fixed test for new tf switch
* Contributors: Marc Hanheide

0.5.0 (2019-10-25)
------------------
* adding a walker into the simulation (`#39 <https://github.com/LCAS/CMP9767M/issues/39>`_)
  * actor with plugin
  * Update CMakeLists.txt
  * arguments to launch world with obstacles
* Contributors: Marc Hanheide

0.4.3 (2019-10-15)
------------------
* Assmnt1920 (`#41 <https://github.com/LCAS/CMP9767M/issues/41>`_)
  * new cropweed models
  * fixed texture height
* Contributors: gcielniak

0.4.2 (2019-10-14)
------------------
* new cropweed models (`#40 <https://github.com/LCAS/CMP9767M/issues/40>`_)
* Removed map_server from test
* Contributors: Marc Hanheide, gcielniak

0.4.1 (2019-10-08)
------------------
* added rviz and fixed frames for rviz (and map)
* Contributors: Marc Hanheide

0.4.0 (2019-10-07)
------------------
* multiple robots
* bug fix
* Friction
* Contributors: Marc Hanheide

0.3.0 (2019-09-23)
------------------
* fixed new thorvald (`#37 <https://github.com/LCAS/CMP9767M/issues/37>`_)
* Tutorial9 (`#22 <https://github.com/LCAS/CMP9767M/issues/22>`_)
  * tutorial9 files
  * fixes
  * fixes 2
  * adding robot frame prefix to config files
  * using amcl instead of fake localization
  * improve simulation performance
* Fix greg (`#17 <https://github.com/LCAS/CMP9767M/issues/17>`_)
  * Moved workshop files into a single repo
  * Dependency fixes
  * Nodelet/PCL dependency fix
  * Fixed warning: package uol_cmp9767m_base should not depend on metapackage thorvald_simulator but on its packages instead
* Contributors: Marc Hanheide, gcielniak

0.2.0 (2018-11-02)
------------------
* added sprayer (`#15 <https://github.com/LCAS/CMP9767M/issues/15>`_)
  * added sprayer
  * added sprayer into model
  * textures with alpha and better lighting
* Contributors: Marc Hanheide

0.1.2 (2018-11-02)
------------------
* navtest added (`#14 <https://github.com/LCAS/CMP9767M/issues/14>`_)
* Contributors: Marc Hanheide

0.1.1 (2018-10-23)
------------------
* Update package.xml
* Contributors: Marc Hanheide

0.1.0 (2018-10-23)
------------------
* fixed test
* Merge pull request `#13 <https://github.com/LCAS/CMP9767M/issues/13>`_ from gpdas/master
  sensor.xacro updated with working velodyne and kinect2.
* fixed `gui` conflict
* Merge branch 'master' into master
* sensor.xacro updated with velodyne and kinect2.
  raw urdf from the <sensor>_description added to sensors.xacro to avoid problems with tf_prefix and topic_names
* Merge pull request `#12 <https://github.com/LCAS/CMP9767M/issues/12>`_ from LCAS/rostest_marc
  added more meaningful rostests
* fixed install
* moved tests
* added more meaningful rostests
* Merge pull request `#7 <https://github.com/LCAS/CMP9767M/issues/7>`_ from LCAS/initial_map
  added cropped map
* added cropped map
* Contributors: Marc Hanheide, gpdas

0.0.4 (2018-10-09)
------------------
* Merge pull request `#5 <https://github.com/LCAS/CMP9767M/issues/5>`_ from gpdas/master
  Updated ground textures
* Cleanup
* Merge branch 'master' of github.com:LCAS/CMP9767M
* 1. New ground textures and enclosure are added
  2. Fixed missing install targets in CMakeLists
  3. World file updated with models with new textures
  4. New wider robot configuration within the package
  5. Hokuyo laser ray visibility in gazebo is disabled
* Contributors: Marc Hanheide, gpdas

0.0.3 (2018-10-09)
------------------
* Merge pull request `#4 <https://github.com/LCAS/CMP9767M/issues/4>`_ from gpdas/master
  Adding sensors to the simulated robot
* Initial world file for uol_cmp9767m_base
  ground texture model added
  new world file to use the new ground model
  launch file updated to load the new world
  CMakeLists updated with a hook to copy the models directory
* dependency correction
* fixes to hokuyo and velodyne parameters
* Fix in velodyne parameters
* Adding hokuyo and velodyne sensors in simulation.
* Contributors: Marc Hanheide, gpdas

0.0.2 (2018-09-25)
------------------
* Merge pull request `#1 <https://github.com/LCAS/CMP9767M/issues/1>`_ from gpdas/master
  Modified thorvald-sim.launch to launch robot in an empty world
* Modified thorvald-sim.launch to launch robot in an empty world
* Contributors: Marc Hanheide, gpdas

0.0.1 (2018-09-24)
------------------
* fixed cmake
* added stub package
* Contributors: Marc Hanheide
