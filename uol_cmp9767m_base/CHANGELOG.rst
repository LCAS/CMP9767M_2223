^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uol_cmp9767m_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
