^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uol_cmp9767m_tutorial
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2019-12-03)
------------------
* Merge pull request `#46 <https://github.com/LCAS/CMP9767M/issues/46>`_ from gpdas/tutorial12_fixes
  Merging this. minor changes.
* remapping some topics
* Contributors: Gautham P Das, gpdas

0.5.2 (2019-11-18)
------------------
* Merge pull request `#44 <https://github.com/LCAS/CMP9767M/issues/44>`_ from LCAS/mapping
  Additional files for workshop 8. AMCL launch can now accept robot_name as a parameter.
* Merge branch 'master' into mapping
* Additional files for workshop 8. AMCL launch can now accept robot_name as a parameter.
* Merge pull request `#43 <https://github.com/LCAS/CMP9767M/issues/43>`_ from LCAS/w7_1920
  space fix planners
* space fix planners
* warning fix in planners config
* Contributors: Grzegorz Cielniak, gcielniak

0.5.1 (2019-10-29)
------------------

0.5.0 (2019-10-25)
------------------

0.4.3 (2019-10-15)
------------------

0.4.2 (2019-10-14)
------------------
* added tf_listener
* Contributors: Marc Hanheide

0.4.1 (2019-10-08)
------------------

0.4.0 (2019-10-07)
------------------
* added working mover
* more examples
* Contributors: Marc Hanheide

0.3.0 (2019-09-23)
------------------
* fixed new thorvald (`#37 <https://github.com/LCAS/CMP9767M/issues/37>`_)
* Merge pull request `#25 <https://github.com/LCAS/CMP9767M/issues/25>`_ from gpdas/tutorial_12
  Tutorial 12
* ros action related files
  1. added an example action DoDishes.action
  2. two scripts to demonstrate do_dishes action server and client
  3. one script to demonstrate topological_navigation action client
  4. minor changes to CMakeLists.txt and package.xml
* corrections to move_base_topo_nav.launch
  1. laser_scan_sensor topic and frame corrected for namespaced thorvalds
  2. footprint modified for the wide robot config
  3. create_new_topo_map.launch updated
* Tutorial 12 - Topological Navigation
  New dependency -> topological_navigation
  New launch files
  - move_base_topo_nav: for launching map_server, and namespaced versions of fake_localization, robot_pose_publisher and move_base
  - topo_nav: for launching topological_navigation related nodes
  - create_new_topo_map: for starting a new topo_map in the mongodb
  New configs
  - planner_topo_nav: TrajectoryPlannerROS is not supported by topological_navigation (yaw_goal_tolerance is not a reconfigurable parameter). So this one uses DWAPlannerROS by default. Some additional constraints on velocity(no Y and min_x=0)
  - planner: default set to a non-holonomic robot
  New topo_map -> test.yaml containing 6 nodes
* Merge pull request `#24 <https://github.com/LCAS/CMP9767M/issues/24>`_ from LCAS/tutorial10
  Merging. fake_localization fix. [jenkins build](https://lcas.lincoln.ac.uk/buildfarm/job/Kpr__uol_cmp9767m__ubuntu_xenial_amd64/33/) successfully completed, but somehow the status was not reached here.
* fake_localization fix
* Merge pull request `#23 <https://github.com/LCAS/CMP9767M/issues/23>`_ from LCAS/tutorial10
  Tutorial10 - had to merge myself ahead of the workshop.
* improved organisation of tutorial10 files
* rviz config
* tutorial10
* Tutorial9 (`#22 <https://github.com/LCAS/CMP9767M/issues/22>`_)
  * tutorial9 files
  * fixes
  * fixes 2
  * adding robot frame prefix to config files
  * using amcl instead of fake localization
  * improve simulation performance
* tutorial8 files (`#19 <https://github.com/LCAS/CMP9767M/issues/19>`_)
* Moved workshop files into a single repo (`#16 <https://github.com/LCAS/CMP9767M/issues/16>`_)
  * Moved workshop files into a single repo
  * Dependency fixes
  * Nodelet/PCL dependency fix
* Contributors: Gautham P Das, Grzegorz Cielniak, Marc Hanheide, gcielniak, gpdas

0.2.0 (2018-11-02)
------------------

0.1.2 (2018-11-02)
------------------

0.1.1 (2018-10-23)
------------------

0.1.0 (2018-10-23)
------------------
* Merge branch 'master' into master
* Merge pull request `#12 <https://github.com/LCAS/CMP9767M/issues/12>`_ from LCAS/rostest_marc
  added more meaningful rostests
* fixed install
* moved tests
* added more meaningful rostests
* simple testing
* Contributors: Marc Hanheide

* Merge branch 'master' into master
* Merge pull request `#12 <https://github.com/LCAS/CMP9767M/issues/12>`_ from LCAS/rostest_marc
  added more meaningful rostests
* fixed install
* moved tests
* added more meaningful rostests
* simple testing
* Contributors: Marc Hanheide

0.0.4 (2018-10-09 13:53)
------------------------

0.0.3 (2018-10-09 07:47)
------------------------

0.0.2 (2018-09-25)
------------------

0.0.1 (2018-09-24)
------------------
