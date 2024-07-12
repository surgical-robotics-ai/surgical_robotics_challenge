Change log
==========

2.0.0 (2024-07-11)
==================

* API changes:
  * Add `SimulationManager` and `SimulationObject` classes to implement potential unit convention
  * Add individual limits for Robot Joint Control UI
  * Add joint limits for PSM and ECM, update the GUI as well
  * Use configuration files based on [dVRK package](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/tree/devel/share/tool) to load the kinematic parameters instead hard-coding
* Deprecated Features:
  * Remove JP recorder (Python based recording scripts), use ROS bag recorder instead
  * Remove the old synthetic phantom 
  * Remove unused files
* New Features:
  * Use `TransformStamped` and `PoseStamped` for cartesian space message types instead of the PyKDL data type
  * Add an MRI-scanned [3-Dmed Soft Tissue Suture Pad](https://www.3-dmed.com/product/soft-tissue-suture-pad/) into the simulation environment as the new phantom
  * Add `move_jp` and `move_cp` commands for PSMs (use interpolation to smooth the long-distance movements)
  * Add Razer Hydra PC Gaming Motion Sensing Controller as an alternative haptic teleoperation device
  * Add new motion recorder and replayer using ROS bags
  * Add the realistic [da Vinci Si PSM Large Needle Driver model](https://github.com/jhu-dvrk/instrument-cad) into the simulation
  * Redo the grasping algorithm using contact sensors
  * Detect tool type from an empty body in PSM yaml files
  * Modify the file names to include the instrument ID
  * Change the namespace of the files to make them self-explained
* Bug fixes:
  * Set the model level gravity to zero in order to improve the control accuracy
  * Rescaled simulation assets to SI units and tune the physical properties of the objects.(In version 1.0.0, the assets were scaled by 10)
  * Redo the mapping between the MTM gripper angles to the simulated PSM gripper angles
  * Cleaned up and reorganized directory structure

1.0.0 (2022-08-19)
==================

* No change log file, initial release
