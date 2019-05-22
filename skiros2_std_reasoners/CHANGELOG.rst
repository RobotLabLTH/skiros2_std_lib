^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package skiros2_std_reasoners
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fixed comments in spatial reasoner
* Spatial reasoner updates objects position based on velocity estimation
* Changed transform message from warn to info
* Contributors: RvmiLab, francesco

1.0.0 (2019-04-30)
------------------
* Init tf buffer from __init\_\_
* Spatial reasoner can compare poses that are not published on tf
* Removed debug msg
* Fix in spatial reasoner for using transform on ROS melodic
* Added skiros2_std_skill package with 2 utility skills: planner and action subscriber
* Bug-fix dealing with None values in hasData function
* Fixed doc tests
* Fix to geometry msgs.
* Contributors: RvmiLab, francesco

0.1.0 (2018-09-25)
------------------
* Bug-fix
* Added unknownT relation, when AllenIntervals cannot be calculated
* Added orientation relations and relative calculations
* Functions changed to snake case
* Reverted last commit
* FrameID is imposted on the elements label, when available.
* Changed function name
* Increased publishing rate at 50hz
* Now computing relations also with unpublished elements.
* Several bug fixes. Now transforming objects when comparing them for semantic relations calculation.
* Updating children when stopping/restarting publishing an object's pose
* Added PoseMsg property
* Added check for stop request
* Added allen intervals calculation and fixed several bugs (`#1 <https://github.com/RVMI/skiros2_std_lib/issues/1>`_)
  * Fixed parsing when removing an object. Removed spaces
  * Spatial reasoner loads all the scene at boot. Fixed bug with LinkedToFrameId property
  * Fix in Allen Intervals calculation (half of size)
* Fix in Allen Intervals calculation (half of size)
* Spatial reasoner loads all the scene at boot. Fixed bug with LinkedToFrameId property
* Fixed parsing when removing an object. Removed spaces
* First commit
* Contributors: Francesco Rovida, francesco
