# pr2_navigation_app
App the RobotWebServer that allows saving location of the PR2 in the map (compatible with RoboFlow)

# Installation instructions:
The following packages need to be installed on the system:

* pr2_2dnav
* robot_pose_publisher
* map_server
* tf2_web_republisher
* rosbridge_server

An accurate map of the area is also required. The path to map can be specified in the app.launch file.

There was a problem with using pr2_2dnav on hydro with nodes not being found. 
If the problem persists (should be fixed in the next deb update), 
copy bin folders from groovy to hydro for pr2_navigation_self_filter and semantic_point_annotator. 

# Launching the app:
If launching the app on its own (not as part of RWS):
on the robot, run 

*roslaunch rws_pr2_navigation app.launch*

and

*roslaunch rosbridge_server rosbridge_websocket.launch*

and 

*rosrun rws_pr2_navigation http_server.py*

Then open *http://c1.cs.washington.edu:8000/* in the browser of your choice.