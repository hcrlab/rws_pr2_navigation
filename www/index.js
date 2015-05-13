var ros = new ROSLIB.Ros({
	url : 'ws://' + window.location.hostname + ':9090'
  });


var navPub = new ROSLIB.Topic({
	ros : ros,
	name : '/navigation_command',
	messageType : 'rws_pr2_navigation/NavigationCommand'
});


var batteryStateListener = new ROSLIB.Topic({
	ros : ros,
	name : '/battery/server',
	messageType : 'pr2_msgs/BatteryServer'
});

var expListener = new ROSLIB.Topic({
	ros : ros,
	name : '/nav_system_state',
	messageType : 'rws_pr2_navigation/NavSystemState'
});

var estimatedPoseListener = new ROSLIB.Topic({
	ros : ros,
	name : '/particlecloud',
	messageType : 'geometry_msgs/PoseArray'
});

var expListenerSrvCli = new ROSLIB.Service({
	ros : ros,
	name : '/get_nav_system_state',
	serviceType : 'rws_pr2_navigation/GetNavSystemState'
});

var isPoseInitialized = false;

// setup the initial pose publisher
var initialPosePub = new ROSLIB.Topic({
	ros : ros,
	name : '/initialpose',
	messageType : 'geometry_msgs/PoseWithCovarianceStamped'
});

// for detecting single vs double clicks on saved locations
var clicks = 0;

var initializeMode = false;

/**
   * Send the initial pose of the robot to the navigation stack with the given pose.
   *
   * @param pose - the initial pose
*/
function setInitialPose(pose) {
    // create a pose with covariance stamped message
    var initPoseMsg = new ROSLIB.Message({
        header: {
          frame_id: '/map',
        },
        pose: {
          pose: pose,
          covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06]
        }

    });
    initialPosePub.publish(initPoseMsg);
}

/**
   * Sets the current location with the given pose.
*/
function setLocation(pose) {
    navPub.publish(new ROSLIB.Message({
        command: "set-pose",
        pose: pose
    }));
}

function init() {
    ros.on("error", function() {
        alert("Error connecting to the ROS server. App will not work.")
    });

    ros.on('connection', function() {
	console.log('Connected to websocket server.');
    });

    ros.on('close', function() {
	console.log('Connection to websocket server closed.');
    });

    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'nav',
      width : 750,
      height : 800
    });

    processPose = NAV2D.Navigator.sendGoal;
    // By default use map as navigation tool: processPose == NAV2D.Navigator.sendGoal.
    // If other controls are checked, change the function.
    /*if (document.querySelector("#setInitRadioBtn").checked) {
	processPose = setInitialPose;
    } else if (document.querySelector("#setLocationRadioBtn").checked) {
	processPose = setLocation;
    }*/

    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      withOrientation : true,
      serverName : '/pr2_move_base',
      processPose : processPose
    });

    document.querySelector("#initializePose").addEventListener("click", function() {
	alert(initializeMode);
	if (!initializeMode) {
	    initializeMode = true;
	    this.style.color = "red";
	    NAV2D.Navigator.processPose = setInitialPose;
	} else {
	    initializeMode = false;
	    this.style.color = "green";
	    NAV2D.Navigator.processPose = NAV2D.Navigator.sendGoal;
	}
    });
    // Setup the controls for the map.
    /*document.querySelector("#setGoalRadioBtn").addEventListener("click", function() {
	NAV2D.Navigator.processPose = NAV2D.Navigator.sendGoal;
    });
    document.querySelector("#setInitRadioBtn").addEventListener("click", function() {
	NAV2D.Navigator.processPose = setInitialPose;
    });
    document.querySelector("#setLocationRadioBtn").addEventListener("click", function() {
	NAV2D.Navigator.processPose = setLocation;
    }); */


    //hook up buttons with com attribute to navigation commands
    [].slice.call(document.querySelectorAll("img[com]")).forEach(function(el) {
	el.addEventListener("click", function() {
	    var relCom = new ROSLIB.Message({
		command : el.getAttribute("com")
	    });
	    navPub.publish(relCom);
	});
    });

    function toggleControls(turnOn) {
        // Enable or disable navigation-related controls.
        if (turnOn) {
            document.querySelector("#navigateBtn").removeAttribute("disabled");
        } else {
            document.querySelector("#navigateBtn").setAttribute("disabled", true);
        }
    }

    function processBatteryState(state) {
        // If robot is plugged in, display warning and turn off navigation controls.
        var plugWarningSpan = document.querySelector('div[id="plugwarning"]');
	if (state.discharging == 0) {
	    toggleControls(false);
	    plugWarningSpan.style.display = "";
	} else {
	    toggleControls(true);
	    plugWarningSpan.style.display = "none";
	}
    }
    
    var current_location; // the current location selected
    function handleLocationClick(self, loc_n) {
	clicks++;
	if (clicks === 1) {
	    timer = setTimeout(function() {
		current_location = self.innerHTML;
		navPub.publish(new ROSLIB.Message({
		    command: "switch-to-location",
		    param: loc_n
		}));
		clicks = 0;
	    }, 500);
	} else {
	    clearTimeout(timer);    //prevent single-click action
	    // Don't allow user to rename if location is not the one currently selected
	    if (current_location !== self.innerHTML) {
		return;
	    }
	    rename(self);  //perform double-click action
	    clicks = 0;
	}
    }    

    var removed = false;
    function rename(self) {
	var editableText = document.createElement("textarea");
	editableText.value = self.innerHTML;
	self.parentNode.replaceChild(editableText, self);
	editableText.focus();
	removed = false;
	editableText.addEventListener("keyup", function() { renameEnter(editableText) });
	editableText.addEventListener("blur", function() { renameBlurred(editableText) });
    }
    
    function renameEnter(self) {
	if (window.event.keyCode == 13) {
	    renameBlurred(self);
	}
    }


    function renameBlurred(self) {
	if (removed) return; // because removing a node fires this again
	removed = true;
	var new_name = self.value;
	var dv = document.createElement("div");
	dv.innerHTML = new_name;
	self.parentNode.replaceChild(dv, self);
	dv.addEventListener("click", function() {
	    handleLocationClick(this, new_name);
	}); 
	navPub.publish(new ROSLIB.Message({
			command: "name-location",
			param: new_name
	}));
    }

    batteryStateListener.subscribe(processBatteryState);

    // modals
    var plug_modal = document.querySelector("#plugwarningdialog");
    document.querySelector("#plugwarning>img").addEventListener("click", function() {
	plug_modal.showModal();
	document.querySelector("body").setAttribute("class", "blur");
    });
    document.querySelector("#plugwarningclose").addEventListener("click", function() {
	plug_modal.close();
	document.querySelector("body").removeAttribute("class", "blur");
    });

    var init_modal = document.querySelector("#initwarningdialog");
    document.querySelector("#initwarning>img").addEventListener("click", function() {
	init_modal.showModal();
	document.querySelector("body").setAttribute("class", "blur");
    });
    document.querySelector("#initwarningclose").addEventListener("click", function() {
	init_modal.close();
	document.querySelector("body").removeAttribute("class", "blur");
    });

    var localized_modal = document.querySelector("#localizedwarningdialog");
    document.querySelector("#localizedwarning>img").addEventListener("click", function() {
	localized_modal.showModal();
	document.querySelector("body").setAttribute("class", "blur");
    });
    document.querySelector("#localizedwarningclose").addEventListener("click", function() {
	localized_modal.close();
	document.querySelector("body").removeAttribute("class", "blur");
    });
   

    var position = null;
    var locationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;
    var rotateLocationMarker = false;
    // click and drag oh yah!
    var locationEventHandler = function(event, mouseState) {
	var stage = viewer.scene;
	// TODO: fix this broken-ness
	if (mouseState === 'dblclick') {
	    rotateLocationMarker = !rotateLocationMarker;	
	    var color;
	    if (rotateLocationMarker) {
		color = createjs.Graphics.getRGB(128, 0, 0, 0.66);
	    } else {
		color = createjs.Graphics.getRGB(128, 128, 0, 0.66);
	    }
	    var x = locationMarker.x;
	    var y = locationMarker.y;
	    var rotation = locationMarker.rotation;
	    stage.removeChild(locationMarker);
	    locationMarker = new ROS2D.NavigationArrow2({
		size : 20,
		strokeSize : 1,
		fillColor : color,
		pulse : false
	    });
	    locationMarker.x = x;
	    locationMarker.y = y;
	    locationMarker.rotation = rotation;
	    locationMarker.scaleX = 1.0 / stage.scaleX;
	    locationMarker.scaleY = 1.0 / stage.scaleY;
	    locationMarker.addEventListener('dblclick', function(event) { locationEventHandler(event, 'dblclick'); });
	    locationMarker.addEventListener('mousedown', function(event) { locationEventHandler(event, 'down'); });
	    locationMarker.addEventListener('pressmove', function(event) { locationEventHandler(event, 'move'); });
	    locationMarker.addEventListener('pressup', function(event) { locationEventHandler(event, 'up'); });
	    stage.addChild(locationMarker);
	    return;
	} 
	position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);
	if (mouseState === 'down') {
	    // really nothing to do here - maybe change opacity?
	} else if (mouseState === 'move') {
	    if (!rotateLocationMarker) {
		var rotation = locationMarker.rotation;
		stage.removeChild(locationMarker);
		locationMarker = new ROS2D.NavigationArrow2({
		    size : 20,
		    strokeSize : 1,
		    fillColor : createjs.Graphics.getRGB(128, 128, 0, 0.66),
		    pulse : false
		});
		locationMarker.x =  positionVec3.x;
		locationMarker.y = -positionVec3.y;
		locationMarker.rotation = rotation;
		locationMarker.scaleX = 1.0 / stage.scaleX;
		locationMarker.scaleY = 1.0 / stage.scaleY;
		stage.addChild(locationMarker);
	    } else {
		console.log("should rotate");
	    }
	} else { // press up
	    locationMarker.addEventListener('dblclick', function(event) { locationEventHandler(event, 'dblclick'); });
	    locationMarker.addEventListener('mousedown', function(event) { locationEventHandler(event, 'down'); });
	    locationMarker.addEventListener('pressmove', function(event) { locationEventHandler(event, 'move'); });
	    locationMarker.addEventListener('pressup', function(event) { locationEventHandler(event, 'up'); });
	    
	    var thetaRadians = locationMarker.rotation * Math.PI / 180.0;
	    var qz =  Math.sin(-thetaRadians/2.0);
	    var qw =  Math.cos(-thetaRadians/2.0);
	    
	    // TODO: fix this shit!
	    /*
	      var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});

	      var pose = new ROSLIB.Pose({
	      position :    positionVec3,
	      orientation : orientation
	      });
	      var p = NAV2D.Navigator.processPose;
	      NAV2D.Navigator.processPose = setLocation;
	      NAV2D.Navigator.processPose(pose);
	      NAV2D.Navigator.processPose = p;
	    */
	}
    }

    var locListCont = document.querySelector("#locationList");

    locationMarker = new ROS2D.NavigationArrow2({
        size : 20,
        strokeSize : 1,
        fillColor : createjs.Graphics.getRGB(128, 128, 0, 0.66),
        pulse : false
    });
    locationMarker.visible = false;
    viewer.scene.addChild(locationMarker);
    // Code for drawing the list of locations.
    var drawState = function(state) {
	//draw location list
	locListCont.innerHTML = "";
	state.location_names.forEach(function(loc_n) {
	    var dv = document.createElement("div");
	    dv.innerHTML = loc_n;
	    dv.addEventListener("click", function() {
		handleLocationClick(this, loc_n);
	    });
	    var li = document.createElement("li");
	    li.appendChild(dv);
	    locListCont.appendChild(li);
	});
	// If there were no saved locations, say so:
	if (state.location_names.length == 0) {
	    locListCont.innerHTML = "(none)"
	}
	if (state.current_location != -1) {
	    // If a location is selected, enable and create buttons for its manipulation.
	    document.querySelector("#deleteBtn").removeAttribute("disabled");
	    document.querySelector("#navigateBtn").removeAttribute("disabled");

	    var current_dv = locListCont.querySelectorAll("div")[state.current_location];
	    current_dv.className = "selected";
	    current_location = current_dv.innerHTML;
	    // update the location on the map
	    locationMarker.x = state.current_location_pose.position.x;
	    locationMarker.y = -state.current_location_pose.position.y;
	    locationMarker.scaleX = 1.0 / viewer.scene.scaleX;
	    locationMarker.scaleY = 1.0 / viewer.scene.scaleY;
	    // change the angle
	    locationMarker.rotation = viewer.scene.rosQuaternionToGlobalTheta(state.current_location_pose.orientation);
	    locationMarker.visible = true;
	    // event handlers
	    locationMarker.addEventListener('dblclick', function(event) { locationEventHandler(event, 'dblclick'); });
	    locationMarker.addEventListener('mousedown', function(event) { locationEventHandler(event, 'down'); });
	    locationMarker.addEventListener('pressmove', function(event) { locationEventHandler(event, 'move'); });
	    locationMarker.addEventListener('pressup', function(event) { locationEventHandler(event, 'up'); });
	} else {
	    // If no location is selected, disable buttons that operate on current location.
	    document.querySelector("#deleteBtn").setAttribute("disabled", true);
	    document.querySelector("#navigateBtn").setAttribute("disabled", true);
	}
    };

    var processPoseArray = function(poseArray) {
	if (isPoseInitialized == false) {
	    isPoseInitialized = true;
            var initWarningSpan = document.querySelector('div[id="initwarning"]');
	    initwarning.style.display = "none";
	}
        var localizedwarning = document.querySelector('div[id="localizedwarning"]');
	poses = poseArray.poses;
	if (poses.length < 600) {
	    localizedwarning.style.display = "none";
	} else {
	    localizedwarning.style.display = "";
	}
    }

    expListener.subscribe(function(state) {
	drawState(state);
    });
    
    estimatedPoseListener.subscribe(function(poseArray) {
	processPoseArray(poseArray);
    });

    expListenerSrvCli.callService(new ROSLIB.ServiceRequest({}), function(result) {
	drawState(result.state);
    });
}