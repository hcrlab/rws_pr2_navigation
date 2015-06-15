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

// true if Initialize Robot Pose button is pressed
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
    processPose = NAV2D.Navigator.setInitialPose;

    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      withOrientation : true,
      serverName : '/pr2_move_base',
      processPose : processPose
    });

    // initialize robot pose interaction - when activated, the user can select the pose
    // on the map and the other controls are blurred out. When deactivated the interaction
    // returns to normal
    document.querySelector("#initializePose").addEventListener("click", function() {
	if (!initializeMode) {
	    initializeMode = true;
	    document.querySelector("#locationControls").className = "blur";
	    this.className = "depressed";
	    NAV2D.Navigator.processPose = setInitialPose;
	} else {
	    initializeMode = false;
	    document.querySelector("#locationControls").className = "";
	    this.className = "";
	    NAV2D.Navigator.processPose = NAV2D.Navigator.sendGoal;
	}
    });

    //hook up buttons with com attribute to navigation commands
    [].slice.call(document.querySelectorAll("img[com]")).forEach(function(el) {
	el.addEventListener("click", function() {
	    var relCom = new ROSLIB.Message({
		command : el.getAttribute("com")
	    });
	    navPub.publish(relCom);
	    if (goalMarkerOnScreen) {
		storeCurrentLocation(goalMarkerPos, goalMarker.rotation * (Math.PI / 180.0)); 
	    }
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
    
    // locations have both click and double click interaction
    // single click = switch to clicked location
    // double click = rename selected location
    function handleLocationClick(self, loc_n) {
	clicks++;
	if (clicks === 1) {
	    timer = setTimeout(function() {
		current_location = self.innerHTML;
		viewer.scene.removeChild(goalMarker);
		goalMarkerOnScreen = false;
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
	// finish renaming if enter key was pressed or if text area deselected
	editableText.addEventListener("keyup", function() { renameEnter(editableText) });
	editableText.addEventListener("blur", function() { renameBlurred(editableText) });
    }

    // rename is key pressed was the enter key
    function renameEnter(self) {
	if (window.event.keyCode == 13) {  // 13 = enter key
	    renameBlurred(self);
	}
    }

    // rename the selected location to the text in the text area
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
   

    var dotColor = createjs.Graphics.getRGB(50, 50, 150, 1.0);
    var dotHoverColor = createjs.Graphics.getRGB(50, 50, 150, 0.7);

    // creates a dot marker with the given position and name on the given stage
    function createDot(stage, position, name) {
	var dotShape = new ROS2D.NavigationDot({
	    size : 7,
	    color : dotColor,
	});
	dotShape.set({name:"dot"});
	dotShape.x = position.x;
	dotShape.y = position.y;
	dotShape.scaleX = 1.0 / stage.scaleX;
	dotShape.scaleY = 1.0 / stage.scaleY;
	dotShape.name = name;
	dotShape.addEventListener('mouseover', function(event) { 
	    document.body.style.cursor='pointer';
	    dotShape.setColor(dotHoverColor);
	});
	dotShape.addEventListener('mouseout', function(event) {
	    document.body.style.cursor='default';
	    dotShape.setColor(dotColor);
	});
	dotShape.addEventListener('mousedown', function(event) { dotOnClick(event, name, 'down'); });
	dotShape.addEventListener('pressup', function(event) { dotOnClick(event, name, 'up'); });
	return dotShape;
    }

    // ARROW VARIABLES
    var rotationRingColor = createjs.Graphics.getRGB(0, 128, 255, 1.0);
    var rotationRingHoverColor = createjs.Graphics.getRGB(0, 128, 255, 0.7);
    var locationArrowColor = createjs.Graphics.getRGB(66, 200, 128, 1.0);
    var locationArrowHoverColor = createjs.Graphics.getRGB(66, 200, 128, 0.7);
    
    // creates a basic arrow shape with the given position and rotation on the given stage
    function createBasicArrow(stage, position, rotation) {
	var marker = new ROS2D.NavigationArrow2({
	    size : 20,
	    strokeSize : 1,
	    fillColor : locationArrowColor,
	    ringColor : rotationRingColor,
	});
	marker.x =  position.x;
	marker.y =  position.y;
	marker.rotation = rotation;
	marker.scaleX = 1.0 / stage.scaleX;
	marker.scaleY = 1.0 / stage.scaleY;
	return marker;
    }

    // all interactions for a location marker. color change on hover, click to change position and orientation.
    function setLocationMarkerCallbacks(marker) {
	locationArrow = marker.getArrow();
	locationArrow.addEventListener('mouseover', function(event) {
	    document.body.style.cursor='pointer';
	    locationMarker.setArrowColor(locationArrowHoverColor);
	});
	locationArrow.addEventListener('mouseout', function(event) {
	    document.body.style.cursor='default';
	    locationMarker.setArrowColor(locationArrowColor);
	});
	locationArrow.addEventListener();
        locationArrow.addEventListener('mousedown', function(event) { locationEventHandler(event, 'down'); });
        locationArrow.addEventListener('pressmove', function(event) { locationEventHandler(event, 'move'); });
        locationArrow.addEventListener('pressup', function(event) { locationEventHandler(event, 'up'); });
	rotationRing = marker.getRotationRing();
	rotationRing.addEventListener('mouseover', function(event) { 
	    document.body.style.cursor='pointer';
	    locationMarker.setRotationColor(rotationRingHoverColor); 
	});
	rotationRing.addEventListener('mouseout', function(event) { 
	    document.body.style.cursor='default';
	    locationMarker.setRotationColor(rotationRingColor); 
	});
	rotationRing.addEventListener('mousedown', function(event) { rotationEventHandler(event, 'down'); });
	rotationRing.addEventListener('pressmove', function(event) { rotationEventHandler(event, 'move'); });
	rotationRing.addEventListener('pressup', function(event) { rotationEventHandler(event, 'up'); });
    }
    
    // stores the currently selected location to the backend
    function storeCurrentLocation(currentPosition, thetaRadians) {
	var qz =  Math.sin(-thetaRadians/2.0);
        var qw =  Math.cos(-thetaRadians/2.0);

        var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});

        var pose = new ROSLIB.Pose({
            position :    currentPosition,
            orientation : orientation
        });
	setLocation(pose);
    }

    var dotDown = false;
    function dotOnClick(event, name, mouseState) {
	event.stopPropagation(); // don't want this to propagate down to the stage event listener
	if (mouseState === 'down') {
	    dotDown = true;
	} else if (dotDown) {
	    dotDown = false;
	    viewer.scene.removeChild(goalMarker);	    
	    goalMarkerOnScreen = false;
	    navPub.publish(new ROSLIB.Message({
		command: "switch-to-location",
		param: name
	    }));
	}
    }

    var position = null;
    var locationMarker = null;
    var rotating = false;
    // CLICK AND DRAG
    var locationEventHandler = function(event, mouseState) {
	event.stopPropagation();
	var stage = viewer.scene;
	position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);
	if (mouseState === 'down') {
	    // nothing to do here
	} else if (mouseState === 'move') {
	    var rotation = locationMarker.rotation;
	    stage.removeChild(locationMarker);
	    positionVec3.y *= -1;
	    locationMarker = createBasicArrow(stage, positionVec3, rotation);
	    // don't need to set callbacks because user is dragging
	    stage.addChild(locationMarker);
	} else { // press up
	    setLocationMarkerCallbacks(locationMarker);
	    // store location
	    storeCurrentLocation(positionVec3, locationMarker.rotation * (Math.PI / 180.0));
	}
    }

    // CLICK AND ROTATE
    var xDelta = 0;
    var yDelta = 0;
    var positionVec3 = null; // reference point for mouse movement
    var markerPositionVec3 = null; // center of rotation
    var thetaRadians = 0;
    var thetaDegrees = 0;
    var rotationEventHandler = function(event, mouseState) {
	event.stopPropagation();
	var stage = viewer.scene;
	if (mouseState === 'down') {
	    var position = stage.globalToRos(event.stageX, event.stageY);
            positionVec3 = new ROSLIB.Vector3(position);
	    markerPositionVec3 = new ROSLIB.Vector3({ x: locationMarker.x, y: locationMarker.y });
	    rotating = true;
	} else if (mouseState === 'move') {
	    stage.removeChild(locationMarker);
            var currentPos = stage.globalToRos(event.stageX, event.stageY);
	    var currentPosVec3 = new ROSLIB.Vector3(currentPos);
            var xDelta = currentPosVec3.x - positionVec3.x;
            var yDelta = currentPosVec3.y - positionVec3.y;
            thetaRadians  = Math.atan2(xDelta,yDelta);
            thetaDegrees = thetaRadians * (180.0 / Math.PI);
            if (thetaDegrees >= 0 && thetaDegrees <= 180) {
		thetaDegrees += 270;
            } else {
		thetaDegrees -= 90;
            }
            locationMarker = createBasicArrow(stage, markerPositionVec3, thetaDegrees);
	    // we don't need to set callbacks here because the user is dragging
            stage.addChild(locationMarker);
	} else { // press up
	    rotating = false;
	    setLocationMarkerCallbacks(locationMarker);
	    markerPositionVec3.y *= -1;
	    storeCurrentLocation(markerPositionVec3, locationMarker.rotation * (Math.PI / 180.0));
	}
    }

    // Initial Location on startup
    var locListCont = document.querySelector("#locationList");
    var dotMarkers = [];
    viewer.scene.enableMouseOver();
    // Code for drawing the list of locations.
    var drawState = function(state) {
	//draw location list
	locListCont.innerHTML = "";
	dotMarkers.forEach(function(dotMarker) {
	    viewer.scene.removeChild(dotMarker);
	});
	dotMarkers = [];
	state.locations.forEach(function(location) { 
	    var dv = document.createElement("div");
	    var loc_n = location.name;
	    dv.innerHTML = loc_n;
	    dv.addEventListener("click", function() {
		handleLocationClick(this, loc_n);
	    });
	    var li = document.createElement("li");
	    li.appendChild(dv);
	    locListCont.appendChild(li);
	    // draw the dots
	    var position = new ROSLIB.Vector3({
		x: location.pose.position.x,
		y: -location.pose.position.y });
	    var marker = createDot(viewer.scene, position, loc_n);
	    dotMarkers.push(marker);
	    viewer.scene.addChild(marker);
	});
	viewer.scene.removeChild(locationMarker);
	// If there were no saved locations, say so:
	if (state.locations.length == 0) {
	    locListCont.innerHTML = "(none)"
	}
	if (state.current_location_index != -1) {
	    // If a location is selected, enable and create buttons for its manipulation.
	    document.querySelector("#deleteBtn").removeAttribute("disabled");
	    document.querySelector("#navigateBtn").removeAttribute("disabled");
	    var current_dv = locListCont.querySelectorAll("div")[state.current_location_index];
	    current_dv.className = "selected";
	    current_location = current_dv.innerHTML;
	    // hide the dot if that location is selected
	    dotMarkers[state.current_location_index].visible = false;
	    // add arrow to map
	    var current_pose = state.locations[state.current_location_index].pose;
	    var position = new ROSLIB.Vector3({
		x: current_pose.position.x, 
		y: -current_pose.position.y });
	    var rotation = viewer.scene.rosQuaternionToGlobalTheta(current_pose.orientation);
	    locationMarker = createBasicArrow(viewer.scene, position, rotation);
	    setLocationMarkerCallbacks(locationMarker);
	    viewer.scene.addChild(locationMarker);
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