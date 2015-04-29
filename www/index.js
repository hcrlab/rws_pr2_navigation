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

    processPose = null;
    // By default use map as navigation tool: processPose == NAV2D.Navigator.sendGoal.
    // If other controls are checked, change the function.
    if (document.querySelector("#setInitRadioBtn").checked) {
		processPose = setInitialPose;
	} else if (document.querySelector("#setLocationRadioBtn").checked) {
		processPose = setLocation;
	}

    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      withOrientation : true,
      serverName : '/pr2_move_base',
      processPose : processPose
    });

    // Setup the controls for the map.
    document.querySelector("#setGoalRadioBtn").addEventListener("click", function() {
		NAV2D.Navigator.processPose = NAV2D.Navigator.sendGoal;
	});
    document.querySelector("#setInitRadioBtn").addEventListener("click", function() {
		NAV2D.Navigator.processPose = setInitialPose;
	});
    document.querySelector("#setLocationRadioBtn").addEventListener("click", function() {
		NAV2D.Navigator.processPose = setLocation;
	});


	//hook up buttons with com attribute to navigation commands
	[].slice.call(document.querySelectorAll("button[com]")).forEach(function(el) {
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
    
    function handleLocationClick(self, loc_n) {
	clicks++;
	if (clicks === 1) {
	    timer = setTimeout(function() {
		navPub.publish(new ROSLIB.Message({
		    command: "switch-to-location",
		    param: loc_n
		}));
		clicks = 0;
	    }, 500);
	} else {
	    clearTimeout(timer);    //prevent single-click action
	    rename(self);  //perform double-click action
	    clicks = 0;
	}
    }    

    function rename(self) {
	var editableText = document.createElement("textarea");
	editableText.value = self.innerHTML;
	self.parentNode.replaceChild(editableText, self);
	editableText.focus();
	editableText.addEventListener("keyup", function() { renameEnter(editableText) });
	editableText.addEventListener("blur", function() { renameBlurred(editableText) });
    }
    
    function renameEnter(self) {
	if (window.event.keyCode == 13) {
	    renameBlurred(self);
	}
    }

    function renameBlurred(self) {
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
    });
    document.querySelector("#plugwarningclose").addEventListener("click", function() {
	plug_modal.close();
    });

    var init_modal = document.querySelector("#initwarningdialog");
    document.querySelector("#initwarning>img").addEventListener("click", function() {
	init_modal.showModal();
    });
    document.querySelector("#initwarningclose").addEventListener("click", function() {
	init_modal.close();
    });

    var localized_modal = document.querySelector("#localizedwarningdialog");
    document.querySelector("#localizedwarning>img").addEventListener("click", function() {
	localized_modal.showModal();
    });
    document.querySelector("#localizedwarningclose").addEventListener("click", function() {
	localized_modal.close();
    });
   
    var overlayDiv = document.querySelector("#overlay");
    var newNameInp = document.querySelector("#newName");

    document.querySelector("#renameBtn").addEventListener("click", function() {
	overlayDiv.style.display = "";
    });
    document.querySelector("#doRename").addEventListener("click", function() {
	navPub.publish(new ROSLIB.Message({
	    command: "name-location",
	    param: newNameInp.value
	}));
	overlayDiv.style.display = "none";
    });
    document.querySelector("#cancelRename").addEventListener("click", function() {
	overlayDiv.style.display = "none";
    });
    
    var locListCont = document.querySelector("#locationList");

    var locationMarker = new ROS2D.NavigationArrow({
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
            document.querySelector("#renameBtn").removeAttribute("disabled");
            document.querySelector("#recordBtn").removeAttribute("disabled");
            document.querySelector("#deleteBtn").removeAttribute("disabled");
            document.querySelector("#navigateBtn").removeAttribute("disabled");

		    locListCont.querySelectorAll("div")[state.current_location].className = "selected";
		    newNameInp.value = state.location_names[state.current_location];

            // update the location on the map
            locationMarker.x = state.current_location_pose.position.x;
            locationMarker.y = -state.current_location_pose.position.y;
            locationMarker.scaleX = 1.0 / viewer.scene.scaleX;
            locationMarker.scaleY = 1.0 / viewer.scene.scaleY;
            // change the angle
            locationMarker.rotation = viewer.scene.rosQuaternionToGlobalTheta(state.current_location_pose.orientation);
            locationMarker.visible = true;
        } else {
            // If no location is selected, disable buttons that operate on current location.
            document.querySelector("#renameBtn").setAttribute("disabled", true);
            document.querySelector("#recordBtn").setAttribute("disabled", true);
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


	//
  }