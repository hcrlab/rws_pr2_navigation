var ros = new ROSLIB.Ros({
	url : 'ws://' + window.location.hostname + ':9090'
  });


var navPub = new ROSLIB.Topic({
	ros : ros,
	name : '/navigation_command',
	messageType : 'pr2_pbd_navigation/NavigationCommand'
});


var batteryStateListener = new ROSLIB.Topic({
	ros : ros,
	name : '/battery/server',
	messageType : 'pr2_msgs/BatteryServer'
});

var expListener = new ROSLIB.Topic({
	ros : ros,
	name : '/nav_system_state',
	messageType : 'pr2_pbd_navigation/NavSystemState'
});

var expListenerSrvCli = new ROSLIB.Service({
	ros : ros,
	name : '/get_nav_system_state',
	serviceType : 'pr2_pbd_interaction/GetNavSystemState'
});

function init() {

    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'nav',
      width : 750,
      height : 800
    });

    // Setup the nav client.
    var nav = NAV2D.OccupancyGridClientNav({
      ros : ros,
      rootObject : viewer.scene,
      viewer : viewer,
      withOrientation : true,
      serverName : '/pr2_move_base'
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
        var controlsSpan = document.querySelector('div[id="controls"]');
//		controlsSpan.style.display = turnOn ? "inline-block" : "none";
        if (turnOn) {
            document.querySelector("#navigateBtn").removeAttribute("disabled");
            document.querySelector("#spinBtn").removeAttribute("disabled");
        } else {
            document.querySelector("#navigateBtn").setAttribute("disabled", true);
            document.querySelector("#spinBtn").setAttribute("disabled", true);
        }
    }

    function processBatteryState(state) {
        // If robot is plugged in, display warning and turn off navigation controls.
        var plugWarningSpan = document.querySelector('div[id="plugwarning"]');
		if (state.discharging == 0) {
		    toggleControls(false);
		    plugWarningSpan.style.display = "inline-block";
		} else {
		    toggleControls(true);
		    plugWarningSpan.style.display = "none";
		}
    }


	batteryStateListener.subscribe(processBatteryState);


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
	var curSpan = document.querySelector("#curLocation");

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
				navPub.publish(new ROSLIB.Message({
					command: "switch-to-location",
					param: loc_n
				}));
			});
			locListCont.appendChild(dv);
		});
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

	expListener.subscribe(function(state) {
		drawState(state);
	});

	expListenerSrvCli.callService(new ROSLIB.ServiceRequest({}), function(result) {
		drawState(result.state);
	});
  }