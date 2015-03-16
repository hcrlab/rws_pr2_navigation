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

function init() {
    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'map',
      width : 600,
      height : 500
    });

    // Setup the map client.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene
    });
    // Scale the canvas to fit to the map
    gridClient.on('change', function(){
      viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
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
        var controlsSpan = document.querySelector('div[id="controls"]');
//		controlsSpan.style.display = turnOn ? "inline-block" : "none";
    }

    function processBatteryState(state) {
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

	document.querySelector("#renPopup").addEventListener("click", function() {
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

	// Code for drawing the list of locations.
	var drawState = function(state) {
		if (window.lockUpdate)
			return;
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
		locListCont.querySelectorAll("div")[state.current_location].className =
			"selected";

		newNameInp.value = state.location_names[state.current_location];

		//current location:
		curSpan.innerHTML = "";
        var delBut = document.createElement("button");
        delBut.innerHTML = "Delete this location";
        delBut.addEventListener("click", function() {
            navPub.publish(new ROSLIB.Message({
                command: "delete-current"
            }));
        });
        curSpan.appendChild(delBut);
        var goBut = document.createElement("button");
        goBut.innerHTML = "Go to this location";
        goBut.addEventListener("click", function() {
            navPub.publish(new ROSLIB.Message({
                command: "navigate-to-current"
            }));
        });
        curSpan.appendChild(goBut);
	};

	expListener.subscribe(function(state) {
		drawState(state);
	})
  }