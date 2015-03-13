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
			command: "name-location ",
			param: newNameInp.value
		}));
		overlayDiv.style.display = "none";
	});
	document.querySelector("#cancelRename").addEventListener("click", function() {
		overlayDiv.style.display = "none";
	});
  }