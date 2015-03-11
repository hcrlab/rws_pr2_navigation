var ros = new ROSLIB.Ros({
	url : 'ws://' + window.location.hostname + ':9090'
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

    function toggleControls(turnOn) {
        var controlsSpan = document.querySelector('div[id="controls"]');
		controlsSpan.style.display = turnOn ? "inline-block" : "none";
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
  }