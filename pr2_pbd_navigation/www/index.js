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

    function processBatteryState(state) {
        var controlsSpan = document.querySelector('div[id="controls"]');
		controlsSpan.style.display = (state.discharging == 0) ? "none" : "inline-block";
        var plugWarningSpan = document.querySelector('div[id="plugwarning"]');
		plugWarningSpan.style.display = (state.discharging == 0) ? "inline-block" : "none";
    }


	batteryStateListener.subscribe(function(state) {
		processBatteryState(state);
	});
  }