/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A navigation arrow is a directed triangle that can be used to display orientation.
 *
 * @constructor
 * @param options - object with following keys:
 *   * size (optional) - the size of the marker
 *   * strokeSize (optional) - the size of the outline
 *   * strokeColor (optional) - the createjs color for the stroke
 *   * fillColor (optional) - the createjs color for the arrow fill
 *   * ringColor (optional) - the createjs color for the ring fill
 */

ROS2D.NavigationArrow2 = function(options) {
    var that = this;
    options = options || {};
    this.size = options.size || 10;
    this.strokeSize = options.strokeSize || 3;
    this.strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
    var fillColor = options.fillColor || createjs.Graphics.getRGB(255, 0, 0);
    var ringColor = options.ringColor || createjs.Graphics.getRGB(0, 0, 255);
    
    // rotation ring
    var rotationRingGraphics = new createjs.Graphics();
    rotationRingGraphics.beginStroke(ringColor);
    rotationRingGraphics.setStrokeStyle(8);
    rotationRingGraphics.arc(0, 0, this.size + 2, 0, Math.PI*2);
    // create the rotation ring shape
    this.rotationRingShape = new createjs.Shape(rotationRingGraphics).set({name:"rotation"});
    // arrow
    var arrowGraphics = new createjs.Graphics();
    arrowGraphics.setStrokeStyle(this.strokeSize);
    arrowGraphics.moveTo(-this.size / 2.0, -this.size / 2.0);
    arrowGraphics.beginStroke(this.strokeColor);
    arrowGraphics.beginFill(fillColor);
    arrowGraphics.lineTo(this.size, 0);
    arrowGraphics.lineTo(-this.size / 2.0, this.size / 2.0);
    arrowGraphics.closePath();
    arrowGraphics.endFill();
    arrowGraphics.endStroke();
    // create the arrow shape
    this.arrowShape = new createjs.Shape(arrowGraphics).set({name:"arrow"});
    
    // Container with the arrow and rotation ring
    createjs.Container.call(this);
    this.addChild(this.rotationRingShape);
    this.addChild(this.arrowShape);
};

ROS2D.NavigationArrow2.prototype.getArrow = function() {
    return this.arrowShape;
};

ROS2D.NavigationArrow2.prototype.getRotationRing = function() {
    return this.rotationRingShape;
};

ROS2D.NavigationArrow2.prototype.setRotationColor = function(color) {
    this.rotationRingShape.graphics
	.clear()
	.beginStroke(color)
	.setStrokeStyle(8)
	.arc(0, 0, this.size + 2, 0, Math.PI*2);
};

ROS2D.NavigationArrow2.prototype.setArrowColor = function(color) {
    this.arrowShape.graphics
	.clear()
	.setStrokeStyle(this.strokeSize)
	.moveTo(-this.size / 2.0, -this.size / 2.0)
	.beginStroke(this.strokeColor)
	.beginFill(color)
	.lineTo(this.size, 0)
	.lineTo(-this.size / 2.0, this.size / 2.0)
	.closePath()
	.endFill()
	.endStroke();
};

ROS2D.NavigationArrow2.prototype.__proto__ = createjs.Container.prototype;
