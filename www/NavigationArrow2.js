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
 *   * fillColor (optional) - the createjs color for the fill
 */
ROS2D.NavigationArrow2 = function(options) {
    var that = this;
    options = options || {};
    var size = options.size || 10;
    var strokeSize = options.strokeSize || 3;
    var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
    var fillColor = options.fillColor || createjs.Graphics.getRGB(255, 0, 0);
    
    var rotationRingGraphics = new createjs.Graphics();
    rotationRingGraphics.beginStroke('#000');
    rotationRingGraphics.setStrokeStyle(8);
    rotationRingGraphics.arc(0, 0, 20, 0, Math.PI*2);
    // create the rotation ring shape
    this.rotationRingShape = new createjs.Shape(rotationRingGraphics).set({name:"rotation"});
    // draw the arrow
    var arrowGraphics = new createjs.Graphics();
    // line width
    arrowGraphics.setStrokeStyle(strokeSize);
    arrowGraphics.moveTo(-size / 2.0, -size / 2.0);
    arrowGraphics.beginStroke(strokeColor);
    arrowGraphics.beginFill(fillColor);
    arrowGraphics.lineTo(size, 0);
    arrowGraphics.lineTo(-size / 2.0, size / 2.0);
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
ROS2D.NavigationArrow2.prototype.__proto__ = createjs.Container.prototype;
