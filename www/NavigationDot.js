ROS2D.NavigationDot = function(options) {
    var that = this;
    options = options || {};
    this.size = options.size || 5;
    this.color = options.color || createjs.Graphics.getRGB(255, 0, 0);

    var dotGraphics = new createjs.Graphics();
    dotGraphics.beginFill(this.color);
    dotGraphics.drawCircle(0,0,this.size);
    dotGraphics.endFill();
    createjs.Shape.call(this, dotGraphics);
};

ROS2D.NavigationDot.prototype.setColor = function(color) {
    this.graphics
        .clear()
        .beginFill(color)
        .drawCircle(0,0,this.size)
        .endFill();
};

ROS2D.NavigationDot.prototype.__proto__ = createjs.Shape.prototype;