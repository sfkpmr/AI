/*

 Inlämningsuppgift 2 för AI - VT22
 
 Grupp 5
 Simon Eklundh
 Max Nyström
 Marcus Wallén
 
 */

class Turret {
  PImage img;
  float rotation_speed;
  float cannon_length;

  PVector position;
  //PVector velocity;
  //PVector acceleration;
  // Variable for heading!
  float heading;


  Turret(float cannon_length) {
    //this.img = loadImage("gunTurret2.png");
    this.position = new PVector(0.0, 0.0);

    this.cannon_length = cannon_length;
    this.heading = 0.0;
    this.rotation_speed = radians(1);
  }

  void turnLeft() {
    this.heading -= this.rotation_speed;
  }

  void turnRight() {
    this.heading += this.rotation_speed;
  }

  void drawTurret() {
    strokeWeight(1);
    //fill(204, 50, 50);
    ellipse(0, 0, 25, 25);
    strokeWeight(3);
    line(0, 0, this.cannon_length, 0);
  }



  void fire() {
  }

  /**
   * This method determines whether this entity can see a particular location in the world. <br>
   * It first checks to see if it is within this entity's view distance and field of view (FOV).
   * If it is then it checks to see if there are any walls or obstacles between them.
   *
   * @param world the world responsible for this entity
   * @param x0 the x position of the location to test
   * @param y0 the y position of the location to test
   * @return true if the entity can see the location
   
   public boolean canSee(World world, double x0, double y0){
   Vector2D toTarget = new Vector2D(x0 - pos.x, y0 - pos.y);
   // See if in view range
   double distToTarget = toTarget.length();
   if(distToTarget > viewDistance)
   return false;
   // See if in field of view
   toTarget.div(distToTarget);  // normalise toTarget
   double cosAngle = heading.dot(toTarget);
   if(cosAngle <  FastMath.cos(viewFOV / 2))
   return false;
   // If we get here then the position is within range and field of view, but do we have an obstruction.
   // First check for an intervening wall
   Set<Wall> walls = world.getWalls(this, x0, y0);
   if(walls != null && !walls.isEmpty()){
   for(Wall wall : walls){
   if(wall.isEitherSide(pos.x, pos.y, x0, y0))
   return false;
   }
   }
   // Next check for an intervening obstacle
   Set<Obstacle> obstacles = world.getObstacles(this, x0, y0);
   if(obstacles != null && !obstacles.isEmpty()){
   for(Obstacle obstacle : obstacles){
   if(obstacle.isEitherSide(pos.x, pos.y, x0, y0))
   return false;
   }
   }
   return true;
   }
   */

  void display() {
    this.position.x = cos(this.heading);
    this.position.y = sin(this.heading);

    rotate(this.heading);
    //image(img, 20, 0);
    drawTurret();
  }
}
