/*

Inlämingsuppgift 1 för AI - VT22

Grupp 5
Simon Eklundh
Max Nyström
Marcus Wallén

*/
import game2dai.entities.*;
import game2dai.maths.*;

class Sprite extends MovingEntity {
 
  PVector position;
  String name;
  float diameter, radius;
  Sprite(){
    super(new Vector2D(), 2.0, new Vector2D(0, 0), 1.0, new Vector2D(0, 0), 2.0, 1.0, 1.0);
  }
  
  //**************************************************
  public String getName(){
    return this.name;
  }
  
  //**************************************************
  public float diameter(){
    return this.diameter;
  }  
  
  //**************************************************
  public float getRadius(){
    return this.radius;
  }   
  
  //**************************************************
  public PVector position(){
    return this.position;
  }
  
}
