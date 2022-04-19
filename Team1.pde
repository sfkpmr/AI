/*

Inlämingsuppgift 1 för AI - VT22

Grupp 5
Simon Eklundh
Max Nyström
Marcus Wallén

*/

import java.util.*;

class Team1 extends Team {

  Team1(int team_id, int tank_size, color c,
    PVector tank0_startpos, int tank0_id, CannonBall ball0,
    PVector tank1_startpos, int tank1_id, CannonBall ball1,
    PVector tank2_startpos, int tank2_id, CannonBall ball2) {
    super(team_id, tank_size, c, tank0_startpos, tank0_id, ball0, tank1_startpos, tank1_id, ball1, tank2_startpos, tank2_id, ball2);

    tanks[0] = new Tank(tank0_id, this, this.tank0_startpos, this.tank_size, ball0);
    tanks[1] = new Tank(tank1_id, this, this.tank1_startpos, this.tank_size, ball1);
    tanks[2] = new Tank3(tank2_id, this, this.tank2_startpos, this.tank_size, ball2);

    //this.homebase_x = 0;
    //this.homebase_y = 0;
  }


  //==================================================
  public class Tank1 extends Tank {

    boolean started;
    Sensor locator;
    Sensor us_front; //ultrasonic_sensor front

    Tank1(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      us_front = getSensor("ULTRASONIC_FRONT");
      addSensor(us_front);

      started = false;
    }

    public void initialize() {
    }

    // Tanken meddelas om kollision med tree.
    public void message_collision(Tree other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");

      chooseAction();
    }

    // Tanken meddelas om kollision med tanken.
    public void message_collision(Tank other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tank)");

      chooseAction();
    }

    public void arrived() {
      super.arrived(); // Tank
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrived()");

      chooseAction();
    }

    public void arrivedRotation() {
      super.arrivedRotation();

      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrivedRotation()");
      //moveTo(new PVector(int(random(width)),int(random(height))));
      //moveTo(grid.getRandomNodePosition()); // Slumpmässigt mål.
      moveForward_state(); // Tank
    }

    public void chooseAction() {
      //moveTo(grid.getRandomNodePosition());
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].chooseAction()");
      //resetTargetStates(); // Tank
      //resetAllMovingStates(); // Tank

      float r = random(1, 360);
      rotateTo(radians(r));
    }

    public void readSensorDistance() {
      SensorReading sr = readSensor_distance(us_front);
      //println("1sr.distance(): "+ sr.distance());
      if ((sr.distance() < this.radius) && this.isMoving) {
        if (!this.stop_state) {
          println("Team"+this.team_id+".Tank["+ this.getId() + "] Har registrerat ett hinder. (Tank.readSensorDistance())");
          //stopMoving();
          //stopTurning_state()
          //this.stop_state = true;
          stopMoving_state(); //Tank
          //chooseAction();
        }
      }
    }

    public void updateLogic() {
      //super.updateLogic();


      // Avoid contact with other objects and tanks.
      float threshold = .1f;
      //println("========================================");
      //println("Team"+this.team_id+".Tank["+ this.getId() + "] : " + us_front.readValue(0));
      //if (us_front.readValue(0) < threshold) {
      //  println("*** Team"+this.team_id+".Tank["+ this.getId() + "]: (us_front.readValue(0) < threshold)");
      //}

      // println("Team"+this.team_id+".Tank["+ this.getId() + "] : " + us_front.readValue1());



      if (!started) {
        started = true;
        initialize();

        moveForward_state();
        //moveForward();
      }

      if (!this.userControlled) {
        readSensorDistance();

        //moveForward_state();
        if (this.idle_state) {
          //rotateTo()
          chooseAction();
        }
      }
    }
  }

  //==================================================
  public class Tank2 extends Tank {

    boolean started;

    //*******************************************************
    Tank2(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      this.started = false;

      //this.isMoving = true;
      //moveTo(grid.getRandomNodePosition());
    }

    //*******************************************************
    // Reterera, fly!
    public void retreat() {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].retreat()");
      moveTo(grid.getRandomNodePosition()); // Slumpmässigt mål.
    }

    //*******************************************************
    // Reterera i motsatt riktning (ej implementerad!)
    public void retreat(Tank other) {
      //println("*** Team"+this.team_id+".Tank["+ this.getId() + "].retreat()");
      //moveTo(grid.getRandomNodePosition());
      retreat();
    }

    //*******************************************************
    // Fortsätt att vandra runt.
    public void wander() {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].wander()");
      //rotateTo(grid.getRandomNodePosition());  // Rotera mot ett slumpmässigt mål.
      moveTo(grid.getRandomNodePosition()); // Slumpmässigt mål.
    }


    //*******************************************************
    // Tanken meddelas om kollision med trädet.
    public void message_collision(Tree other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");
      wander();
    }

    //*******************************************************
    // Tanken meddelas om kollision med tanken.
    public void message_collision(Tank other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tank)");

      //moveTo(new PVector(int(random(width)),int(random(height))));
      //println("this.getName());" + this.getName()+ ", this.team_id: "+ this.team_id);
      //println("other.getName());" + other.getName()+ ", other.team_id: "+ other.team_id);

      if ((other.getName() == "tank") && (other.team_id != this.team_id)) {
        if (this.hasShot && (!other.isDestroyed)) {
          println("["+this.team_id+":"+ this.getId() + "] SKJUTER PÅ ["+ other.team_id +":"+other.getId()+"]");
          fire();
        } else {
          retreat(other);
        }

        rotateTo(other.position);
        //wander();
      } else {
        wander();
      }
    }

    //*******************************************************
    // Tanken meddelas om den har kommit hem.
    public void message_arrivedAtHomebase() {
      //println("*** Team"+this.team_id+".Tank["+ this.getId() + "].message_isAtHomebase()");
      println("! Hemma!!! Team"+this.team_id+".Tank["+ this.getId() + "]");
    }

    //*******************************************************
    // används inte.
    public void readyAfterHit() {
      super.readyAfterHit();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].readyAfterHit()");

      //moveTo(grid.getRandomNodePosition());
      wander();
    }

    //*******************************************************
    public void arrivedRotation() {
      super.arrivedRotation();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrivedRotation()");
      //moveTo(new PVector(int(random(width)),int(random(height))));
      arrived();
    }

    //*******************************************************
    public void arrived() {
      super.arrived();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrived()");

      //moveTo(new PVector(int(random(width)),int(random(height))));
      //moveTo(grid.getRandomNodePosition());
      wander();
    }

    //*******************************************************
    public void updateLogic() {
      super.updateLogic();

      if (!started) {
        started = true;
        moveTo(grid.getRandomNodePosition());
      }

      if (!this.userControlled) {

        //moveForward_state();
        if (this.stop_state) {
          //rotateTo()
          wander();
        }

        if (this.idle_state) {
          wander();
        }
      }
    }
  }

  //==================================================
  public class Tank3 extends Tank {
    Stack<PVector> desiredPath;
    boolean started, first, bumpedIntoTree = false;
    PVector tempTarget = null, oldPosition = positionPrev;

    HashMap<PVector, NodeAI> graph = new HashMap<PVector, NodeAI>();

    Tank3(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      /*
      Code for testing (gives the tank all nodes in memory in advance)

      for (Node[] nn : grid.nodes) {
       for (Node n : nn) {
       NodeAI nai = new NodeAI(n.position);
       // nai.valid = n.position.equals(grid.getNearestNode(new PVector(200, 550, 0)).position) ? false : true;
       graph.put(n.position, nai);
       }
       }
       */
      this.started = false;
    }

    @Override
      public HashMap<PVector, NodeAI> getMap() {
      return graph;
    }

    // Tanken meddelas om kollision med trädet.
    public void message_collision(Tree other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");
      bumpedIntoTree = true;
    }

    public void arrived() {
      PVector tempTarget = targetPosition;
      super.arrived();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrived()");
      graph.get(grid.getNearestNode(position).position).valid = true;
      graph.get(grid.getNearestNode(position).position).visited = true;
      oldPosition = grid.getNearestNode(position).position;
    }

    boolean canMoveToDirection(String dir) {
      return !graph.containsKey(goDirection(dir));
    }

    PVector goDirection(String dir) {
      PVector temp = new PVector(0, 0, 0);

      switch (dir) {
      case "down":
        temp = grid.getNearestNode(temp.add(position).add(0, 50, 0)).position;
        break;
      case "left":
        temp = grid.getNearestNode(temp.add(position).add(-50, 0, 0)).position;
        break;
      case "up":
        temp = grid.getNearestNode(temp.add(position).add(0, -50, 0)).position;
        break;
      case "right":
        temp = grid.getNearestNode(temp.add(position).add(50, 0, 0)).position;
        break;
      default:
        temp = grid.getNearestNode(temp.add(position)).position;
      }
      return temp;
    }

    /*
     Our strategy for searching the map is divided into two parts. If there are adjacent nodes that have not been visisted before,
     then we try to move to one such node in the following order: right, up, left, down. However,
     if all adjacent nodes have already been visited, then we try to move to the node to the down, left, up or right, in that order.
     */
    void moveOneStep() {

      if (canMoveToDirection("right")) {
        saveAndWalk(goDirection("right"), 50, 0);
      } else if (canMoveToDirection("up")) {
        saveAndWalk(goDirection("up"), 0, -50);
      } else if (canMoveToDirection("left")) {
        saveAndWalk(goDirection("left"), -50, 0);
      } else if (canMoveToDirection("down")) {
        saveAndWalk(goDirection("down"), 0, 50);
      } else {
        println("Knows everything");
        if (canMove(goDirection("down"))) {
          moveBy(new PVector(0, 50, 0));
        } else if (canMove(goDirection("left"))) {
          moveBy(new PVector(-50, 0, 0));
        } else if (canMove(goDirection("up"))) {
          moveBy(new PVector(0, -50, 0));
        } else if (canMove(goDirection("right"))) {
          moveBy(new PVector(50, 0, 0));
        } else {
          println("Error in pathfinding for primary search.");
        }
      }
    }

    private boolean canMove(PVector direction) {
      NodeAI temp = graph.get(direction);

      return (temp.valid && !temp.position.equals(graph.get(goDirection("")).position));
    }

    private void saveAndWalk(PVector walkTo, int i, int i1) {
      graph.put(walkTo, new NodeAI(walkTo));
      moveBy(new PVector(i, i1, 0));
    }

    /*
     The default objective of the agent is to efficiently search the world, with a couple of exceptions;
     when encountering a tree it should return to the previous position and reevaluate,
     when encountering an enemy it should return to base and report for 3 seconds.
     */
    public void updateLogic() {

      if (!started) {
        started = true;
        NodeAI startingNode = new NodeAI(grid.getNearestNodePosition(startpos));
        startingNode.valid = true;
        graph.put(startpos, startingNode);
      }

      if (!this.userControlled) {

        if (this.isRetreating) {

          pathFinding();

          if (isAtHomebase && !isReporting) {
            waitUntil = millis() + 3000;
            println("Reporting! Current world time: " + millis() + " waitUntil: " + waitUntil + " " + remainingTime);
            stopMoving();
            isRetreating = false;
            isReporting = true;
          }
        }

        if (millis() >= waitUntil && isReporting ) {
          println("Done reporting at: "+millis() + " " + remainingTime);
          isReporting = false;
          stop_state = true;
          this.desiredPath = null;
        }

        if (this.stop_state && !isRetreating && !isReporting && !bumpedIntoTree) {
          println("moving");
          moveOneStep();
        } else if (bumpedIntoTree && !isRetreating && !isReporting) {

          NodeAI tmp = graph.get(grid.getNearestNode(position).position);
          tmp.valid = false;

          moveTo(oldPosition);
          println("försöker backa från trädet");
          bumpedIntoTree = false;
        }
      }
    }

    //The tank generates an efficient path home when it begins its retreat, and then moves one node every time it is not moving.
    void pathFinding() {

      if (this.desiredPath == null) {
        this.desiredPath = dijkstras(grid.getNearestNodePosition(this.position), grid.getNearestNodePosition(this.startpos));
        println(desiredPath);
        this.desiredPath = aStar(grid.getNearestNodePosition(this.position), grid.getNearestNodePosition(this.startpos));
        println(desiredPath);
        desiredPath.pop();
      }

      if (!isMoving) {
        PVector var2 = desiredPath.pop();
        moveTo(var2);
      }
    }

    /** 

    Algorithm that finds the closest path between two Nodes given two PVectors.
    Using a graph that the tanks holds in memory, expands the cheapest routes first
    until it finds the closest path home or until no path is found. The cost to 
    traverse the graph is 1 per edge. The cheapest route is the path with the lowest cost.
    
    Based on the explanation from: 
    Mark Allen Weiss. Data Structures and Algorithm Analysis in Java. 
    Pearson Education, third edition, 2012, p.392

    @return   Stack with the nodes that the tank needs to go to in order to get to the home base.
    */ 
    public Stack<PVector> dijkstras(PVector start, PVector dest) {
      // Dijkstra's uses a priority queue for getting the next frontier node with 
      // the lowest cost that will be explored next
      PriorityQueue<NodeAI> q = new PriorityQueue<NodeAI>(new Comparator() {
        @Override
          public int compare(Object a, Object b) {
          NodeAI anode = (NodeAI) a;
          NodeAI bnode = (NodeAI) b;

          return Double.compare(anode.pathCost, bnode.pathCost);
        }
      }
      );

      // Reset costs and path for all nodes
      for (NodeAI n : graph.values()) {
        n.path = null;
        n.pathVisited = false;
        n.pathCost = Double.POSITIVE_INFINITY;
      }

      // Add the node we are standing on as the starting node in the graph
      NodeAI first = graph.get(start);
      first.pathCost = 0;

      // Initialize the algorithm by seting the start node first in the queue
      q.add(first);
      NodeAI goal = graph.get(dest);
      // unnecessery int for dijkstra's algo, but is used to get a print out of the nr of nodes the algorithm explored
      int exploredNodes = 0;


      while (!q.isEmpty() && !goal.pathVisited) {
        exploredNodes++;
        
        NodeAI v = q.remove();
        v.pathVisited = true;

        for (PVector pVec : v.adjacentNodeVectors()) {

          NodeAI next = graph.get(pVec);
          // if the path doesn't exist in memory or is invalid (i.e. cannot be walked on) then ignore this iteration
          if (next == null || !next.valid) continue;

          if (next.pathVisited == false) {
            double cost = v.pathCost + 1;

            if (cost < next.pathCost) {
              next.pathCost = cost;
              //
              next.path = v.position;
              q.add(next);
            }
          }
        }
      }
      String ex = String.format("Dijkstra explored %d numder of nodes", exploredNodes);
      println(ex);
      return getPath(dest);
    }

    Stack<PVector> getPath(PVector destination) {
      Stack<PVector> path = new Stack<PVector>();
      path.push(destination);
      //   println("destination: " + destination);
      NodeAI it = graph.get(destination);
      while (it != null) {
        NodeAI tmp = graph.get(it.path);
        if (tmp == null) break;
        //    println("tmp.position: " + tmp.position);
        path.push(tmp.position);
        it = graph.get(tmp.position);
      }
      return path;
    }


/** 
    Algorithm that finds the closest path between two Nodes given two PVectors.
    Using a graph that the tanks holds in memory, expands the cheapest routes first
    until it finds the closest path home or until no path is found. 
    
    The cost to traverse the graph is 1 per edge and A* also uses a heuristic function 
    that is defined as the euclidian distance from the current node to the starting 
    position, which the tank has in memory. 
    
    Based on the explanation from: 
    Mark Allen Weiss. Data Structures and Algorithm Analysis in Java. 
    Pearson Education, third edition, 2012, p. 399.

    @return   Stack with the nodes that the tank needs to go to in order to get to the home base.
    */ 
    public Stack<PVector> aStar(PVector start, PVector dest) {
      // A* uses a priority queue for getting the next frontier node with the lowest cost that will be explored next.
      // PriorityQueue<NodeAI> q = new PriorityQueue<NodeAI>(Comparator.comparingDouble((NodeAI a) -> a.fCost));
      PriorityQueue<NodeAI> q = new PriorityQueue<NodeAI>(new Comparator()
      {
        @Override
          public int compare(Object a, Object b) {
          NodeAI anode = (NodeAI) a;
          NodeAI bnode = (NodeAI) b;
          return Double.compare(anode.fCost, bnode.fCost);
        }
      }
      );
      // Reset costs and path for all nodes
      for (NodeAI n : graph.values()) {
        n.pathVisited = false;
        n.fCost = Double.POSITIVE_INFINITY;
        n.pathCost = Double.POSITIVE_INFINITY;
        n.path = null;
      }

      // Initialize the algorithm by seting the start node first in the queue
      NodeAI first = graph.get(start);
      first.pathCost = 0;
      first.setFCost(dest);
      q.add(first);


      NodeAI goal = graph.get(dest);
      // unnecessery int for A*, but is used to get a print out of the nr of nodes the algorithm explored
      int exploredNodes = 0;
      while (!q.isEmpty() && !goal.pathVisited) {
        NodeAI expanding = q.remove();
        exploredNodes++;
        expanding.pathVisited = true;

        for (PVector pVec : expanding.adjacentNodeVectors()) {

          NodeAI next = graph.get(pVec);
          // if the path doesn't exist in memory or is invalid (i.e. cannot be walked on) then ignore this iteration
          if (next == null || !next.valid) continue;

          if (next.pathVisited == false) {
            double cost = expanding.pathCost + 1;

            if (cost < next.pathCost) {
              next.pathCost = cost;
              next.setFCost(dest);
              
              next.path = expanding.position;
              q.add(next);
            }
          }
        }
      }
      String ex = String.format("A* explored %d numder of nodes", exploredNodes);
      println(ex);
      return getPath(dest);
    }
  }
}
