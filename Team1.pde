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
    Stack path;
    boolean started;
    boolean first;
    //boolean moving20_120;
    PVector tempTarget = null;
    PVector oldPosition = positionPrev;
    boolean bumpedIntoTree = false;


    HashMap<PVector, NodeAI> graph = new HashMap<PVector, NodeAI>();

    Tank3(int id, Team team, PVector startpos, float diameter, CannonBall ball) {
      super(id, team, startpos, diameter, ball);

      for (Node[] nn : grid.nodes) {
        for (Node n : nn) {
          NodeAI nai = new NodeAI(n.position);
          nai.valid = n.position.equals(grid.getNearestNode(new PVector(200, 550, 0)).position) ? false : true;
          graph.put(n.position, nai);
        }
      }
      this.started = false;
      //this.moving20_120 = true;
    }

    @Override
      public HashMap<PVector, NodeAI> getMap() {
      return graph;
    }

    // Tanken meddelas om kollision med trädet.
    public void message_collision(Tree other) {
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].collision(Tree)");
      bumpedIntoTree = true;
      //rotateTo(grid.getRandomNodePosition());
    }

    public void arrived() {
      PVector tempTarget = targetPosition;
      super.arrived();
      println("*** Team"+this.team_id+".Tank["+ this.getId() + "].arrived()");

      graph.get(grid.getNearestNode(position).position).visited = true;
      oldPosition = grid.getNearestNode(position).position;


    }


    private Node moveOneStep() {
      

       if (graph.get(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position).valid && !graph.get(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position).position.equals(graph.get(grid.getNearestNode(position).position).position) ) {
          // addNode

          moveBy(new PVector(0, 50, 0)); // (go to node)
          println("valid bot" + millis());
          return null;

          // otherwise, find another way I guess?
        } else if (graph.get(grid.getNearestNode(new PVector(50, 0, 0).add(position)).position).valid && !graph.get(grid.getNearestNode(new PVector(50, 0, 0).add(position)).position).position.equals(graph.get(grid.getNearestNode(position).position).position) ){ // and node is valid and seen?

          moveBy(new PVector(50, 0, 0)); // (go to node)
          return null;
        }
      
      

      // right
      if (!graph.containsKey(grid.getNearestNode(new PVector(50, 0, 0).add(position)).position)) { // and node is valid and seen?
        // addNode
        graph.put(grid.getNearestNode(new PVector(50, 0, 0).add(position)).position, new NodeAI(grid.getNearestNode(new PVector(50, 0, 0).add(position)).position));
        moveBy(new PVector(50, 0, 0)); // (go to node)
        //

        // up
      } else if (!graph.containsKey(grid.getNearestNode(new PVector(0, -50, 0).add(position)).position)) {

        // addNode
        println("up");
        graph.put(grid.getNearestNode(new PVector(0, -50, 0).add(position)).position, new NodeAI(grid.getNearestNode(new PVector(0, -50, 0).add(position)).position));
        moveBy(new PVector(0, -50, 0)); // (go to node)

        //left
      } else if (!graph.containsKey(grid.getNearestNode(new PVector(-50, 0, 0).add(position)).position)) {

        // addNode
        graph.put(grid.getNearestNode(new PVector(-50, 0, 0).add(position)).position, new NodeAI(grid.getNearestNode(new PVector(-50, 0, 0).add(position)).position));
        moveBy(new PVector(-50, 0, 0)); // (go to node)


        //down
      } else if (!graph.containsKey(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position)) {
        // addNode
        graph.put(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position, new NodeAI(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position));
        moveBy(new PVector(0, 50, 0)); // (go to node)

        // otherwise, find another way I guess?
      } else {


        //down
        if (graph.get(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position).valid && !graph.get(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position).position.equals(graph.get(grid.getNearestNode(position).position).position)) {
          // addNode

          moveBy(new PVector(0, 50, 0)); // (go to node)
          println("valid bot");

          // otherwise, find another way I guess?
        }
        // left
        else if (graph.get(grid.getNearestNode(new PVector(-50, 0, 0).add(position)).position).valid && !graph.get(grid.getNearestNode(new PVector(-50, 0, 0).add(position)).position).position.equals(graph.get(grid.getNearestNode(position).position).position)) {

          // addNode
          moveBy(new PVector(-50, 0, 0)); // (go to node)


          //up
        } else if (graph.get(grid.getNearestNode(new PVector(0, -50, 0).add(position)).position).valid && !graph.get(grid.getNearestNode(new PVector(0, -50, 0).add(position)).position).position.equals(graph.get(grid.getNearestNode(position).position).position)) {
          println("valid top");

          moveBy(new PVector(0, -50, 0)); // (go to node)

          //right
        } else if (graph.get(grid.getNearestNode(new PVector(50, 0, 0).add(position)).position).valid && !graph.get(grid.getNearestNode(new PVector(0, 50, 0).add(position)).position).position.equals(graph.get(grid.getNearestNode(position).position).position)) { // and node is valid and seen?

          moveBy(new PVector(50, 0, 0)); // (go to node)
        } else {
          println("error in pathfinding for primary search");
        }
      }

      return null;
    }



    public void updateLogic() {
      super.updateLogic();

      if (!started) {
        started = true;
        NodeAI startingNode = new NodeAI(grid.getNearestNodePosition(startpos));
        graph.put(startpos, startingNode);
      }

      if (!this.userControlled) {

        if (this.isRetreating) {

          if (isAtHomebase && !isReporting) {

            stopMoving();
            isRetreating = false;
            isReporting = true;
            waitUntil = millis() + 3000;
          }
          println("retreating");

          // wayfinding here
          pathFinding();

          //return to base
        }

        if (millis() >= waitUntil) {
          isReporting = false;
          this.path = null;
        }

        if (this.stop_state && !isRetreating && !isReporting && !bumpedIntoTree) {
          println("moving");
          moveOneStep();
        } else if (bumpedIntoTree) {

          NodeAI tmp = graph.get(grid.getNearestNode(position).position);
          tmp.valid = false;

          moveTo(oldPosition);
          println("försöker backa från trädet");
          bumpedIntoTree = false;
        }
      }
      println("stop state: " + stop_state);
    }

    void pathFinding() {

      if (this.path == null) {
        println("FINDING PATH MADDATRACKKAhhH");
        this.path = dijkstras(grid.getNearestNodePosition(this.position), grid.getNearestNodePosition(this.startpos));
        println(path);
        this.path = aStar(grid.getNearestNodePosition(this.position), grid.getNearestNodePosition(this.startpos));
        println(path);
      }
    }

    public Stack<PVector> dijkstras(PVector start, PVector dest) {
      // dijkstras uses a priority queue for getiing the next frontier node with the lowest cost that will be explored next
      PriorityQueue<NodeAI> q = new PriorityQueue<NodeAI>(new Comparator() {
        @Override
          public int compare(Object a, Object b) {
          NodeAI anode = (NodeAI) a;
          NodeAI bnode = (NodeAI) b;

          return (int)(anode.pathCost - bnode.pathCost
            );
        }
      }

      );

      // initialize all nodes for the algorithm
      for (NodeAI n : graph.values()) {
        n.path = null;
        n.pathVisited = false;
        n.pathCost = Double.POSITIVE_INFINITY;
      }

      // add the first node from the graph
      NodeAI first = graph.get(start);
      first.pathCost = 0;

      q.add(first);

      NodeAI d = graph.get(dest);
      int exploredNodes = 0;
      while (!q.isEmpty() && !d.pathVisited) {
        NodeAI v = q.remove();
        exploredNodes++;
        v.pathVisited = true;

        for (PVector pVec : v.adjacentNodes()) {

          NodeAI next = graph.get(pVec);
          if (next == null) continue;

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
      String ex = String.format("dijkstra explored %d numder of nodes", exploredNodes);
      println(ex);
      return getPath(dest);
    }

    Stack<PVector> getPath(PVector destination) {
      Stack<PVector> path = new Stack<PVector>();
      path.push(destination);
      println("destination: " + destination);
      NodeAI it = graph.get(destination);
      while (it != null) {
        NodeAI tmp = graph.get(it.path);
        if (tmp == null) break;
        println("tmp.position: " + tmp.position);
        path.push(tmp.position);
        it = graph.get(tmp.position);
      }
      return path;
    }



    public Stack<PVector> aStar(PVector start, PVector dest) {
      // dijkstras uses a priority queue for getting the next frontier node with the lowest cost that will be explored next
      // PriorityQueue<NodeAI> q = new PriorityQueue<NodeAI>(Comparator.comparingDouble((NodeAI a) -> a.fCost));
      PriorityQueue<NodeAI> q = new PriorityQueue<NodeAI>(new Comparator() {
        @Override
          public int compare(Object a, Object b) {
          NodeAI anode = (NodeAI) a;
          NodeAI bnode = (NodeAI) b;

          return (int)(anode.fCost - bnode.fCost
            );
        }
      }
      );
      // initialize all nodes for the algorithm
      for (NodeAI n : graph.values()) {
        n.pathVisited = false;
        n.fCost = Double.POSITIVE_INFINITY;
        n.pathCost = Double.POSITIVE_INFINITY;
        n.path = null;
      }



      // add the first node from the graph
      NodeAI first = graph.get(start);
      first.pathCost = 0;
      first.setFCost(dest);
      q.add(first);

      NodeAI d = graph.get(dest);
      int exploredNodes = 0;
      while (!q.isEmpty() && !d.pathVisited) {
        NodeAI v = q.remove();
        exploredNodes++;
        v.pathVisited = true;

        for (PVector pVec : v.adjacentNodes()) {

          NodeAI next = graph.get(pVec);
          if (next == null) continue;

          if (next.pathVisited == false) {
            double cost = v.pathCost + 1;

            if (cost < next.pathCost) {
              next.pathCost = cost;
              next.setFCost(dest);
              //
              next.path = v.position;
              q.add(next);
            }
          }
        }
      }
      String ex = String.format("A* explored %d numder of nodes", exploredNodes);
      println(ex);
      return getPath(dest);
    }



    /*

     while (!q.isEmpty() && dest.notKnown()) {
     StopVertex v = q.remove();
     v.setKnown(true);
     
     Collection<List<StopVertex.Departure>> adj = v.adjacent.tailMap(v.arrived).values();
     for (List<StopVertex.Departure> departureList : adj) {
     for (StopVertex.Departure w : departureList) {
     if (w.destination.notKnown()) {
     double cost = w.timeBetweenStations(time);
     if (cost < w.destination.getGCost()) {
     w.destination.setG(cost);
     w.destination.arrived = w.arrival;
     w.destination.setF(hCost(w.destination, dest));
     w.destination.setPath(v, w.getTrip());
     q.add(w.destination);
     }
     }
     }
     }
     }
     
     printPath(dest);
     printAStar(time, dest);
     }
     */
  }
}
