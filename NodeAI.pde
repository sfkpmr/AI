/*

 Inlämningsuppgift 2 för AI - VT22
 
 Grupp 5
 Simon Eklundh
 Max Nyström
 Marcus Wallén
 
 */

class NodeAI extends Node {
  double pathCost, fCost;
  PVector path;
  boolean valid, visited, pathVisited;

  NodeAI(PVector position) {
    super(position.x, position.y);
    this.path = null;
    this.valid = false;
    this.visited = false;
  }

  PVector[] adjacentNodeVectors() {

    PVector left = grid.getNearestNode(new PVector(-50, 0, 0).add(this.position)).position;
    PVector up = grid.getNearestNode(new PVector(0, -50, 0).add(this.position)).position;
    PVector right = grid.getNearestNode(new PVector(50, 0, 0).add(this.position)).position;
    PVector down = grid.getNearestNode(new PVector(0, 50, 0).add(this.position)).position;

    return new PVector[]{left, up, right, down};
  }

  void setFCost(PVector dest) {
    this.fCost = pathCost + hCost(dest);
  }

  double hCost(PVector dest) {
    return PVector.dist(this.position, dest);
  }
}
