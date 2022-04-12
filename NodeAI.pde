class NodeAI extends Node {

  NodeAI right, left, up, down;
  boolean valid, visited;

  NodeAI(PVector position) {

    super(position.x, position.y);
    this.right = null;
    this.left = null;
    this.up = null;
    this.down = null;
    this.valid = false;
    this.visited = false;
  }
}
