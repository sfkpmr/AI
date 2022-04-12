class NodeAI extends Node {

  NodeAI right, left, up, down;
  boolean valid;

  NodeAI(PVector position) {

    super(position.x, position.y);
    this.right = null;
    this.left = null;
    this.up = null;
    this.down = null;
    this.valid = false;
  }
}
