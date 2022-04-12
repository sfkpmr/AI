class NodeAI extends Node {

  PVector right, left, up, down;

  NodeAI(PVector position, PVector right, PVector left, PVector up, PVector down) {

    super(position.x, position.y);

    this.right = null;

    this.left = null;

    this.up = null;

    this.down = null;
  }
}
