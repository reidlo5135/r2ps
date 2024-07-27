import * as rclnodejs from "rclnodejs";

const NODE_NAME: string = "r2ps_behavior_controller";

export default class R2PSBehaviorControllerNode extends rclnodejs.Node {

    constructor() {
        super(NODE_NAME);
        this.getLogger().info(`${this.name()} created`);
    }
}