import * as rclnodejs from "rclnodejs";

const NODE_NAME: string = "r2ps_behavior_manager";

export default class R2PSBehaviorManagerNode extends rclnodejs.Node {

    constructor() {
        super(NODE_NAME);
        this.getLogger().info(`${this.name()} created`);
    }
}