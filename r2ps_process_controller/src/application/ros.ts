import * as rclnodejs from "rclnodejs";
import { Node, Timer, Publisher, Options, QoS, r2ps_msgs } from "rclnodejs";

export default class ROSService {

    private _node: Node;
    private currentNodesCheckTimer: Timer;
    private currentNodeListPublisher: Publisher<"r2ps_msgs/msg/NodeList">;

    constructor(node: Node) {
        this._node = node;
        this.bindFunctions();

        this.currentNodesCheckTimer = this._node.createTimer(
            500,
            this.currentNodesCheckTimerCallback.bind(this)
        );

        const currentNodeListPublisherOpts: Options = { qos: QoS.profileSystemDefault };
        this.currentNodeListPublisher = this._node.createPublisher(
            "r2ps_msgs/msg/NodeList",
            "/r2ps/r2/node/list",
            currentNodeListPublisherOpts
        );
    }

    private bindFunctions(): void {
        this.currentNodesCheckTimerCallback = this.currentNodesCheckTimerCallback.bind(this);
        this.getAndPublishCurrentNodeList = this.getAndPublishCurrentNodeList.bind(this);
    }

    private currentNodesCheckTimerCallback(): void {
        this.getAndPublishCurrentNodeList();
    }

    private getAndPublishCurrentNodeList(): void {
        try {
            const currentNodeList: r2ps_msgs.msg.NodeList = rclnodejs.createMessageObject("r2ps_msgs/msg/NodeList");

            let currentNodes: string[] = this._node.getNodeNames();

            currentNodes = currentNodes.filter(currentNode => {
                if (currentNode === this._node.name() || currentNode.includes(this._node.name()) || currentNode.includes("r2ps")) {
                    return false;
                }

                if (currentNode.includes("_ros2") || currentNode.includes("daemon")) {
                    return false;
                }

                return true;
            });

            currentNodeList.node_list = currentNodes;
            this.currentNodeListPublisher.publish(currentNodeList);
        } catch (e: any) {
            this._node.getLogger().error(`${e}`);
        }
    }
}