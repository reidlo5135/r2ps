import { Node } from "rclnodejs";
import ROSService from "../application/ros";
import ProcessService from "../application/process";

const NODE_NAME: string = "r2ps_process_controller";

export default class R2PSProcessControllerNode extends Node {

    private _rosService: ROSService;
    private _processService: ProcessService;

    constructor() {
        super(NODE_NAME);
        this.getLogger().info(`${this.name()} created`);

        this._rosService = new ROSService(this);
        this._processService = new ProcessService(this);
    }
}