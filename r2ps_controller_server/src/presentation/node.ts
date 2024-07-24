import * as cprocess from "child_process";
import * as rclnodejs from "rclnodejs";

const NODE_NAME: string = "r2ps_controller_server";

export default class R2PSControllerServerNode extends rclnodejs.Node {

    constructor() {
        super(NODE_NAME);
        this.getLogger().info(`${this.name()} created`);

        this.bindFunctions();

        const mainTimer: rclnodejs.Timer = this.createTimer(
            500,
            this.mainTimerCallback.bind(this)
        );
    }

    private bindFunctions(): void {
        this.mainTimerCallback = this.mainTimerCallback.bind(this);
        this.getCurrentNodeList = this.getCurrentNodeList.bind(this);
    }

    private mainTimerCallback(): void {
        this.getCurrentNodeList();
    }

    private getCurrentNodeList(): void {
        try {
            const currentNodeList: string[] = this.getNodeNames();

            for (const currentNode of currentNodeList) {
                if (currentNode === this.name() || currentNode.includes(this.name())) {
                    continue;
                }

                this.getLogger().info(`Current Node : ${JSON.stringify(currentNode)}`);

                cprocess.exec(`ps -ef | grep ${currentNode}`, (error: cprocess.ExecException | null, stdOut: string, stdErr: string) => {
                    if (error) {
                        this.getLogger().error(`exec error: ${error.message}`);
                        return;
                    }

                    const stdLines: string[] = stdOut.split('\n');
                    for (const stdLine of stdLines) {
                        if (stdLine.includes("grep")) {
                            continue;
                        }

                        const columns: string[] = stdLine.trim().split(/\s+/);
                        if (columns.length > 1) {
                            const pid: string = columns[1];
                            this.getLogger().info(`Extracted PID: ${pid}`);
                        }
                    }

                    if (stdErr) {
                        this.getLogger().error(`stderr: ${stdErr}`);
                        return;
                    }
                });
            }
        } catch (e: any) {
            this.getLogger().error(`${e}`);
        }
    }
}