import * as cprocess from "child_process";
import * as rclnodejs from "rclnodejs";
import { Node, Options, Publisher, Service, ServiceRequestMessage, ServiceResponse, QoS, Timer, r2ps_msgs } from "rclnodejs";

export default class ProcessService {

    private _node: Node;
    private currentProcessesCheckTimer: Timer;
    private currentProcessListPublisher: Publisher<"r2ps_msgs/msg/ProcessList">;
    private processControlService: Service;

    constructor(node: Node) {
        this._node = node;
        this.bindFunctions();

        this.currentProcessesCheckTimer = this._node.createTimer(
            500,
            this.currentProcessesCheckTimerCallback.bind(this)
        );

        const currentProcessListPublisherOpts: Options = { qos: QoS.profileSystemDefault };
        this.currentProcessListPublisher = this._node.createPublisher(
            "r2ps_msgs/msg/ProcessList",
            "/r2ps/ps/process/list",
            currentProcessListPublisherOpts
        );
        
        this.processControlService = this._node.createService(
            "r2ps_msgs/srv/ProcessControl",
            `/${this._node.name()}/ps/control`,
            this.processControlServiceCallback.bind(this)
        )
    }

    private bindFunctions(): void {
        this.currentProcessesCheckTimerCallback = this.currentProcessesCheckTimerCallback.bind(this);
        this.getAndPublishProcessList = this.getAndPublishProcessList.bind(this);
    }

    private currentProcessesCheckTimerCallback(): void {
        this.getAndPublishProcessList();
    }

    public getAndPublishProcessList(): void {
        try {
            const currentProcessList: r2ps_msgs.msg.Process[] = [];
            const processList: r2ps_msgs.msg.ProcessList = rclnodejs.createMessageObject("r2ps_msgs/msg/ProcessList");

            cprocess.exec("ps aux | grep ros", (error: cprocess.ExecException | null, stdOut: string, stdErr: string) => {
                if (error) {
                    console.error(`exec error: ${error.message}`);
                    return;
                }

                const stdLines: string[] = stdOut.split('\n');

                if (stdLines.length === 0) {
                    console.info("No processes found, publishing empty process list.");
                    this.currentProcessListPublisher.publish(processList);
                    return;
                }

                for (const stdLine of stdLines) {
                    if (stdLine.includes("grep") || stdLine.includes("echo") || stdLine.includes("pub")) {
                        continue;
                    }

                    if (stdLine === this._node.name() || stdLine.includes(this._node.name()) || stdLine.includes("r2ps")) {
                        continue;
                    }

                    if (stdLine.includes("_ros2") || stdLine.includes("daemon")) {
                        continue;
                    }

                    const stdColumns: string[] = stdLine.trim().split(/\s+/);

                    if (stdColumns.length > 1) {
                        console.info("-------------------------");
                        console.info(`User : ${stdColumns[0]}`);
                        console.info(`Pid : ${stdColumns[1]}`);
                        console.info(`Pname : ${stdColumns[11]}`);
                        console.info("-------------------------\n");

                        const pid: string = stdColumns[1];
                        const pname: string = stdColumns[11];
                        
                        const currentProcess: r2ps_msgs.msg.Process = rclnodejs.createMessageObject("r2ps_msgs/msg/Process");
                        currentProcess.pid = parseInt(pid, 10);
                        currentProcess.pname = pname;
                        currentProcessList.push(currentProcess);
                    }
                }

                if (stdErr) {
                    console.error(`stderr: ${stdErr}`);
                    return;
                }

                processList.process_list = currentProcessList;
                this.currentProcessListPublisher.publish(processList);
            });
        } catch (e: any) {
            console.error(`ps error : ${e}`);
            return;
        }
    }

    private processControlServiceCallback(
        request: ServiceRequestMessage<"r2ps_msgs/srv/ProcessControl">,
        response: ServiceResponse<"r2ps_msgs/srv/ProcessControl">
    ): void {
        console.info(`behavior : ${JSON.stringify(request.behavior)}`);

        const behavior: r2ps_msgs.msg.Behavior = rclnodejs.createMessageObject("r2ps_msgs/msg/Behavior");

        let result: r2ps_msgs.srv.ProcessControl_Response = response.template;
        result.result = true;
        response.send(result);
    }
}