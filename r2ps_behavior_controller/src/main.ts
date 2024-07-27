import * as rclnodejs from "rclnodejs";
import R2PSBehaviorControllerNode from "./presentation/node";

async function initialize(): Promise<void> {
  await rclnodejs.init();
  const node: rclnodejs.Node = new R2PSBehaviorControllerNode();
  node.spin();
}

(async function main(): Promise<void> {
  await initialize();
})().catch((e: any): void => {
  console.error(`${e}`);
  process.exitCode = 1;
});

process.on("SIGINT", (): void => {
  console.log("Terminated by CTRL-C");
  rclnodejs.shutdown();
  process.exit();
});