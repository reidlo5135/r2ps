import * as rclnodejs from 'rclnodejs';

async function example() {

  await rclnodejs.init();
  let node = rclnodejs.createNode('MyNode');

  let publisher = node.createPublisher('std_msgs/msg/String', 'foo');
  let cnt = 0;
  let msg = rclnodejs.createMessageObject('std_msgs/msg/String');
  node.createTimer(1000, () => {
    msg.data = `msg: ${cnt += 1}`
    publisher.publish(msg);
  });

  node.spin();
}

(async function main(): Promise<void> {
  example();
})().catch((): void => {
  process.exitCode = 1
});

process.on("SIGINT", (): void => {
  console.log("Terminated by CTRL-C");
  rclnodejs.shutdown();
  process.exit();
});