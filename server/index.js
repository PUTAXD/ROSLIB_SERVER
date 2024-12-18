import RosConnection from "./class/RosConnectionConfig/RosConnection.js";
import Config from "./config.json" assert { type: "json" };
import MsgServer2Pc from "./msg/server/server2pc.json" assert { type: "json" };
import ROSLIB from "roslib";

const PC2BS = [];
const rosConnections = [];
let MSGSERVER2PC = { ...MsgServer2Pc };

// initiation and declaration
for (let key in Config) {
  const url = "ws://" + Config[key].ip + ":" + Config[key].port;
  const rosConnection = new RosConnection(url);
  rosConnections.push(rosConnection);
}

for (const rosConnection of rosConnections) {
  // console.log(rosConnection.url);
  rosConnection.init();
}

// subscriber robot;
rosConnections.forEach((rosConnection, i) => {
  rosConnections[i].addListener(
    "/master/pc2bs_roslib",
    "iris_msgs/pc2bs_roslib"
  );

  rosConnections[i].subscribeToListener("/master/pc2bs_roslib", (message) => {
    PC2BS[i] = { ...message };
  });
});

// connection to BS
const urlHamas = "ws://localhost:9090";
const hamasConnection = new RosConnection(urlHamas);
hamasConnection.init();

hamasConnection.addListener("/bs2pc_server", "robot_pkg/bs2pc_roslib");
hamasConnection.subscribeToListener("/bs2pc_server", (message) => {
  // console.log("hamas : ", message);
  MSGSERVER2PC = { ...message };
});

// publisher PC to BS From Server
rosConnections.forEach((rosConnection, i) => {
  hamasConnection.addPublisher("/pc2bs_server_" + i, "robot_pkg/pc2bs_roslib");
  //publish
  setInterval(() => {
    const msg = new ROSLIB.Message(PC2BS[i]);
    console.log("msg : ", msg, "i : ", i);
    hamasConnection.publishers["/pc2bs_server_" + i].publish(msg);
  }, 1000);
});

// publisher to robot
// rosConnections.forEach((rosConnection, i) => {
//   rosConnections[i].addPublisher("/bs2pc_roslib", "iris_msgs/bs2pc_roslib");
//   //publish

//   setInterval(() => {
//     const msg = new ROSLIB.Message(MSGSERVER2PC);
//     console.log("msg : ", msg);
//     rosConnections[i].publishers["/bs2pc_roslib"].publish(msg);
//   }, 1000);
// });

// auto reconnect
setInterval(() => {
  if (PC2BS) {
    console.log("robot : ", rosConnections[0].isConnect);
  }
  console.log(PC2BS)

  rosConnections.forEach((rosConnection, i) => {
    if (rosConnections[i].isConnect == false) {
      rosConnections[i].init();
      rosConnections[i].subscribeToListener("/master/pc2bs_roslib", (message) => {
        // console.log("data robot_hamas : ", rosConnections[0].isConnect);
        PC2BS[i] = { ...message };
      });
    }
  });
  if (hamasConnection.isConnect == false) {
    hamasConnection.init();
    hamasConnection.addListener("/bs2pc_server", "iris_msgs/bs2pc_roslib");
    hamasConnection.subscribeToListener("/bs2pc_server", (message) => {
      console.log("hamas : ", message);
      MsgServer2Pc = { ...message };
    });
  }
}, 1000);
