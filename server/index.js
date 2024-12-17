import RosConnection from "./class/RosConnectionConfig/RosConnection.js";

const rosConnection = new RosConnection();
rosConnection.init();

rosConnection.addListener("/data_robot", "robot_pkg/data_robot");
rosConnection.addPublisher("/server2pc", "robot_pkg/server2pc");

//publish
rosConnection.publishToPublisher(
  "/server2pc",
  {
    say: "Halo",
  },
  500
);

//subscribe
// rosConnection.subscribeToListener("/data_robot", (message) => {
//   console.log(message);
// });
