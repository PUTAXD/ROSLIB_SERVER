import ROSLIB from "roslib";

class RosConnection {
  constructor(url = "ws://localhost:9090") {
    this.ros = new ROSLIB.Ros({
      url: url,
      encoding: "ascii",
    });
    this.listeners = {};
    this.publishers = {}; // Dictionary untuk menyimpan listener berdasarkan topik
  }

  // Inisialisasi koneksi ROS dan mendengarkan event 'connection', 'error', dan 'close'
  init() {
    this.ros.on("connection", () => {
      console.log("Connected to WebSocket server");
    });

    this.ros.on("error", (error) => {
      console.log("Error connecting to WebSocket server:", error);
    });

    this.ros.on("close", () => {
      console.log("Connection to WebSocket server closed");
    });
  }

  // Menambahkan listener ke dictionary 'listeners' berdasarkan topik
  addListener(topicName, messageType, onMessageCallback) {
    const listener = new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType: messageType,
    });

    // Menyimpan listener di dictionary
    this.listeners[topicName] = listener;
    console.log(`Subscribed to topic: ${topicName}`);
  }

  // Mengakses listener dari dictionary dan langsung subscribe ke topik
  subscribeToListener(topicName, onMessageCallback) {
    const listener = this.listeners[topicName];
    if (listener) {
      listener.subscribe(onMessageCallback);
    } else {
      console.error(`Listener for topic ${topicName} not found`);
    }
  }

  // Menghapus listener berdasarkan nama topik
  removeListener(topicName) {
    const listener = this.listeners[topicName];
    if (listener) {
      listener.unsubscribe();
      console.log(`Unsubscribed from topic: ${topicName}`);
      delete this.listeners[topicName];
    } else {
      console.log(`No listener found for topic: ${topicName}`);
    }
  }

  // Menghapus semua listener
  removeAllListeners() {
    for (const topicName in this.listeners) {
      const listener = this.listeners[topicName];
      listener.unsubscribe();
      console.log(`Unsubscribed from topic: ${topicName}`);
    }
    this.listeners = {}; // Menghapus semua listener dari dictionary
  }

  // Menutup koneksi ROS
  closeConnection() {
    this.ros.close();
    console.log("WebSocket connection closed.");
  }
}

// Contoh penggunaan

// Membuat instance RosConnection
const rosConnection = new RosConnection();

// Inisialisasi koneksi
rosConnection.init();

// Menambahkan listener untuk topik /data_robot dengan tipe pesan robot_pkg/data_robot
rosConnection.addListener("/data_robot", "robot_pkg/data_robot", (message) => {
  console.log("Custom handler for /data_robot:", message);
});

// Mengakses listener untuk /data_robot dan mencoba subscribe
// rosConnection.subscribeToListener("/data_robot", (message) => {
//   console.log("Manually handling /data_robot message:", message);
// });

// Menghapus listener untuk topik tertentu
// rosConnection.removeListener('/data_robot');

// Menghapus semua listener
// rosConnection.removeAllListeners();

// Jika ingin menutup koneksi dan unsubscribe
// rosConnection.closeConnection();
