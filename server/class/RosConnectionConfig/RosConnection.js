import ROSLIB from "roslib";
import ListenerBuilder from "./ListenerBuilder.js";
import PublisherBuilder from "./PublisherBuilder.js";
class RosConnection {
  constructor(url = "ws://localhost:9090") {
    this.url = url;
    this.ros = null;
    this.isConnect = false;
    this.listeners = {};
    this.publishers = {};
  }

  init() {
    this.ros = new ROSLIB.Ros({
      url: this.url,
    });

    this.ros.on("connection", () => {
      this.isConnect = true;
      console.log("Connected to WebSocket server");
    });

    this.ros.on("error", (error) => {
      this.isConnect = false;
      console.log("Error connecting to WebSocket server:", error);
      // this.retryConnection();
    });

    this.ros.on("close", () => {
      this.isConnect = false;
      console.log("Connection to WebSocket server closed");
    });
  }

  retryConnection() {
    console.log("Retrying connection in 2 seconds...");
    setTimeout(() => {
      this.init(); // Coba lagi untuk menyambung ke WebSocket server
    }, 2000); // Tunggu selama 5 detik sebelum mencoba lagi
  }

  addPublisher(topicName, messageType, queueSize = 10) {
    const builder = new PublisherBuilder(this.ros);
    builder
      .setTopic(topicName)
      .setMessageType(messageType)
      .setQueueSize(queueSize);

    const publisher = builder.build();

    // Menyimpan listener di dictionary
    this.publishers[topicName] = publisher;
  }

  addListener(topicName, messageType) {
    const builder = new ListenerBuilder(this.ros);
    builder.setTopic(topicName).setMessageType(messageType);

    const listener = builder.build();

    // Menyimpan listener di dictionary
    this.listeners[topicName] = listener;
    console.log(`Subscribed to topic: ${topicName}`);
  }

  publishWithTimer(topicName, messageData, period = 100) {
    let publishIntervalId;
    publishIntervalId = setInterval(() => {
      this.publishOnce(topicName, messageData);
    }, period);
  }

  publishOnce(topicName, messageData) {
    if (this.publishers[topicName] == null) {
      console.error(`Publisher for topic ${topicName} does not exist.`);
      return;
    }

    const publisher = this.publishers[topicName];
    const message = new ROSLIB.Message(messageData);
    publisher.publish(message);
    console.log(`Published message to ${topicName}:`, message);
  }

  subscribeToListener(topicName, onMessageCallback) {
    const listener = this.listeners[topicName];
    if (listener) {
      listener.subscribe(onMessageCallback);
    } else {
      console.error(`Listener for topic ${topicName} not found`);
    }
  }

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

  removeAllListeners() {
    for (const topicName in this.listeners) {
      const listener = this.listeners[topicName];
      listener.unsubscribe();
      console.log(`Unsubscribed from topic: ${topicName}`);
    }
    this.listeners = {};
  }

  closeConnection() {
    this.ros.close();
    console.log("WebSocket connection closed.");
  }
}

export default RosConnection;
