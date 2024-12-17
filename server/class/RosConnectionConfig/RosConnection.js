import ROSLIB from "roslib";
import ListenerBuilder from "./ListenerBuilder.js";
import PublisherBuilder from "./PublisherBuilder.js";
class RosConnection {
  constructor(url = "ws://localhost:9090") {
    this.ros = new ROSLIB.Ros({
      url: url,
    });
    this.listeners = {};
    this.publishers = {};
  }

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

  publishToPublisher(topicName, messageData, period = 100) {
    if (!this.publishers[topicName]) {
      console.error(`Publisher for topic ${topicName} does not exist.`);
      return;
    }

    const publisher = this.publishers[topicName];
    const message = new ROSLIB.Message(messageData);

    setInterval(() => {
      publisher.publish(message);
      console.log(`Published message to ${this.topicName}:`, message);
    }, period);

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
