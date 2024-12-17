import ROSLIB from "roslib";

class ListenerBuilder {
  constructor(ros) {
    this.ros = ros; // ROSLIB.Ros instance
    this.topicName = ""; // Default topic
    this.messageType = ""; // Default message type
    this.onMessage = (message) => {}; // Default handler untuk pesan
  }

  setTopic(topicName) {
    this.topicName = topicName;
    return this; // Return this untuk chaining
  }

  setMessageType(messageType) {
    this.messageType = messageType;
    return this; // Return this untuk chaining
  }

  setOnMessage(callback) {
    this.onMessage = callback;
    return this; // Return this untuk chaining
  }

  build() {
    if (!this.topicName || !this.messageType) {
      throw new Error("Topic name and message type are required!");
    }

    const listener = new ROSLIB.Topic({
      ros: this.ros,
      name: this.topicName,
      messageType: this.messageType,
    });

    return listener;
  }
}

export default ListenerBuilder;
