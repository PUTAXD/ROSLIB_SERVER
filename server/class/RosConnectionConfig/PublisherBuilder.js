import ROSLIB from "roslib";

class PublisherBuilder {
  constructor(ros) {
    if (!ros) {
      throw new Error("ROS connection is required");
    }
    this.ros = ros;
    this.topicName = "";
    this.messageType = "";
    this.queueSize = 10;
  }

  setTopic(name) {
    this.topicName = name;
    return this;
  }

  setMessageType(type) {
    this.messageType = type;
    return this;
  }

  setQueueSize(size) {
    this.queueSize = size;
    return this;
  }

  build() {
    if (!this.topicName || !this.messageType) {
      throw new Error(
        "Topic name, message type, and message data are required"
      );
    }

    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: this.topicName,
      messageType: this.messageType,
    });

    console.log(
      `Publisher started on topic ${this.topicName} with message type ${this.messageType}`
    );
    return topic;
  }
}

export default PublisherBuilder;
