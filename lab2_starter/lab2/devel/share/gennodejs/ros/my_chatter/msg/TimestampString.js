// Auto-generated. Do not edit!

// (in-package my_chatter.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TimestampString {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_input = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('user_input')) {
        this.user_input = initObj.user_input
      }
      else {
        this.user_input = '';
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TimestampString
    // Serialize message field [user_input]
    bufferOffset = _serializer.string(obj.user_input, buffer, bufferOffset);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TimestampString
    let len;
    let data = new TimestampString(null);
    // Deserialize message field [user_input]
    data.user_input = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.user_input.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'my_chatter/TimestampString';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '028e8e15fbe176faa71f2b94e4d747e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string user_input
    float64 timestamp
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TimestampString(null);
    if (msg.user_input !== undefined) {
      resolved.user_input = msg.user_input;
    }
    else {
      resolved.user_input = ''
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    return resolved;
    }
};

module.exports = TimestampString;
