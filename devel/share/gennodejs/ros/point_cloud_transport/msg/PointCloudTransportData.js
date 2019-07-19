// Auto-generated. Do not edit!

// (in-package point_cloud_transport.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PointCloudTransportData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.string_a = null;
      this.number_a = null;
    }
    else {
      if (initObj.hasOwnProperty('string_a')) {
        this.string_a = initObj.string_a
      }
      else {
        this.string_a = '';
      }
      if (initObj.hasOwnProperty('number_a')) {
        this.number_a = initObj.number_a
      }
      else {
        this.number_a = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PointCloudTransportData
    // Serialize message field [string_a]
    bufferOffset = _serializer.string(obj.string_a, buffer, bufferOffset);
    // Serialize message field [number_a]
    bufferOffset = _serializer.uint32(obj.number_a, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PointCloudTransportData
    let len;
    let data = new PointCloudTransportData(null);
    // Deserialize message field [string_a]
    data.string_a = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [number_a]
    data.number_a = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.string_a.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'point_cloud_transport/PointCloudTransportData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1041fd5d40af632dd7d721f20580cc73';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # TODO: implement message format for transporting compressed point cloud
    
    # placeholder
    
    string string_a
    
    uint32 number_a
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PointCloudTransportData(null);
    if (msg.string_a !== undefined) {
      resolved.string_a = msg.string_a;
    }
    else {
      resolved.string_a = ''
    }

    if (msg.number_a !== undefined) {
      resolved.number_a = msg.number_a;
    }
    else {
      resolved.number_a = 0
    }

    return resolved;
    }
};

module.exports = PointCloudTransportData;
