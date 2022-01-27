// Auto-generated. Do not edit!

// (in-package pkg_task5.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class msgRosIotResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag_success = null;
    }
    else {
      if (initObj.hasOwnProperty('flag_success')) {
        this.flag_success = initObj.flag_success
      }
      else {
        this.flag_success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type msgRosIotResult
    // Serialize message field [flag_success]
    bufferOffset = _serializer.bool(obj.flag_success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type msgRosIotResult
    let len;
    let data = new msgRosIotResult(null);
    // Deserialize message field [flag_success]
    data.flag_success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pkg_task5/msgRosIotResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a50265a5223d2c015174ffc1e1099b23';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # result
    bool flag_success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new msgRosIotResult(null);
    if (msg.flag_success !== undefined) {
      resolved.flag_success = msg.flag_success;
    }
    else {
      resolved.flag_success = false
    }

    return resolved;
    }
};

module.exports = msgRosIotResult;