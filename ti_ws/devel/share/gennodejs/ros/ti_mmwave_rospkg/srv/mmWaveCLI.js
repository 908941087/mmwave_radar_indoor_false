// Auto-generated. Do not edit!

// (in-package ti_mmwave_rospkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class mmWaveCLIRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.comm = null;
    }
    else {
      if (initObj.hasOwnProperty('comm')) {
        this.comm = initObj.comm
      }
      else {
        this.comm = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mmWaveCLIRequest
    // Serialize message field [comm]
    bufferOffset = _serializer.string(obj.comm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mmWaveCLIRequest
    let len;
    let data = new mmWaveCLIRequest(null);
    // Deserialize message field [comm]
    data.comm = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.comm.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ti_mmwave_rospkg/mmWaveCLIRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '705dab568ba6ff458350c8b88cb19648';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string comm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mmWaveCLIRequest(null);
    if (msg.comm !== undefined) {
      resolved.comm = msg.comm;
    }
    else {
      resolved.comm = ''
    }

    return resolved;
    }
};

class mmWaveCLIResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.resp = null;
    }
    else {
      if (initObj.hasOwnProperty('resp')) {
        this.resp = initObj.resp
      }
      else {
        this.resp = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mmWaveCLIResponse
    // Serialize message field [resp]
    bufferOffset = _serializer.string(obj.resp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mmWaveCLIResponse
    let len;
    let data = new mmWaveCLIResponse(null);
    // Deserialize message field [resp]
    data.resp = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.resp.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ti_mmwave_rospkg/mmWaveCLIResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b791c1a4a4f0cee32b54dd1a73706a59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string resp
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mmWaveCLIResponse(null);
    if (msg.resp !== undefined) {
      resolved.resp = msg.resp;
    }
    else {
      resolved.resp = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: mmWaveCLIRequest,
  Response: mmWaveCLIResponse,
  md5sum() { return 'f079c47a57c95983638c539cc506d12d'; },
  datatype() { return 'ti_mmwave_rospkg/mmWaveCLI'; }
};
