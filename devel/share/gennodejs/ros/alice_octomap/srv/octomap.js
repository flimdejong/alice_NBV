// Auto-generated. Do not edit!

// (in-package alice_octomap.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class octomapRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomapRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomapRequest
    let len;
    let data = new octomapRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alice_octomap/octomapRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Service receives no input
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new octomapRequest(null);
    return resolved;
    }
};

class octomapResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.occupied_voxels = null;
      this.total_voxels = null;
    }
    else {
      if (initObj.hasOwnProperty('occupied_voxels')) {
        this.occupied_voxels = initObj.occupied_voxels
      }
      else {
        this.occupied_voxels = 0;
      }
      if (initObj.hasOwnProperty('total_voxels')) {
        this.total_voxels = initObj.total_voxels
      }
      else {
        this.total_voxels = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomapResponse
    // Serialize message field [occupied_voxels]
    bufferOffset = _serializer.int32(obj.occupied_voxels, buffer, bufferOffset);
    // Serialize message field [total_voxels]
    bufferOffset = _serializer.int32(obj.total_voxels, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomapResponse
    let len;
    let data = new octomapResponse(null);
    // Deserialize message field [occupied_voxels]
    data.occupied_voxels = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [total_voxels]
    data.total_voxels = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alice_octomap/octomapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd6c656edb646bcaf655c3aa23e2812b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 occupied_voxels
    int32 total_voxels
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new octomapResponse(null);
    if (msg.occupied_voxels !== undefined) {
      resolved.occupied_voxels = msg.occupied_voxels;
    }
    else {
      resolved.occupied_voxels = 0
    }

    if (msg.total_voxels !== undefined) {
      resolved.total_voxels = msg.total_voxels;
    }
    else {
      resolved.total_voxels = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: octomapRequest,
  Response: octomapResponse,
  md5sum() { return 'd6c656edb646bcaf655c3aa23e2812b3'; },
  datatype() { return 'alice_octomap/octomap'; }
};
