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

class octomap_srv_clientRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomap_srv_clientRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomap_srv_clientRequest
    let len;
    let data = new octomap_srv_clientRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alice_octomap/octomap_srv_clientRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new octomap_srv_clientRequest(null);
    return resolved;
    }
};

class octomap_srv_clientResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.occupied_voxels = null;
    }
    else {
      if (initObj.hasOwnProperty('occupied_voxels')) {
        this.occupied_voxels = initObj.occupied_voxels
      }
      else {
        this.occupied_voxels = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomap_srv_clientResponse
    // Serialize message field [occupied_voxels]
    bufferOffset = _serializer.int32(obj.occupied_voxels, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomap_srv_clientResponse
    let len;
    let data = new octomap_srv_clientResponse(null);
    // Deserialize message field [occupied_voxels]
    data.occupied_voxels = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alice_octomap/octomap_srv_clientResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2ce102c71339eb15c8ed24c4bfa8169';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 occupied_voxels
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new octomap_srv_clientResponse(null);
    if (msg.occupied_voxels !== undefined) {
      resolved.occupied_voxels = msg.occupied_voxels;
    }
    else {
      resolved.occupied_voxels = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: octomap_srv_clientRequest,
  Response: octomap_srv_clientResponse,
  md5sum() { return 'c2ce102c71339eb15c8ed24c4bfa8169'; },
  datatype() { return 'alice_octomap/octomap_srv_client'; }
};
