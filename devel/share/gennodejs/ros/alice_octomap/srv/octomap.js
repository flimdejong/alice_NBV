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
      this.total_voxels = null;
      this.occupied_voxels = null;
      this.x_values = null;
      this.y_values = null;
      this.z_values = null;
      this.occupancy = null;
    }
    else {
      if (initObj.hasOwnProperty('total_voxels')) {
        this.total_voxels = initObj.total_voxels
      }
      else {
        this.total_voxels = 0;
      }
      if (initObj.hasOwnProperty('occupied_voxels')) {
        this.occupied_voxels = initObj.occupied_voxels
      }
      else {
        this.occupied_voxels = 0;
      }
      if (initObj.hasOwnProperty('x_values')) {
        this.x_values = initObj.x_values
      }
      else {
        this.x_values = [];
      }
      if (initObj.hasOwnProperty('y_values')) {
        this.y_values = initObj.y_values
      }
      else {
        this.y_values = [];
      }
      if (initObj.hasOwnProperty('z_values')) {
        this.z_values = initObj.z_values
      }
      else {
        this.z_values = [];
      }
      if (initObj.hasOwnProperty('occupancy')) {
        this.occupancy = initObj.occupancy
      }
      else {
        this.occupancy = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type octomapResponse
    // Serialize message field [total_voxels]
    bufferOffset = _serializer.int32(obj.total_voxels, buffer, bufferOffset);
    // Serialize message field [occupied_voxels]
    bufferOffset = _serializer.int32(obj.occupied_voxels, buffer, bufferOffset);
    // Serialize message field [x_values]
    bufferOffset = _arraySerializer.float64(obj.x_values, buffer, bufferOffset, null);
    // Serialize message field [y_values]
    bufferOffset = _arraySerializer.float64(obj.y_values, buffer, bufferOffset, null);
    // Serialize message field [z_values]
    bufferOffset = _arraySerializer.float64(obj.z_values, buffer, bufferOffset, null);
    // Serialize message field [occupancy]
    bufferOffset = _arraySerializer.bool(obj.occupancy, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type octomapResponse
    let len;
    let data = new octomapResponse(null);
    // Deserialize message field [total_voxels]
    data.total_voxels = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [occupied_voxels]
    data.occupied_voxels = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x_values]
    data.x_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y_values]
    data.y_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [z_values]
    data.z_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [occupancy]
    data.occupancy = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.x_values.length;
    length += 8 * object.y_values.length;
    length += 8 * object.z_values.length;
    length += object.occupancy.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alice_octomap/octomapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e49c62c1add6ce5ed13e188a48e66fc4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 total_voxels
    int32 occupied_voxels
    float64[] x_values
    float64[] y_values
    float64[] z_values
    bool[] occupancy
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new octomapResponse(null);
    if (msg.total_voxels !== undefined) {
      resolved.total_voxels = msg.total_voxels;
    }
    else {
      resolved.total_voxels = 0
    }

    if (msg.occupied_voxels !== undefined) {
      resolved.occupied_voxels = msg.occupied_voxels;
    }
    else {
      resolved.occupied_voxels = 0
    }

    if (msg.x_values !== undefined) {
      resolved.x_values = msg.x_values;
    }
    else {
      resolved.x_values = []
    }

    if (msg.y_values !== undefined) {
      resolved.y_values = msg.y_values;
    }
    else {
      resolved.y_values = []
    }

    if (msg.z_values !== undefined) {
      resolved.z_values = msg.z_values;
    }
    else {
      resolved.z_values = []
    }

    if (msg.occupancy !== undefined) {
      resolved.occupancy = msg.occupancy;
    }
    else {
      resolved.occupancy = []
    }

    return resolved;
    }
};

module.exports = {
  Request: octomapRequest,
  Response: octomapResponse,
  md5sum() { return 'e49c62c1add6ce5ed13e188a48e66fc4'; },
  datatype() { return 'alice_octomap/octomap'; }
};
