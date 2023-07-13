// Auto-generated. Do not edit!

// (in-package get_pointcloud.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Obsposelist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.obs_poses_list = null;
    }
    else {
      if (initObj.hasOwnProperty('obs_poses_list')) {
        this.obs_poses_list = initObj.obs_poses_list
      }
      else {
        this.obs_poses_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obsposelist
    // Serialize message field [obs_poses_list]
    // Serialize the length for message field [obs_poses_list]
    bufferOffset = _serializer.uint32(obj.obs_poses_list.length, buffer, bufferOffset);
    obj.obs_poses_list.forEach((val) => {
      bufferOffset = geometry_msgs.msg.TransformStamped.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obsposelist
    let len;
    let data = new Obsposelist(null);
    // Deserialize message field [obs_poses_list]
    // Deserialize array length for message field [obs_poses_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obs_poses_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obs_poses_list[i] = geometry_msgs.msg.TransformStamped.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.obs_poses_list.forEach((val) => {
      length += geometry_msgs.msg.TransformStamped.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'get_pointcloud/Obsposelist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0aaa214c546cd6dd8dd28dc01d8ebbac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/TransformStamped[] obs_poses_list
    ================================================================================
    MSG: geometry_msgs/TransformStamped
    # This expresses a transform from coordinate frame header.frame_id
    # to the coordinate frame child_frame_id
    #
    # This message is mostly used by the 
    # <a href="http://wiki.ros.org/tf">tf</a> package. 
    # See its documentation for more information.
    
    Header header
    string child_frame_id # the frame id of the child frame
    Transform transform
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Obsposelist(null);
    if (msg.obs_poses_list !== undefined) {
      resolved.obs_poses_list = new Array(msg.obs_poses_list.length);
      for (let i = 0; i < resolved.obs_poses_list.length; ++i) {
        resolved.obs_poses_list[i] = geometry_msgs.msg.TransformStamped.Resolve(msg.obs_poses_list[i]);
      }
    }
    else {
      resolved.obs_poses_list = []
    }

    return resolved;
    }
};

module.exports = Obsposelist;
