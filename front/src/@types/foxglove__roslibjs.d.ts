declare module '@foxglove/roslibjs' {
  import ROSLIB from 'roslib';
  export default ROSLIB;
  export import Ros = ROSLIB.Ros;
  export import Topic = ROSLIB.Topic;
  export import Service = ROSLIB.Service;
  export import ServiceRequest = ROSLIB.ServiceRequest;
  export import ServiceResponse = ROSLIB.ServiceResponse;
  export import Param = ROSLIB.Param;
  export import Message = ROSLIB.Message;
  export import Transform = ROSLIB.Transform;
  export import Quaternion = ROSLIB.Quaternion;
  export import Vector3 = ROSLIB.Vector3;
  export import Pose = ROSLIB.Pose;
}
