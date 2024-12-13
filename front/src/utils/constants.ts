export const ROS_URL =
  process.env.REACT_APP_ENV === 'dev' || process.env.REACT_APP_ENV === 'prod'
    ? 'ws://localhost:9090'
    : 'ws://rosbridge_test:9090'
