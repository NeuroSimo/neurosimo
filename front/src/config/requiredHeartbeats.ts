/** Nodes that publish std_msgs/msg/Empty heartbeats on the running stack. */
export const REQUIRED_NEUROSIMO_HEARTBEATS = [
  { id: 'eeg_bridge', topic: '/neurosimo/eeg_bridge/heartbeat', label: 'eeg_bridge' },
  { id: 'eeg_simulator', topic: '/neurosimo/eeg_simulator/heartbeat', label: 'eeg_simulator' },
  { id: 'preprocessor', topic: '/neurosimo/preprocessor/heartbeat', label: 'preprocessor' },
  { id: 'presenter', topic: '/neurosimo/presenter/heartbeat', label: 'presenter' },
  { id: 'decider', topic: '/neurosimo/decider/heartbeat', label: 'decider' },
  {
    id: 'experiment_coordinator',
    topic: '/neurosimo/experiment_coordinator/heartbeat',
    label: 'experiment_coordinator',
  },
  { id: 'resource_monitor', topic: '/neurosimo/resource_monitor/heartbeat', label: 'resource_monitor' },
  { id: 'trigger_timer', topic: '/neurosimo/trigger_timer/heartbeat', label: 'trigger_timer' },
] as const
