"""
Phastimate decider module for NeuroSimo.

Reference:
Zrenner, C., Galevska, D., Nieminen, J.O., Baur, D., Stefanou, M.I., Ziemann, U. (2020). 
The shaky ground truth of real-time phase estimation. NeuroImage, 214, 116761.
https://doi.org/10.1016/j.neuroimage.2020.116761

Available at:
https://www.sciencedirect.com/science/article/pii/S1053811920302262

MATLAB implementation:
https://github.com/bnplab/phastimate

This version is based on MATLAB adaptation of Phastimate by Joonas Laurinoja at Aalto University.
"""

from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np
from scipy.signal import filtfilt, hilbert
from spectrum import aryule

# Bandpass filter coefficients from MATLAB reference implementation
# TODO: Consider generating these coefficients using scipy.signal.firls for better maintainability
BANDPASS_FILTER_COEFFICIENTS = np.array([
    -0.0001, -0.0022, -0.0045, -0.0069, -0.0094, -0.0118, -0.0142, -0.0164, 
    -0.0184, -0.0202, -0.0217, -0.0228, -0.0235, -0.0237, -0.0235, -0.0228,
    -0.0216, -0.0199, -0.0177, -0.0152, -0.0122, -0.0088, -0.0053, -0.0015, 
     0.0025,  0.0064,  0.0104,  0.0142,  0.0177,  0.0210,  0.0240,  0.0264,
     0.0284,  0.0299,  0.0308,  0.0311,  0.0308,  0.0299,  0.0284,  0.0264, 
     0.0240,  0.0210,  0.0177,  0.0142,  0.0104,  0.0064,  0.0025, -0.0015,
    -0.0053, -0.0088, -0.0122, -0.0152, -0.0177, -0.0199, -0.0216, -0.0228, 
    -0.0235, -0.0237, -0.0235, -0.0228, -0.0217, -0.0202, -0.0184, -0.0164,
    -0.0142, -0.0118, -0.0094, -0.0069, -0.0045, -0.0022, -0.0001
])

# EEG channel indices for C3 referencing
C3_CHANNEL_INDEX = 4
REFERENCE_CHANNEL_INDICES = [20, 22, 24, 26]  # Reference channels for C3
REFERENCE_WEIGHT = 0.25

# Phase estimation constants
TARGET_PHASE_RADIANS = np.pi  # Target waveform trough
DEFAULT_PHASE_TOLERANCE = 0.05
DEFAULT_HILBERT_WINDOW_SIZE = 64
DEFAULT_EDGE_SAMPLES = 35
DEFAULT_AR_MODEL_ORDER = 15
DEFAULT_DOWNSAMPLE_RATIO = 10

# Processing timing constants
DEFAULT_PROCESSING_INTERVAL_SECONDS = 0.1
DEFAULT_BUFFER_SIZE_SECONDS = 1.0

class Decider:
    """
    Real-time EEG phase estimation and trigger scheduling using Phastimate algorithm.
    
    This class implements the Phastimate algorithm for real-time phase estimation
    of EEG signals and schedules triggers based on target phase detection.
    """
    
    def __init__(self, subject_id: str, num_eeg_channels: int, num_emg_channels: int, sampling_frequency: int):
        """
        Initialize the Decider with parameters and filter design.
        
        Args:
            subject_id: ID of the subject
            num_eeg_channels: Number of EEG channels (unused but kept for interface compatibility)
            num_emg_channels: Number of EMG channels (unused but kept for interface compatibility)
            sampling_frequency: Sampling frequency in Hz
        """
        self.subject_id = subject_id
        self.sampling_frequency = sampling_frequency

        # Phastimate algorithm parameters
        self.hilbert_window_size = DEFAULT_HILBERT_WINDOW_SIZE
        self.edge_samples = DEFAULT_EDGE_SAMPLES
        self.ar_model_order = DEFAULT_AR_MODEL_ORDER
        self.downsample_ratio = DEFAULT_DOWNSAMPLE_RATIO

        # Processing timing parameters
        self.periodic_processing_interval = DEFAULT_PROCESSING_INTERVAL_SECONDS
        self.buffer_size_seconds = DEFAULT_BUFFER_SIZE_SECONDS

        # Convert timing parameters to samples
        self.buffer_size_samples = int(self.buffer_size_seconds * sampling_frequency)

        # Filter coefficients
        self.bandpass_filter_coefficients = BANDPASS_FILTER_COEFFICIENTS

        # Phase targeting parameters
        self.target_phase_radians = TARGET_PHASE_RADIANS
        self.phase_tolerance = DEFAULT_PHASE_TOLERANCE
        
        # Maximum number of future samples to consider for trigger scheduling
        self.max_future_samples = int(self.edge_samples / 2)

        # Number of warm-up rounds to prevent first-call delays (see README.md for details)
        self.warm_up_rounds = 2

        print("Phastimate decider initialized for subject", subject_id, "with sampling frequency", sampling_frequency, "Hz.")

    def get_configuration(self) -> Dict[str, Union[int, bool, List]]:
        """
        Return the configuration for the processing interval and sample window.
        
        Returns:
            Dictionary containing processing configuration parameters
        """
        return {
            # Data configuration
            # Use seconds; convert buffer length in samples to seconds for clarity
            'sample_window': [-(self.buffer_size_samples - 1) / self.sampling_frequency, 0.0],
            
            # Periodic processing
            'periodic_processing_enabled': True,
            'periodic_processing_interval': 0.1,  # Process every 0.1 seconds
            'pulse_lockout_duration': 2.0,  # Prevent periodic processing for 2.0 seconds after pulse
            
            # Event system
            'event_processors': {
                'pulse': self.process_pulse,
            },
        }

    def process_periodic(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray, 
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """
        Process the EEG data to estimate phase and schedule a trigger periodically.
        
        Args:
            reference_time: Reference time point for the sample window (typically current sample time)
            time_offsets: Array of time offsets relative to reference_time
            eeg_buffer: EEG data buffer (samples x channels)
            emg_buffer: EMG data buffer (unused)
            reference_index: Index in the buffer corresponding to reference_time (where time_offsets[i] == 0)
            is_coil_at_target: Whether the coil is currently at the target position
            
        Returns:
            Dictionary with 'timed_trigger' key and execution time, or None if no trigger scheduled
        """
        # Extract C3 channel with common average reference
        c3_referenced_data = self._extract_c3_referenced_data(eeg_buffer)
        if c3_referenced_data is None:
            return None

        # Preprocess the data
        preprocessed_data = self._preprocess_eeg(c3_referenced_data)

        # Estimate future phases using Phastimate algorithm
        estimated_phases = self._estimate_phases(preprocessed_data)
        if estimated_phases is None:
            return None

        # Find optimal trigger timing
        trigger_timing = self._find_optimal_trigger_timing(estimated_phases, reference_time)
        
        return trigger_timing

    def _extract_c3_referenced_data(self, eeg_buffer: np.ndarray) -> Optional[np.ndarray]:
        """
        Extract C3 channel data with common average reference.
        
        Args:
            eeg_buffer: EEG data buffer (samples x channels)
            
        Returns:
            Referenced C3 data or None if extraction fails
        """
        try:
            c3_data = eeg_buffer[:, C3_CHANNEL_INDEX]
            reference_data = np.sum(eeg_buffer[:, REFERENCE_CHANNEL_INDICES], axis=1)
            return c3_data - REFERENCE_WEIGHT * reference_data
        except IndexError:
            print("Error: EEG buffer does not have expected number of channels")
            return None

    def _preprocess_eeg(self, data: np.ndarray) -> np.ndarray:
        """
        Preprocess EEG data by demeaning and downsampling.
        
        Args:
            data: Input EEG data
            
        Returns:
            Preprocessed and downsampled data
        """
        # Remove DC component
        demeaned_data = data - np.mean(data)
        
        # Downsample the data
        return demeaned_data[::self.downsample_ratio]

    def _estimate_phases(self, data: np.ndarray) -> Optional[np.ndarray]:
        """
        Estimate future phases using the Phastimate algorithm.
        
        Args:
            data: Preprocessed EEG data
            
        Returns:
            Array of estimated phases or None if estimation fails
        """
        estimated_phases, _ = self.phastimate(
            data,
            self.bandpass_filter_coefficients, 
            [1.0], 
            self.edge_samples, 
            self.ar_model_order, 
            self.hilbert_window_size
        )
        
        return estimated_phases

    def _find_optimal_trigger_timing(self, estimated_phases: np.ndarray, reference_time: float) -> Optional[Dict[str, Any]]:
        """
        Find optimal trigger timing based on estimated phases.
        
        Args:
            estimated_phases: Array of estimated phase values
            reference_time: Reference time point
            
        Returns:
            Dictionary with trigger timing or None if no suitable timing found
        """
        # Extract future phase estimates (second half of the estimation window)
        num_samples = estimated_phases.shape[0]
        future_phase_estimates = estimated_phases[num_samples // 2:]
        
        # Limit to maximum future samples
        future_phase_estimates = future_phase_estimates[:self.max_future_samples]

        # Calculate phase differences from target
        phase_differences = np.angle(np.exp(1j * (future_phase_estimates - self.target_phase_radians)))

        # Find the sample with minimum phase difference
        optimal_sample_index = np.argmin(np.abs(phase_differences))
        min_phase_difference = phase_differences[optimal_sample_index]

        # Check if phase difference is within tolerance
        if np.abs(min_phase_difference) > self.phase_tolerance:
            print(f'Not triggering: phase difference {min_phase_difference:.3f} radians exceeds tolerance')
            return None

        # Calculate trigger execution time
        time_offset_seconds = (optimal_sample_index * self.downsample_ratio) / self.sampling_frequency
        execution_time = reference_time + time_offset_seconds

        print(f'Triggering at time {execution_time:.3f} s based on phase estimate at time {reference_time:.1f} s')

        return {'timed_trigger': execution_time}

    def phastimate(self, data: np.ndarray, filter_b: np.ndarray, filter_a: List[float], 
                   edge_samples: int, ar_order: int, hilbert_window_size: int,
                   offset_correction: int = 0, iterations: Optional[int] = None, 
                   ar_method: str = 'aryule') -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Estimate the phase of the EEG signal using autoregressive modeling and Hilbert transform.
        
        This is the core Phastimate algorithm that performs:
        1. Bandpass filtering of the input signal
        2. Autoregressive (AR) modeling for forward prediction
        3. Hilbert transform for phase extraction
        
        Args:
            data: Input EEG signal
            filter_b: Numerator coefficients of the bandpass filter
            filter_a: Denominator coefficients of the bandpass filter  
            edge_samples: Number of edge samples to remove after filtering
            ar_order: Order of the autoregressive model
            hilbert_window_size: Size of the window for Hilbert transform
            offset_correction: Offset correction (unused)
            iterations: Number of forward prediction iterations
            ar_method: Method for AR parameter estimation ('aryule')
            
        Returns:
            Tuple of (estimated_phases, estimated_amplitudes) or (None, None) if estimation fails
        """
        # Calculate default number of iterations if not specified
        if iterations is None:
            iterations = edge_samples + int(np.ceil(hilbert_window_size / 2))

        # Validate data length for filtering
        min_padding_length = 3 * (max(len(filter_a), len(filter_b)) - 1)
        if data.shape[0] <= min_padding_length:
            print(f"Insufficient data for filtering: {data.shape[0]} <= {min_padding_length}")
            return None, None

        # Apply bandpass filter
        filtered_data = filtfilt(filter_b, filter_a, data)

        # Remove edge samples to mitigate filter transients
        if filtered_data.shape[0] <= 2 * edge_samples:
            print(f"Insufficient data after edge removal: {filtered_data.shape[0]} <= {2 * edge_samples}")
            return None, None

        edge_removed_data = filtered_data[edge_samples:-edge_samples]

        # Fit autoregressive model
        ar_coefficients = self._fit_ar_model(edge_removed_data, ar_order, ar_method)
        if ar_coefficients is None:
            return None, None

        # Perform forward prediction
        predicted_data = self._forward_predict(edge_removed_data, ar_coefficients, iterations)

        # Extract phase and amplitude using Hilbert transform
        return self._extract_phase_amplitude(predicted_data, hilbert_window_size)

    def _fit_ar_model(self, data: np.ndarray, ar_order: int, ar_method: str) -> Optional[np.ndarray]:
        """
        Fit autoregressive model to the data.
        
        Args:
            data: Input data for AR modeling
            ar_order: Order of the AR model
            ar_method: Method for AR parameter estimation
            
        Returns:
            AR coefficients or None if fitting fails
        """
        if len(data) < ar_order:
            print(f"Insufficient data for AR model: {len(data)} < {ar_order}")
            return None

        if ar_method == 'aryule':
            try:
                ar_params, _, _ = aryule(data, ar_order)
                # Flip and negate coefficients for prediction equation
                return -1 * ar_params[::-1]
            except Exception as e:
                print(f"AR model fitting failed: {e}")
                return None
        else:
            raise ValueError(f'Unknown AR method: {ar_method}')

    def _forward_predict(self, data: np.ndarray, ar_coefficients: np.ndarray, iterations: int) -> np.ndarray:
        """
        Perform forward prediction using AR model.
        
        Args:
            data: Historical data
            ar_coefficients: AR model coefficients
            iterations: Number of prediction steps
            
        Returns:
            Extended data with predictions
        """
        # Initialize prediction array
        total_length = len(data) + iterations
        predicted_data = np.zeros(total_length)
        predicted_data[:len(data)] = data

        # Perform iterative prediction
        ar_order = len(ar_coefficients)
        for i in range(iterations):
            prediction_index = len(data) + i
            data_window = predicted_data[prediction_index - ar_order:prediction_index]
            predicted_data[prediction_index] = np.sum(ar_coefficients * data_window)

        return predicted_data

    def _extract_phase_amplitude(self, data: np.ndarray, window_size: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Extract phase and amplitude using Hilbert transform.
        
        Args:
            data: Input signal data
            window_size: Size of the analysis window
            
        Returns:
            Tuple of (phases, amplitudes) or (None, None) if extraction fails
        """
        if data.shape[0] < window_size:
            print(f'Insufficient data for Hilbert transform: {data.shape[0]} < {window_size}')
            return None, None

        # Extract the analysis window (last window_size samples)
        analysis_window = data[-window_size:]

        # Compute analytic signal using Hilbert transform
        analytic_signal = hilbert(analysis_window)
        phases = np.angle(analytic_signal)
        amplitudes = np.abs(analytic_signal)

        return phases, amplitudes

    def process_eeg_trigger(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray, 
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process EEG trigger from the EEG device."""
        print(f'EEG trigger received at time {reference_time:.4f}')
        # Phastimate doesn't process EEG triggers, just log them
        return None

    def process_pulse(
            self, reference_time: float, reference_index: int, time_offsets: np.ndarray, 
            eeg_buffer: np.ndarray, emg_buffer: np.ndarray, is_coil_at_target: bool) -> Optional[Dict[str, Any]]:
        """Process pulse event."""
        print(f'Pulse event received at time {reference_time:.4f}')
        # Add your pulse event handling logic here
        return None
