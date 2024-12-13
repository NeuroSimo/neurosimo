# Set channel types
raw_data.set_channel_types(channel_types)

# Set the EEG reference to the right mastoid
raw_data.set_eeg_reference(ref_channels='average', ch_type='eeg', projection=False)

# Channels present in Tübingen REFTEP but not Aalto REFTEP
raw_data.drop_channels(['FT9', 'FT10', 'TP9', 'TP10'])

# Path to the Aalto channel names text file
channel_file_path = os.path.join(sound_directory, 'channel_names.txt')

# Load channel names from the text file
with open(channel_file_path, 'r') as file:
    channel_names = file.read().splitlines()

# Load leadfieldmatrix from the csv file
leadfield_file_path = os.path.join(sound_directory, 'LFM_Aalto_ReftepPP.csv')
leadfield_matrix_old = np.loadtxt(leadfield_file_path, delimiter=',')
leadfield_matrix_mean = np.mean(leadfield_matrix_old, axis=0)
leadfield_matrix = leadfield_matrix_old - leadfield_matrix_mean

# Get the channels present in the raw data
raw_channels = raw_data.ch_names

# Identify missing channels
missing_channels = [ch for ch in channel_names if ch not in raw_channels]
extra_channels = [ch for ch in raw_channels if ch not in channel_names and ch not in channel_types]

# Print missing and extra channels
#if missing_channels:
    #print(f"Missing channels: {missing_channels}")
#if extra_channels:
    #print(f"Extra channels: {extra_channels}")

# Remove the missing channels from channel_names and update the lead field matrix
for ch in missing_channels:
    idx = channel_names.index(ch)
    channel_names.remove(ch)
    leadfield_matrix = np.delete(leadfield_matrix, idx, axis=0)  # Remove corresponding row
    leadfield_matrix = np.delete(leadfield_matrix, idx, axis=1)  # Remove corresponding column

# Remove extra channels from raw data
raw_data.drop_channels(extra_channels)

# Combine EEG and EMG channel names
all_channel_names = channel_names + [ch for ch in raw_data.ch_names if ch in channel_types]

# Reorder the raw data channels to match the lead field matrix and include EMG channels
raw_data.reorder_channels(all_channel_names)
