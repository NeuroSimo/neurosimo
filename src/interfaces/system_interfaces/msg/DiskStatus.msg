# Disk space status

uint64 free_bytes        # Available disk space in bytes
uint64 total_bytes       # Total disk space in bytes

uint64 warning_threshold_bytes  # Warning threshold in bytes
uint64 error_threshold_bytes    # Error threshold in bytes
bool is_ok               # True if free_bytes >= error_threshold_bytes
