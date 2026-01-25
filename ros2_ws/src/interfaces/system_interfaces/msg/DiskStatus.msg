# Disk space status for a monitored path

string path              # Mount/directory path being monitored
uint64 free_bytes        # Available disk space in bytes
uint64 total_bytes       # Total disk space in bytes
float32 free_percent     # Free space as percentage (0-100)

uint64 threshold_bytes   # Configured minimum threshold in bytes
bool is_ok               # True if free_bytes >= threshold_bytes
