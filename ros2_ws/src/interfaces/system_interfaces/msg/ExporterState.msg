# State of the session exporter

uint8 IDLE = 0
uint8 EXPORTING = 1
uint8 ERROR = 2

uint8 state
string recording_name
string export_path
float32 progress  # 0.0 to 1.0
