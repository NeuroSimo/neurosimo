def parse_size_string(size_str: str) -> int:
    """
    Parse human-readable size strings like '25GiB', '500MiB', '1GB', etc.
    to bytes.

    Supports units: GiB, MiB, KiB, GB, MB, KB (case insensitive)
    """
    if not size_str:
        raise ValueError("Empty size string")

    # Extract number and unit
    import re
    match = re.match(r'^(\d+(?:\.\d+)?)\s*([A-Za-z]+)$', size_str.strip())
    if not match:
        raise ValueError(f"Invalid size format: {size_str}. Expected format: <number><unit> (e.g., 25GiB)")

    number_str, unit = match.groups()
    number = float(number_str)

    # Define multipliers (base 10 for GB/MB/KB, base 2 for GiB/MiB/KiB)
    multipliers = {
        'GiB': 1024**3,
        'MiB': 1024**2,
        'KiB': 1024,
        'GB': 1000**3,
        'MB': 1000**2,
        'KB': 1000,
        'GIB': 1024**3,  # case insensitive
        'MIB': 1024**2,
        'KIB': 1024,
        'gib': 1024**3,
        'mib': 1024**2,
        'kib': 1024,
        'gb': 1000**3,
        'mb': 1000**2,
        'kb': 1000,
    }

    if unit not in multipliers:
        raise ValueError(f"Unsupported unit: {unit}. Supported units: {', '.join(multipliers.keys())}")

    return int(number * multipliers[unit])