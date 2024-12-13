import cpp_bindings

# Override Python's native print() function.
def print(*args, sep=' ', end='\n', file=None, flush=False):
    # Ignore 'end' character as we don't actually want an explicit newline in the log.
    output = sep.join(map(str, args))

    cpp_bindings.log(output)

    # Handle the file and flush parameters if needed.
    if file is not None:
        file.write(output)
        if flush:
            file.flush()

def print_throttle(*args, period=1.0, sep=' ', end='\n', file=None, flush=False):
    assert period > 0.0, 'The period must be greater than zero.'

    # Ignore 'end' character as we don't actually want an explicit newline in the log.
    output = sep.join(map(str, args))

    cpp_bindings.log_throttle(output, period)

    # Handle the file and flush parameters if needed.
    if file is not None:
        file.write(output)
        if flush:
            file.flush()
