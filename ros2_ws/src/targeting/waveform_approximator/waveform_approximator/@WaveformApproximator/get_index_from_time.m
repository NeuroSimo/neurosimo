function index = get_index_from_time(obj, time)
    % GET_INDEX_FROM_TIME Get the index corresponding to a time in a state trajectory.
    %
    %   Inputs:
    %       - TIME: a scalar representing the time
    %
    %   Outputs:
    %       - INDEX: a scalar representing the index of the time in the time
    %       vector
    %
    index = floor(time / obj.time_resolution) + 1;
