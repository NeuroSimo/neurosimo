function [parameter, error] = golden_section_search(obj, error_function, lower_bound, upper_bound)

    % Have a tolerance that is small enough compared to the time resolution.
    bounds_tolerance = obj.time_resolution / 4;

    % Golden ratio
    phi = (1 + sqrt(5)) / 2;

    % Calculate the initial points
    x1 = upper_bound - (upper_bound - lower_bound) / phi;
    x2 = lower_bound + (upper_bound - lower_bound) / phi;

    while true
        error1 = error_function(x1);
        error2 = error_function(x2);

        % If the error is smaller at x1, the new upper bound is x2
        if error1 < error2
            upper_bound = x2;
            x2 = x1;
            x1 = upper_bound - (upper_bound - lower_bound) / phi;
        else
            lower_bound = x1;
            x1 = x2;
            x2 = lower_bound + (upper_bound - lower_bound) / phi;
        end

        % If the bounds are close enough, break the loop.
        if upper_bound - lower_bound < bounds_tolerance
            break
        end
    end

    parameter = (upper_bound + lower_bound) / 2;
    error = error_function(parameter);
end
