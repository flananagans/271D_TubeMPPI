classdef Distance_Sensor < handle
    properties
        obs_flag = 0;
    end
    function obj = Distance_Sensor(actual_states, covariance, mean)
        obj.obs_flag = 0;


    end
    function set_obs_flag(obj, flag)
        obj.obs_flag = flag;

    end

    function obs_est = sensor_state_est(obj, true_obstacle_position, mean, variance)
        


end
