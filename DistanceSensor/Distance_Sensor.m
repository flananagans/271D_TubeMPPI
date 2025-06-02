classdef Distance_Sensor < handle
    properties
        obs_flag = 0;
        covariance = 0; 
        mean = 0; 

    end
    %%
    function obj = Distance_Sensor(mean, covariance)
        obj.obs_flag = 0;
        obj.covariance = covariance; 
        obj.mean = mean;


    end
    function set_obs_flag(obj, flag)
        obj.obs_flag = flag;

    end
    function obs_est = sensor_state_est(obj, true_obstacle_position, mean, variance)
        perception_error = mvnrnd(mean, variance);n
        obs_est = true_obstacle_position + perception_error;

    end
    function obs_dist = observed_distance(obj, obe_est, robot_pos)
        er = obs_est - robot_pos; 
        obs_dist = norm(er);
end

