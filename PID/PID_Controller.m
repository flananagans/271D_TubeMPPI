
classdef PID_Controller <handle
    properties
        Kp = 0; 
        Ki = 0; 
        Kd = 0; 
        error; 
        past_error; 
        dt; 
        
    end
    methods
        function obj=PID_Controller(Kp, Ki, Kd,dt)
            %A,B State Space Matrices
            %Kp, Ki, Kd, 
            obj.Kp = Kp; 
            obj.Ki = Ki; 
            obj.Kd = Kd; 
            obj.error = 0;
            obj.past_error = 0;
            obj.dt = dt;
    
        end
        function u = PID_U(obj,e)
            error_change = e - obj.past_error;
            u = obj.Kp*e + obj.Kd*error_change + obj.Ki*e*obj.dt;
       
        end
    end

end
