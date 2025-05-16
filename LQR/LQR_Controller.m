


%%% Class for LQG Function %%%

classdef LQR_Controller <handle
    properties
        A = []; 
        B = [];
        Q = [];
        R = [];



    end
    methods
    function obj=LQR_Controller(A,B,Qn,Wn,Q,R,Qi)
        %A,B State Space Matrices
        %Qn: covariance matrix for process noise
        %Wn: covariance matrix measurement noise
        %Q and R: Weigting Matrices
        obj.A = A; 
        obj.B = B; 
        obj.Qn = Qn;
        obj.Wn = Wn; 
        obj.Q = Q; 
        obj.R = R; 
        % obj.QXU = blkdiag(Q,R);
        % obj.QWU = blkdiag(Qn,Rn);
        % obj.QI = Qi;
    end
    function K = findGain(obj)
        [K,~,~] = dlqr(obj.A,obj.B,obj.Q, obj.R);
    end
    end
end
