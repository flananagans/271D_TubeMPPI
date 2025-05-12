function [x_traj, u_traj, K_traj, cost_trace] = iLQG(x0, u_init, dyn_func, cost_func, max_iter)
    % iLQG controller implementation
    % Inputs:
    %   x0        - Initial state
    %   u_init    - Initial control sequence [m x N]
    %   dyn_func  - Function handle: [x_next, A, B] = dyn_func(x, u)
    %   cost_func - Function handle: [l, lx, lu, lxx, luu, lux] = cost_func(x, u)
    %   max_iter  - Maximum number of iterations
    %
    % Outputs:
    %   x_traj    - Optimized state trajectory
    %   u_traj    - Optimized control trajectory
    %   K_traj    - Feedback gain trajectory
    %   cost_trace - Cost at each iteration

    alpha = 0.1; % step size
    gamma = 0.5; % backtracking rate
    reg = 1e-6;  % regularization

    N = size(u_init, 2);
    n = length(x0);
    m = size(u_init, 1);
    u_traj = u_init;

    cost_trace = zeros(max_iter, 1);

    for iter = 1:max_iter
        % Forward pass
        x_traj = zeros(n, N+1);
        x_traj(:,1) = x0;
        cost = 0;
        A = cell(1, N);
        B = cell(1, N);
        l = zeros(1, N+1);
        lx = cell(1, N+1);
        lu = cell(1, N);
        lxx = cell(1, N+1);
        luu = cell(1, N);
        lux = cell(1, N);

        % Evaluate trajectory and cost
        for k = 1:N
            u = u_traj(:,k);
            x = x_traj(:,k);
            [x_next, Ak, Bk] = dyn_func(x, u);
            [lk, lxk, luk, lxxk, luuk, luxk] = cost_func(x, u);

            x_traj(:,k+1) = x_next;
            A{k} = Ak;
            B{k} = Bk;
            l(k) = lk;
            lx{k} = lxk;
            lu{k} = luk;
            lxx{k} = lxxk;
            luu{k} = luuk;
            lux{k} = luxk;

            cost = cost + lk;
        end
        % Final cost
        [lf, lxf, ~, lxxf, ~, ~] = cost_func(x_traj(:,N+1), zeros(m,1));
        l(N+1) = lf;
        lx{N+1} = lxf;
        lxx{N+1} = lxxf;
        cost = cost + lf;

        cost_trace(iter) = cost;
        fprintf('Iteration %d: Cost = %.4f\n', iter, cost);

        % Backward pass
        Vx = lx{N+1};
        Vxx = lxx{N+1};

        K_traj = cell(1, N);
        du_traj = zeros(m, N);

        for k = N:-1:1
            Qx = lx{k} + A{k}' * Vx;
            Qu = lu{k} + B{k}' * Vx;
            Qxx = lxx{k} + A{k}' * Vxx * A{k};
            Quu = luu{k} + B{k}' * Vxx * B{k} + reg * eye(m);
            Qux = lux{k} + B{k}' * Vxx * A{k};

            % Compute feedback gains
            K = -Quu \ Qux;
            du = -Quu \ Qu;

            K_traj{k} = K;
            du_traj(:,k) = du;

            Vx = Qx + K' * Quu * du + K' * Qu + Qux' * du;
            Vxx = Qxx + K' * Quu * K + K' * Qux + Qux' * K;
        end

        % Line search / forward pass
        x_new = x0;
        for k = 1:N
            dx = x_new - x_traj(:,k);
            du = du_traj(:,k) + K_traj{k} * dx;
            u_traj(:,k) = u_traj(:,k) + alpha * du;

            [x_new, ~, ~] = dyn_func(x_new, u_traj(:,k));
        end
    end
end
