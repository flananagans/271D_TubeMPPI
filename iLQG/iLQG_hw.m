%make a class for iLQG, put params in and then the method is the function

classdef iLQG_hw
    properties
        car = []; %car object
        
        lims = [-5 5;         % wheel angle limits (radians)
             -5  5]; %control limits
        parallel = true; %use parallel line-search?
        Alpha = 10.^linspace(0,-3,11); %backtracking coefficients
        tolFun = 1e-7; %reduction exit critereon
        tolGrad = 1e-4; %gradient exit critereon
        maxIter = 500; %maximum iterations
        lambda = 1; %initial value for lambda
        dlambda = 1 %initial value for dlambda
        lambdaFactor = 1.6
        lambdaMax = 1e10; %lambda maximum value
        lambdaMin = 1e-6; %lambda minimum value
        regType = 1; %regularization type 1: q_uu+lambda*eye(); 2: V_xx+lambda*eye()
        zMin = 0; %minimal accepted reduction ratio
        diffFn = []; %user-defined diff for sub-space optimization
        plot = 1; % 0: no;  k>0: every k iters; k<0: every k iters, with derivs window
        print = 2; %0: no;  1: final; 2: iter; 3: iter, detailed
        plotFn = @(x)0; %user-defined graphics callback
        cost = []; %initial cost for pre-rolled trajectory 
        DYNCST; 

    end

    methods

        % Constructor
        function obj = iLQG_hw(system)

            % NIKI: If you need the sampling time, that is given by
            % system.dt
            obj.car = system;
            obj.DYNCST = @(x,u,i) obj.car_dyn_cst(x,u,i);

        end

        function [x, u, L, Vx, Vxx, cost, trace, stop] = solve(obj, x0, u0)
        % solve - solve the deterministic finite-horizon optimal control problem.
        %
        %        minimize sum_i CST(x(:,i),u(:,i)) + CST(x(:,end))
        %            u
        %        s.t.  x(:,i+1) = DYN(x(:,i),u(:,i))
        %
        % Inputs
        % ======
        % DYNCST - A combined dynamics and cost function. It is called in
        % three different formats.
        %
        %  1) step:
        %   [xnew,c] = DYNCST(x,u,i) is called during the forward pass. 
        %   Here the state x and control u are vectors: size(x)==[n 1],  
        %   size(u)==[m 1]. The cost c and time index i are scalars.
        %   If Op.parallel==true (the default) then DYNCST(x,u,i) is be 
        %   assumed to accept vectorized inputs: size(x,2)==size(u,2)==K
        %  
        %  2) final:
        %   [~,cnew] = DYNCST(x,nan) is called at the end the forward pass to compute
        %   the final cost. The nans indicate that no controls are applied.
        %  
        %  3) derivatives:
        %   [~,~,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = DYNCST(x,u,I) computes the
        %   derivatives along a trajectory. In this case size(x)==[n N+1] where N
        %   is the trajectory length. size(u)==[m N+1] with NaNs in the last column
        %   to indicate final-cost. The time indexes are I=(1:N).
        %   Dimensions match the variable names e.g. size(fxu)==[n n m N+1]
        %   note that the last temporal element N+1 is ignored for all tensors
        %   except cx and cxx, the final-cost derivatives.
        %
        % x0 - The initial state from which to solve the control problem. 
        % Should be a column vector. If a pre-rolled trajectory is available
        % then size(x0)==[n N+1] can be provided and Op.cost set accordingly.
        %
        % u0 - The initial control sequence. A matrix of size(u0)==[m N]
        % where m is the dimension of the control and N is the number of state
        % transitions. 
        %
        %
        % Op - optional parameters, see below
        %
        % Outputs
        % =======
        % x - the optimal state trajectory found by the algorithm.
        %     size(x)==[n N+1]
        %
        % u - the optimal open-loop control sequence.
        %     size(u)==[m N]
        %
        % L - the optimal closed loop control gains. These gains multiply the
        %     deviation of a simulated trajectory from the nominal trajectory x.
        %     size(L)==[m n N]
        %
        % Vx - the gradient of the cost-to-go. size(Vx)==[n N+1]
        %
        % Vxx - the Hessian of the cost-to-go. size(Vxx)==[n n N+1]
        %
        % cost - the costs along the trajectory. size(cost)==[1 N+1]
        %        the cost-to-go is V = fliplr(cumsum(fliplr(cost)))
        %
        % lambda - the final value of the regularization parameter
        %
        % trace - a trace of various convergence-related values. One row for each
        %         iteration, the columns of trace are
        %         [iter lambda alpha g_norm dcost z sum(cost) dlambda]
        %         see below for details.
        %
        % timing - timing information
        %
        %
        %
        % BIBTeX:
        %
        % @INPROCEEDINGS{
        % author={Tassa, Y. and Mansard, N. and Todorov, E.},
        % booktitle={Robotics and Automation (ICRA), 2014 IEEE International Conference on},
        % title={Control-Limited Differential Dynamic Programming},
        % year={2014}, month={May}, doi={10.1109/ICRA.2014.6907001}}
        %---------------------- user-adjustable parameters ------------------------
        
        DYNCST = obj.DYNCST;
        
        % --- initial sizes and controls
        n   = size(x0, 1);          % dimension of state vector
        m   = size(u0, 1);          % dimension of control vector
        N   = size(u0, 2);          % number of state transitions
        u   = u0;                   % initial control sequence
        % --- proccess options
        
        lambda   = obj.lambda;
        dlambda  = obj.dlambda;
        verbosity = obj.print;
        % --- initialize trace data structure
        trace = struct('iter',nan,'lambda',nan,'dlambda',nan,'cost',nan,...
                'alpha',nan,'grad_norm',nan,'improvement',nan,'reduc_ratio',nan,...
                'time_derivs',nan,'time_forward',nan,'time_backward',nan);
        trace = repmat(trace,[min(obj.maxIter,1e6) 1]);
        trace(1).iter = 1;
        trace(1).lambda = obj.lambda;
        trace(1).dlambda = obj.dlambda;
        % --- initial trajectory
        if size(x0,2) == 1
            diverge = true;
            for alpha = obj.Alpha
                %[x,un,cost]  = forward_pass(x0(:,1),alpha*u,[],[],[],1,DYNCST,obj.lims,[]);
                %% start forward pass
                x0 = x0(:,1); u = alpha*u; L = []; x = []; du = []; Alpha = 1; lims = obj.lims; diff = [];
                n        = size(x0,1);
                K        = length(Alpha);
                K1       = ones(1,K); % useful for expansion
                m        = size(u,1);
                N        = size(u,2);
                xnew        = zeros(n,K,N);
                xnew(:,:,1) = x0(:,ones(1,K));
                unew        = zeros(m,K,N);
                cnew        = zeros(1,K,N+1);
                for i = 1:N
                    unew(:,:,i) = u(:,i*K1);
                    
                    if ~isempty(du)
                        unew(:,:,i) = unew(:,:,i) + du(:,i)*Alpha;
                    end    
                    
                    if ~isempty(L)
                        if ~isempty(diff)
                            dx = diff(xnew(:,:,i), x(:,i*K1));
                        else
                            dx          = xnew(:,:,i) - x(:,i*K1);
                        end
                        unew(:,:,i) = unew(:,:,i) + L(:,:,i)*dx;
                    end
                    
                    if ~isempty(lims)
                        unew(:,:,i) = min(lims(:,2*K1), max(lims(:,1*K1), unew(:,:,i)));
                    end
                    [xnew(:,:,i+1), cnew(:,:,i)]  = DYNCST(xnew(:,:,i), unew(:,:,i), i*K1);
                end
                [~, cnew(:,:,N+1)] = DYNCST(xnew(:,:,N+1),nan(m,K,1),i);
                % put the time dimension in the columns
                xnew = permute(xnew, [1 3 2]);
                unew = permute(unew, [1 3 2]);
                cnew = permute(cnew, [1 3 2]);
                x = xnew;
                un = unew;
                cost = cnew;
                %% end forward pass
                % simplistic divergence test
                if all(abs(x(:)) < 1e8)
                    u = un;
                    diverge = false;
                    break
                end
            end
        elseif size(x0,2) == N+1 % pre-rolled initial forward pass
            x        = x0;
            diverge  = false;
            if isempty(obj.cost)
                error('pre-rolled initial trajectory requires cost')
            else
                cost     = obj.cost;
            end
        else
            error('pre-rolled initial trajectory must be of correct length')
        end
        trace(1).cost = sum(cost(:));
        % user plotting
        obj.plotFn(x);
        if diverge
            [Vx,Vxx, stop]  = deal(nan);
            L        = zeros(m,n,N);
            cost     = [];
            trace    = trace(1);
            if verbosity > 0
                fprintf('\nEXIT: Initial control sequence caused divergence\n');
            end
            return
        end
        % constants, timers, counters
        flgChange   = 1;
        stop        = 0;
        dcost       = 0;
        z           = 0;
        expected    = 0;
        print_head  = 6; % print headings every print_head lines
        last_head   = print_head;
        t_start     = tic;
        if verbosity > 0
            fprintf('\n=========== begin iLQG ===========\n');
        end
%         graphics(obj.plot,x,u,cost,zeros(m,n,N),[],[],[],[],[],[],trace,1);
        for iter = 1:obj.maxIter
            if stop
                break;
            end
            trace(iter).iter = iter;    
            
            %====== STEP 1: differentiate dynamics and cost along new trajectory
            if flgChange
                t_diff = tic;

                xsplit = x;
                usplit = [u nan(m,1)];
                last = 1:N+1;
                [r,c] = size(xsplit);

                fx_split = zeros(4,4,c);
                fu_split = zeros(4,2,c);
                fxx_split = [];
                fxu_split = [];
                fuu_split = [];
                cx_split = zeros(4,c);
                cu_split = zeros(2,c);
                cxx_split = zeros(4,4,c);
                cxu_split = zeros(4,2,c);
                cuu_split = zeros(2,2,c);
                for j = 1:c
                    [~,~,fx_split(:,:,j), fu_split(:,:,j), fxx_split, fxu_split, fuu_split, cx_split(:,j), cu_split(:,j), cxx_split(:,:,j), cxu_split(:,:,j), cuu_split(:,:,j)]  = DYNCST(xsplit(:,j), usplit(:,j), last(:,j));
                end
                fx = fx_split;
                fu = fu_split;
                fxx = fxx_split;
                fxu = fxu_split;
                fuu = fuu_split;
                cx = cx_split;
                cu = cu_split;
                cxx = cxx_split;
                cxu = cxu_split;
                cuu = cuu_split;
                %[~,~,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu]   = DYNCST(x, [u nan(m,1)], 1:N+1);
                trace(iter).time_derivs = toc(t_diff);
                flgChange   = 0;
            end
            
            %====== STEP 2: backward pass, compute optimal control law and cost-to-go
            backPassDone   = 0;
            while ~backPassDone
                
                t_back   = tic;
                %[diverge, Vx, Vxx, l, L, dV] = back_pass(cx,cu,cxx,cxu,cuu,fx,fu,fxx,fxu,fuu,lambda,obj.regType,obj.lims,u);
                %% start back pass function
                regType = obj.regType; lims = obj.lims;
                % tensor multiplication for DDP terms
                vectens = @(a,b) permute(sum(bsxfun(@times,a,b),1), [3 2 1]);
                N  = size(cx,2);
                n  = numel(cx)/N;
                m  = numel(cu)/N;
                cx    = reshape(cx,  [n N]);
                cu    = reshape(cu,  [m N]);
                cxx   = reshape(cxx, [n n N]);
                cxu   = reshape(cxu, [n m N]);
                cuu   = reshape(cuu, [m m N]);
                k     = zeros(m,N-1);
                K     = zeros(m,n,N-1);
                Vx    = zeros(n,N);
                Vxx   = zeros(n,n,N);
                dV    = [0 0];
                Vx(:,N)     = cx(:,N);
                Vxx(:,:,N)  = cxx(:,:,N);
                diverge  = 0;
                for i = N-1:-1:1
                    
                    Qu  = cu(:,i)      + fu(:,:,i)'*Vx(:,i+1);
                    Qx  = cx(:,i)      + fx(:,:,i)'*Vx(:,i+1);
                    Qux = cxu(:,:,i)'  + fu(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
                    if ~isempty(fxu)
                        fxuVx = vectens(Vx(:,i+1),fxu(:,:,:,i));
                        Qux   = Qux + fxuVx;
                    end
                    
                    Quu = cuu(:,:,i)   + fu(:,:,i)'*Vxx(:,:,i+1)*fu(:,:,i);
                    if ~isempty(fuu)
                        fuuVx = vectens(Vx(:,i+1),fuu(:,:,:,i));
                        Quu   = Quu + fuuVx;
                    end
                    
                    Qxx = cxx(:,:,i)   + fx(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
                    if ~isempty(fxx)
                        Qxx = Qxx + vectens(Vx(:,i+1),fxx(:,:,:,i));
                    end
                    
                    Vxx_reg = (Vxx(:,:,i+1) + lambda*eye(n)*(regType == 2));
                    
                    Qux_reg = cxu(:,:,i)'   + fu(:,:,i)'*Vxx_reg*fx(:,:,i);
                    if ~isempty(fxu)
                        Qux_reg = Qux_reg + fxuVx;
                    end
                    
                    QuuF = cuu(:,:,i)  + fu(:,:,i)'*Vxx_reg*fu(:,:,i) + lambda*eye(m)*(regType == 1);
                    
                    if ~isempty(fuu)
                        QuuF = QuuF + fuuVx;
                    end
                    
                    if isempty(lims) || lims(1,1) > lims(1,2)
                        % no control limits: Cholesky decomposition, check for non-PD
                        [R,d] = chol(QuuF);
                        if d ~= 0
                            diverge  = i;
                            return;
                        end
                        
                        % find control law
                        kK = -R\(R'\[Qu Qux_reg]);
                        k_i = kK(:,1);
                        K_i = kK(:,2:n+1);
                        
                    else        % solve Quadratic Program
                        lower = lims(:,1)-u(:,i);
                        upper = lims(:,2)-u(:,i);
                        
                        if eig(QuuF) > 0
                            [k_i,result,R,free] = boxQP(QuuF,Qu,lower,upper,k(:,min(i+1,N-1)));
                        else
                            A = QuuF;
                            delta = 10e-5;
                            [V,D] = eig((A + A')/2);
                            D = diag(max(diag(D), delta));  % delta > 0
                            A_new = V * D * V';
                            [k_i,result,R,free] = boxQP(A_new,Qu,lower,upper,k(:,min(i+1,N-1)));
                        end
                            if result < 1
                            diverge  = i;
                            display(result)
                            display(i)
                            display(eig(QuuF))
                            return;
                        end
                        
                        K_i    = zeros(m,n);
                        if any(free)
                            Lfree        = -R\(R'\Qux_reg(free,:));
                            K_i(free,:)   = Lfree;
                        end
                        
                    end
                    
                    % update cost-to-go approximation
                    dV          = dV + [k_i'*Qu  .5*k_i'*Quu*k_i];
                    Vx(:,i)     = Qx  + K_i'*Quu*k_i + K_i'*Qu  + Qux'*k_i;
                    Vxx(:,:,i)  = Qxx + K_i'*Quu*K_i + K_i'*Qux + Qux'*K_i;
                    Vxx(:,:,i)  = .5*(Vxx(:,:,i) + Vxx(:,:,i)');
                    
                    % save controls/gains
                    k(:,i)      = k_i;
                    K(:,:,i)    = K_i;
                end
                
                l = k;
                L = K;
                %% end backpass function
                trace(iter).time_backward = toc(t_back);
                
                if diverge
                    if verbosity > 2
                        fprintf('Cholesky failed at timestep %d.\n',diverge);
                    end
                    dlambda   = max(dlambda * obj.lambdaFactor, obj.lambdaFactor);
                    lambda    = max(lambda * dlambda, obj.lambdaMin);
                    if lambda > obj.lambdaMax
                        break;
                    end
                    continue
                end

                backPassDone      = 1;
            end
            % check for termination due to small gradient
            g_norm         = mean(max(abs(l) ./ (abs(u)+1),[],1));
            trace(iter).grad_norm = g_norm;
            if g_norm < obj.tolGrad && lambda < 1e-5
                dlambda   = min(dlambda / obj.lambdaFactor, 1/obj.lambdaFactor);
                lambda    = lambda * dlambda * (lambda > obj.lambdaMin);
                if verbosity > 0
                    fprintf('\nSUCCESS: gradient norm < tolGrad\n');
                end
                break;
            end
            
            %====== STEP 3: line-search to find new control sequence, trajectory, cost
            fwdPassDone  = 0;
            if backPassDone
                t_fwd = tic;
                if obj.parallel  % parallel line-search
                    %[xnew,unew,costnew] = forward_pass(x0 ,u, L, x(:,1:N), l, obj.Alpha, DYNCST,obj.lims,obj.diffFn);
                    %% start forward pass
                    x0 = x0(:,1); u = u; x = x(:,1:N); du = l; Alpha = obj.Alpha; lims = obj.lims; diff = obj.diffFn;
                    n        = size(x0,1);
                    K        = length(Alpha);
                    K1       = ones(1,K); % useful for expansion
                    m        = size(u,1);
                    N        = size(u,2);
                    xnew        = zeros(n,K,N);
                    xnew(:,:,1) = x0(:,ones(1,K));
                    unew        = zeros(m,K,N);
                    cnew        = zeros(1,K,N+1);
                    for i = 1:N
                        unew(:,:,i) = u(:,i*K1);
                        
                        if ~isempty(du)
                            unew(:,:,i) = unew(:,:,i) + du(:,i)*Alpha;
                        end    
                        
                        if ~isempty(L)
                            if ~isempty(diff)
                                dx = diff(xnew(:,:,i), x(:,i*K1));
                            else
                                dx          = xnew(:,:,i) - x(:,i*K1);
                            end
                            unew(:,:,i) = unew(:,:,i) + L(:,:,i)*dx;
                        end
                        
                        if ~isempty(lims)
                            unew(:,:,i) = min(lims(:,2*K1), max(lims(:,1*K1), unew(:,:,i)));
                        end
                        xsplit = xnew(:,:,i);
                        usplit = unew(:,:,i);
                        [r,c] = size(xsplit);

                        xnew_split = zeros(r,c);
                        cnew_split = zeros(1,c);
                        for j = 1:c
                            [xnew_split(:,j), cnew_split(1,j)]  = DYNCST(xsplit(:,j), usplit(:,j), i*K1(:,j));
                        end
                        xnew(:,:,i+1) = xnew_split;
                        cnew(:,:,i) = cnew_split;
                    end
                    xsplit = xnew(:,:,N+1);
                    usplit = nan(m,K,1);
                    [r,c] = size(xsplit);

                    cnew_split = zeros(1,c);
                    for j = 1:c
                        [~, cnew_split(1,j)]  = DYNCST(xsplit(:,j), usplit(:,j), i);
                    end
                   
                    cnew(:,:,i) = cnew_split;
                    %[~, cnew(:,:,N+1)] = DYNCST(xnew(:,:,N+1),nan(m,K,1),i);
                    % put the time dimension in the columns
                    xnew = permute(xnew, [1 3 2]);
                    unew = permute(unew, [1 3 2]);
                    cnew = permute(cnew, [1 3 2]);
                    
                    costnew = cnew;
                    %% end forward pass
                    Dcost               = sum(cost(:)) - sum(costnew,2);
                    [dcost, w]          = max(Dcost);
                    alpha               = obj.Alpha(w);
                    expected            = -alpha*(dV(1) + alpha*dV(2));
                    if expected > 0
                        z = dcost/expected;
                    else
                        z = sign(dcost);
                        warning('non-positive expected reduction: should not occur');
                    end
                    if (z > obj.zMin)
                        fwdPassDone = 1;
                        costnew     = costnew(:,:,w);
                        xnew        = xnew(:,:,w);
                        unew        = unew(:,:,w);
                    end
                else            % serial backtracking line-search
                    for alpha = obj.Alpha
                        %[xnew,unew,costnew]   = forward_pass(x0 ,u+l*alpha, L, x(:,1:N),[],1,DYNCST,obj.lims,obj.diffFn);
                        %% start forward pass
                        x0 = x0(:,1); u = u+l*alpha; x = x(:,1:N); du = []; Alpha = 1; lims = obj.lims; diff = obj.diffFn;
                        n        = size(x0,1);
                        K        = length(Alpha);
                        K1       = ones(1,K); % useful for expansion
                        m        = size(u,1);
                        N        = size(u,2);
                        xnew        = zeros(n,K,N);
                        xnew(:,:,1) = x0(:,ones(1,K));
                        unew        = zeros(m,K,N);
                        cnew        = zeros(1,K,N+1);
                        for i = 1:N
                            unew(:,:,i) = u(:,i*K1);
                            
                            if ~isempty(du)
                                unew(:,:,i) = unew(:,:,i) + du(:,i)*Alpha;
                            end    
                            
                            if ~isempty(L)
                                if ~isempty(diff)
                                    dx = diff(xnew(:,:,i), x(:,i*K1));
                                else
                                    dx          = xnew(:,:,i) - x(:,i*K1);
                                end
                                unew(:,:,i) = unew(:,:,i) + L(:,:,i)*dx;
                            end
                            
                            if ~isempty(lims)
                                unew(:,:,i) = min(lims(:,2*K1), max(lims(:,1*K1), unew(:,:,i)));
                            end
                            [xnew(:,:,i+1), cnew(:,:,i)]  = DYNCST(xnew(:,:,i), unew(:,:,i), i*K1);
                        end
                        [~, cnew(:,:,N+1)] = DYNCST(xnew(:,:,N+1),nan(m,K,1),i);
                        % put the time dimension in the columns
                        xnew = permute(xnew, [1 3 2]);
                        unew = permute(unew, [1 3 2]);
                        cnew = permute(cnew, [1 3 2]);
                        
                        costnew = cnew;
                        %% end forward pass
                        dcost    = sum(cost(:)) - sum(costnew(:));
                        expected = -alpha*(dV(1) + alpha*dV(2));
                        if expected > 0
                            z = dcost/expected;
                        else
                            z = sign(dcost);
                            warning('non-positive expected reduction: should not occur');
                        end
                        if (z > obj.zMin)
                            fwdPassDone = 1;
                            break;
                        end
                    end
                end
                if ~fwdPassDone
                    alpha = nan; % signals failure of forward pass
                end
                trace(iter).time_forward = toc(t_fwd);
            end
            
            %====== STEP 4: accept step (or not), draw graphics, print status
            
            % print headings
            if verbosity > 1 && last_head == print_head
                last_head = 0;
                fprintf('%-12s','iteration','cost','reduction','expected','gradient','log10(lambda)')
                fprintf('\n');
            end
            
            if fwdPassDone
                
                % print status
                if verbosity > 1
                    fprintf('%-12d%-12.6g%-12.3g%-12.3g%-12.3g%-12.1f\n', ...
                        iter, sum(cost(:)), dcost, expected, g_norm, log10(lambda));
                    last_head = last_head+1;
                end
                
                % decrease lambda
                dlambda   = min(dlambda / obj.lambdaFactor, 1/obj.lambdaFactor);
                lambda    = lambda * dlambda * (lambda > obj.lambdaMin);
                
                % accept changes
                u              = unew;
                x              = xnew;
                cost           = costnew;
                flgChange      = 1;
                obj.plotFn(x);
                
                % terminate ?
                if dcost < obj.tolFun
                    if verbosity > 0
                        fprintf('\nSUCCESS: cost change < tolFun\n');
                    end
                    break;
                end
                
            else % no cost improvement
                % increase lambda
                dlambda  = max(dlambda * obj.lambdaFactor, obj.lambdaFactor);
                lambda   = max(lambda * dlambda, obj.lambdaMin);
                
                % print status
                if verbosity > 1
                    fprintf('%-12d%-12s%-12.3g%-12.3g%-12.3g%-12.1f\n', ...
                        iter,'NO STEP', dcost, expected, g_norm, log10(lambda));           
                    last_head = last_head+1;
                end     
                
                % terminate ?
                if lambda > obj.lambdaMax,
                    if verbosity > 0
                        fprintf('\nEXIT: lambda > lambdaMax\n');
                    end
                    break;
                end
            end
            % update trace
            trace(iter).lambda      = lambda;
            trace(iter).dlambda     = dlambda;
            trace(iter).alpha       = alpha;
            trace(iter).improvement = dcost;
            trace(iter).cost        = sum(cost(:));
            trace(iter).reduc_ratio = z;
            %stop = graphics(obj.plot,x,u,cost,L,Vx,Vxx,fx,fxx,fu,fuu,trace(1:iter),0);
        end
        % save lambda/dlambda
        trace(iter).lambda      = lambda;
        trace(iter).dlambda     = dlambda;
        if stop
            if verbosity > 0
                fprintf('\nEXIT: Terminated by user\n');
            end
        end
        if iter == obj.maxIter
            if verbosity > 0
                fprintf('\nEXIT: Maximum iterations reached.\n');
            end
        end
        if ~isempty(iter)
            diff_t = [trace(1:iter).time_derivs];
            diff_t = sum(diff_t(~isnan(diff_t)));
            back_t = [trace(1:iter).time_backward];
            back_t = sum(back_t(~isnan(back_t)));
            fwd_t = [trace(1:iter).time_forward];
            fwd_t = sum(fwd_t(~isnan(fwd_t)));
            total_t = toc(t_start);
            if verbosity > 0
                fprintf(['\n'...
                    'iterations:   %-3d\n'...
                    'final cost:   %-12.7g\n' ...
                    'final grad:   %-12.7g\n' ...
                    'final lambda: %-12.7e\n' ...
                    'time / iter:  %-5.0f ms\n'...
                    'total time:   %-5.2f seconds, of which\n'...
                    '  derivs:     %-4.1f%%\n'...
                    '  back pass:  %-4.1f%%\n'...
                    '  fwd pass:   %-4.1f%%\n'...
                    '  other:      %-4.1f%% (graphics etc.)\n'...
                    '=========== end iLQG ===========\n'],...
                    iter,sum(cost(:)),g_norm,lambda,1e3*total_t/iter,total_t,...
                    [diff_t, back_t, fwd_t, (total_t-diff_t-back_t-fwd_t)]*100/total_t);
            end
            trace    = trace(~isnan([trace.iter]));
        %     timing   = [diff_t back_t fwd_t total_t-diff_t-back_t-fwd_t];
            %graphics(obj.plot,x,u,cost,L,Vx,Vxx,fx,fxx,fu,fuu,trace,2); % draw legend
        else
            error('Failure: no iterations completed, something is wrong.')
        end
        end

        function [xnew,unew,cnew] = forward_pass(x0,u,L,x,du,Alpha,DYNCST,lims,diff)

        % parallel forward-pass (rollout)
        % internally time is on the 3rd dimension, 
        % to facillitate vectorized dynamics calls
        n        = size(x0,1);
        K        = length(Alpha);
        K1       = ones(1,K); % useful for expansion
        m        = size(u,1);
        N        = size(u,2);
        xnew        = zeros(n,K,N);
        xnew(:,:,1) = x0(:,ones(1,K));
        unew        = zeros(m,K,N);
        cnew        = zeros(1,K,N+1);
        for i = 1:N
            unew(:,:,i) = u(:,i*K1);
            
            if ~isempty(du)
                unew(:,:,i) = unew(:,:,i) + du(:,i)*Alpha;
            end    
            
            if ~isempty(L)
                if ~isempty(diff)
                    dx = diff(xnew(:,:,i), x(:,i*K1));
                else
                    dx          = xnew(:,:,i) - x(:,i*K1);
                end
                unew(:,:,i) = unew(:,:,i) + L(:,:,i)*dx;
            end
            
            if ~isempty(lims)
                unew(:,:,i) = min(lims(:,2*K1), max(lims(:,1*K1), unew(:,:,i)));
            end
            [xnew(:,:,i+1), cnew(:,:,i)]  = DYNCST(xnew(:,:,i), unew(:,:,i), i*K1);
        end
        [~, cnew(:,:,N+1)] = DYNCST(xnew(:,:,N+1),nan(m,K,1),i);
        % put the time dimension in the columns
        xnew = permute(xnew, [1 3 2]);
        unew = permute(unew, [1 3 2]);
        cnew = permute(cnew, [1 3 2]);
        end


        function [diverge, Vx, Vxx, k, K, dV] = back_pass(cx,cu,cxx,cxu,cuu,fx,fu,fxx,fxu,fuu,lambda,regType,lims,u)
        % Perform the Ricatti-Mayne backward pass
        % tensor multiplication for DDP terms
        vectens = @(a,b) permute(sum(bsxfun(@times,a,b),1), [3 2 1]);
        N  = size(cx,2);
        n  = numel(cx)/N;
        m  = numel(cu)/N;
        cx    = reshape(cx,  [n N]);
        cu    = reshape(cu,  [m N]);
        cxx   = reshape(cxx, [n n N]);
        cxu   = reshape(cxu, [n m N]);
        cuu   = reshape(cuu, [m m N]);
        k     = zeros(m,N-1);
        K     = zeros(m,n,N-1);
        Vx    = zeros(n,N);
        Vxx   = zeros(n,n,N);
        dV    = [0 0];
        Vx(:,N)     = cx(:,N);
        Vxx(:,:,N)  = cxx(:,:,N);
        diverge  = 0;
        for i = N-1:-1:1
            
            Qu  = cu(:,i)      + fu(:,:,i)'*Vx(:,i+1);
            Qx  = cx(:,i)      + fx(:,:,i)'*Vx(:,i+1);
            Qux = cxu(:,:,i)'  + fu(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
            if ~isempty(fxu)
                fxuVx = vectens(Vx(:,i+1),fxu(:,:,:,i));
                Qux   = Qux + fxuVx;
            end
            
            Quu = cuu(:,:,i)   + fu(:,:,i)'*Vxx(:,:,i+1)*fu(:,:,i);
            if ~isempty(fuu)
                fuuVx = vectens(Vx(:,i+1),fuu(:,:,:,i));
                Quu   = Quu + fuuVx;
            end
            
            Qxx = cxx(:,:,i)   + fx(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
            if ~isempty(fxx)
                Qxx = Qxx + vectens(Vx(:,i+1),fxx(:,:,:,i));
            end
            
            Vxx_reg = (Vxx(:,:,i+1) + lambda*eye(n)*(regType == 2));
            
            Qux_reg = cxu(:,:,i)'   + fu(:,:,i)'*Vxx_reg*fx(:,:,i);
            if ~isempty(fxu)
                Qux_reg = Qux_reg + fxuVx;
            end
            
            QuuF = cuu(:,:,i)  + fu(:,:,i)'*Vxx_reg*fu(:,:,i) + lambda*eye(m)*(regType == 1);
            
            if ~isempty(fuu)
                QuuF = QuuF + fuuVx;
            end
            
            if nargin < 13 || isempty(lims) || lims(1,1) > lims(1,2)
                % no control limits: Cholesky decomposition, check for non-PD
                [R,d] = chol(QuuF);
                if d ~= 0
                    diverge  = i;
                    return;
                end
                
                % find control law
                kK = -R\(R'\[Qu Qux_reg]);
                k_i = kK(:,1);
                K_i = kK(:,2:n+1);
                
            else        % solve Quadratic Program
                lower = lims(:,1)-u(:,i);
                upper = lims(:,2)-u(:,i);
                
                if eig(QuuF) > 0
                    [k_i,result,R,free] = boxQP(QuuF,Qu,lower,upper,k(:,min(i+1,N-1)));
                else
                    A = QuuF;
                    delta = 10e-5;
                    [V,D] = eig((A + A')/2);
                    D = diag(max(diag(D), delta));  % delta > 0
                    A_new = V * D * V';
                    [k_i,result,R,free] = boxQP(A_new,Qu,lower,upper,k(:,min(i+1,N-1)));               
                end
                if result < 1
                    diverge  = i;
                    return;
                end
                
                K_i    = zeros(m,n);
                if any(free)
                    Lfree        = -R\(R'\Qux_reg(free,:));
                    K_i(free,:)   = Lfree;
                end
                
            end
            
            % update cost-to-go approximation
            dV          = dV + [k_i'*Qu  .5*k_i'*Quu*k_i];
            Vx(:,i)     = Qx  + K_i'*Quu*k_i + K_i'*Qu  + Qux'*k_i;
            Vxx(:,:,i)  = Qxx + K_i'*Quu*K_i + K_i'*Qux + Qux'*K_i;
            Vxx(:,:,i)  = .5*(Vxx(:,:,i) + Vxx(:,:,i)');
            
            % save controls/gains
            k(:,i)      = k_i;
            K(:,:,i)    = K_i;
        end
        end

        
        function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = car_dyn_cst(obj,x,u,i)
        % combine car dynamics and cost
        % use helper function finite_difference() to compute derivatives

        assert(isnumeric(obj.car.x) && all(isfinite(obj.car.x(:))), 'Error: obj.car.x is not numeric or contains invalid values.');
        cost = obj.car.calculateCost;
        assert(isnumeric(cost) , 'Error: obj.car.calculateCost did not return a numeric value.');
        %assert(nargout == 2, 'car_dyn_cst should return exactly 2 outputs.');
        
        

        %give new state and control inputs to system
        u(isnan(u)) = 0;
        obj.car.x = x;
        obj.car.setControl(u);

        %update system state
         
        if sum(sum(isnan(obj.car.u))) > 0
            obj.car.u(isnan(obj.car.u)) = 0;
        end
        obj.car.updateState;


        if nargout == 2
            f = obj.car.x;
            c = obj.car.calculateCost;
        else
            % state and control indices
            ix = 1:4;
            iu = 5:6;
            
            % dynamics first derivatives
            xu_dyn  = @(xu) obj.sysDyn(xu);
            J       = obj.finite_difference(xu_dyn, [x; u]);
            fx      = J(:,ix,:);
            fu      = J(:,iu,:);
            
            % dynamics second derivatives
            [fxx,fxu,fuu] = deal([]);
          
            
            % cost first derivatives
            xu_cost = @(xu) obj.calculateCost(xu);
            J       = squeeze(obj.finite_difference(xu_cost, [x; u]));
            cx      = J(:,ix,:);
            cu      = J(:,iu,:);
            
            % cost second derivatives
            xu_Jcst = @(xu) obj.calculateCost(xu);
            JJ      = obj.finite_difference(xu_Jcst, [x; u]);
            JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
            cxx     = JJ(ix,ix,:);
            cxu     = JJ(ix,iu,:);
            cuu     = JJ(iu,iu,:);
            
            [f,c] = deal([]);
        end
        end


        function J = finite_difference(obj, fun, x, h)
        % simple finite-difference derivatives
        % assumes the function fun() is vectorized
        
        if nargin < 4
            h = 2^-17;
        end
        
        [n, K]  = size(x);
        H       = [zeros(n,1) h*eye(n)];
        H       = permute(H, [1 3 2]);
        X       = x+H;
        X       = reshape(X, n, K*(n+1));
        Y       = fun(X);
        m       = numel(Y)/(K*(n+1));
        Y       = reshape(Y, m, K, n+1);
        J       = (Y(:,:,2:end)+ -Y(:,:,1)) / h;
        J       = permute(J, [1 3 2]);

        end

        %dynamics function for iLQG
        function x_new = sysDyn(obj,x)
            x_new = obj.car.A*x(1:4, :) + obj.car.B*x(5:6, :);
        end

        % Computing Costs and Contraints
        function c = calculateCost(obj, x)
            [row col] = size(x);
            c = zeros(1, col);
            for i = 1:col
                if isnan(x(5,i)) || isnan(x(6,i))
                    c(1,col)  = x(1:4,col)'*obj.car.Q*x(1:4,col);
                else
                    c(1,i) = x(1:4,i)'*obj.car.Q*x(1:4,i) + x(5:6,i)'*obj.car.R*x(5:6,i);
                end
            end
            
            
        end



    end

end

