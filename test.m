car = DiscreteLinearSystem;
ilqg = iLQG_hw(car);
x0 = [0;0;0;0];
u0 = .1*randn(2,500);

ilqg.solve(x0, u0)


%x0 = x0(:,1); u = alpha*u; L = []; x = []; du = []; Alpha = 1; lims = obj.lims; diff = []; (for forward pass input)
% x=xnew(:,:,i); u=  unew(:,:,i); i=i*K1; (for DYNCST input)
% fun = xu_dyn; x = [x; u]; h = 2^-17; (for finite difference input)
