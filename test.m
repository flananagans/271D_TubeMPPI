car = DiscreteLinearSystem;
ilqg = iLQG_hw(car);
x0 = [0;0;0;0];
u0 = .1*randn(2,500);

[x, u, L, Vx, Vxx, cost, trace, stop] = ilqg.solve(x0, u0);
