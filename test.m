clear
close all

car = DiscreteLinearSystem;
ilqg = iLQG_hw(car);
x0 = [10;5;0;0];
u0 = .1*randn(2,1000);

[x, u, L, Vx, Vxx, cost, trace, stop] = ilqg.solve(x0, u0);

plot(x(1,:), x(2,:))
hold on
plot(x(1,1), x(2,1), '*r')
plot(x(1,end), x(2,end),'*b')