clear
close all

car = DiscreteLinearSystem;
ilqg = iLQG_hw(car);
x0 = [10;5;0;0];
u0 = 10*randn(2,500);

[x, u, L, Vx, Vxx, cost, trace, stop] = ilqg.solve(x0, u0);

plot(x(1,:), x(2,:))
hold on
plot(x(1,1), x(2,1), '*r')
plot(x(1,end), x(2,end),'*b')

figure;
plot(u0(:,1), u0(:,2))
hold on
plot(u(:,1), u(:,2))