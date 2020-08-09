% test curveFittingCeres example

addpath('../mex');

a=3;
b=2;
c=1;

N = 1000;

x_data = linspace(0, 1, N)';
y_data = exp(a*x_data.*x_data + b*x_data + c);

y_data_noised = y_data + randn(N, 1);

abc_estimated = curveFittingCeresMex(x_data, y_data_noised)
y_data_estimated = exp(abc_estimated(1)*x_data.*x_data + abc_estimated(2)*x_data + abc_estimated(3));

figure();
plot(x_data, y_data, 'b-');
hold on;
plot(x_data, y_data_noised, 'y*');
hold on;
plot(x_data, y_data_estimated, 'bo');
