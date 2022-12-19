%clear; close all; clc;

x = 0; y = 0; r = 0.1; c = 'b';

th = 0:pi/10:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;

clear th r x y c

x_circle = [x_circle x_circle x_circle x_circle];
y_circle = [y_circle y_circle y_circle y_circle];
t = transpose(linspace(1,40,length(x_circle)));

circles = plot(x_circle, y_circle);

hold on

figure(2);

circles = plot(x_circle, y_circle);

hold on;

figure(1);

x_circle = [t, transpose(x_circle)];
y_circle = [t, transpose(y_circle)];


% x_circle_out_lin = out.x_circle_out_lin.data;
% y_circle_out_lin = out.y_circle_out_lin.data;
% 
% circles = plot(x_circle_out_lin, y_circle_out_lin);

title('Sistema linearizado');

axis equal

figure(2);

x_circle_out_nlin = out.x_circle_out_nlin.data;
y_circle_out_nlin = out.y_circle_out_nlin.data;

circles = plot(x_circle_out_nlin, y_circle_out_nlin);

title('Sistema não linearizado');

axis equal

