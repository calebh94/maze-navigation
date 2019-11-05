close all;
clear all;
clc;

limit = 0.8;

while 1
    tmp = rand;
    if tmp > limit 
        disp("stop the loop");
        break
    end
    disp("keep running");
end

c_x = 2;
c_y = 2;
node_neighbor = [c_x-1 c_y-1; c_x c_y-1; c_x+1 c_y-1; c_x-1 c_y; c_x+1 c_y; c_x-1 c_y+1; c_x c_y+1; c_x+1 c_y+1];
disp(node_neighbor);

a = [1, 2];
b = [3, 4];
A = [a; b];
disp(size(A));
disp(A(1,:));

b = [4 5; 4 5; 1 2];
c = [4, 5];
d = [0 0];

[~,index] = ismember(d,c,'rows');

f_list = [];

f_value = 10;

f_list = [f_list; f_value];

f_value = 20;

f_list = [f_list; f_value];

f_list(1) = [];
f_list(1) = [];

disp(f_list);

