%Computes the linear and angular acceleration of the box
%given its current position and orientation
%INPUTS:
%x: current x position of the box
%y: current y position of the box
%theta: current orientation of the box
%box_params: a struct containing the parameters that describe the system
%Fields:
%box_params.m: mass of the box
%box_params.I: moment of inertia w/respect to centroid
%box_params.g: acceleration due to gravity
%box_params.k_list: list of spring stiffnesses
%box_params.l0_list: list of spring natural lengths
%box_params.P_world: 2 x n list of static mounting
% points for the spring (in the world frame)
%box_params.P_box: 2 x n list of mounting points
% for the spring (in the box frame)
%
%OUTPUTS
%ax: x acceleration of the box
%ay: y acceleration of the box
%atheta: angular acceleration of the box
function [ax,ay,atheta] = compute_accel(x,y,theta,box_params)
P_C = [x ; y];
P_i = compute_rbt(x,y,theta,box_params.P_box);
P_iC = P_i - P_C;
F_spring_list = zeros(2, length(box_params.k_list));
for i = 1:length(box_params.k_list)
    l0 = box_params.l0_list(i);
    k = box_params.k_list(i);
    PA = P_i(:, i);
    PB = box_params.P_world(:, i);
    F_spring_list(:,i) = compute_spring_force(k,l0,PA,PB);
    tau_ext_list = P_iC(1)*F_spring_list(2, i) - P_iC(2)*F_spring_list(1,i);
end

F_grav = [0; -box_params.m * box_params.g];
linear_accel = (F_grav + sum(F_spring_list,2)) / box_params.m;
ax = linear_accel(1);
ay = linear_accel(2);
atheta = sum(tau_ext_list)/box_params.I;
end