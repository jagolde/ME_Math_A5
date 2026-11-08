
%define system parameters
    box_params = struct();
    box_params.m = 3; % kg
    box_params.I = 5; % kg*m^2
    box_params.g = 9.81; % m/s^2

    %define per spring
    box_params.spring_number = 8;
    box_params.LG = 1.5; %m this is the length of spring compressed by gravity
    box_params.l0 = box_params.LG * 2;
    box_params.k_list = 50*(ones(1,box_params.spring_number)); % N/m
    box_params.l0_list = box_params.l0(ones(1,box_params.spring_number)); % m


    %define box parameters
    box_params.LW = 5; %m
    box_params.LH = 5; %m

    %define box boundaries
    box_params.Pbl_box = [-box_params.LW; -box_params.LH];
    box_params.Pbr_box = [ box_params.LW; -box_params.LH];
    box_params.Ptl_box = [-box_params.LW;  box_params.LH];
    box_params.Ptr_box = [ box_params.LW;  box_params.LH];
    box_params.boundary_pts = [box_params.Pbl_box, box_params.Pbr_box,box_params.Ptr_box, box_params.Ptl_box, box_params.Pbl_box];


    box_params.Ptl1_world = box_params.Ptl_box + [-box_params.LG;0];
    box_params.Ptl2_world = box_params.Ptl_box + [0;box_params.LG];
    box_params.Ptr1_world = box_params.Ptr_box + [0;(box_params.l0)];
    box_params.Ptr2_world = box_params.Ptr_box + [box_params.l0;0];
    
    box_params.Pbl1_world = box_params.Pbl_box + [-box_params.LG;-box_params.LG];
    box_params.Pbl2_world = box_params.Pbl_box + [box_params.LG;-box_params.LG];
    box_params.Pbr1_world = box_params.Pbr_box + [0;(-box_params.l0)];
    box_params.Pbr2_world = box_params.Pbr_box + [box_params.l0;0];

    box_params.P_world = [box_params.Ptl1_world,box_params.Ptl2_world,box_params.Ptr1_world,box_params.Ptr2_world, ...
        box_params.Pbr1_world,box_params.Pbr2_world, box_params.Pbl1_world,box_params.Pbl2_world];

    box_params.P_box = [box_params.Ptl_box,box_params.Ptl_box,box_params.Ptr_box,box_params.Ptr_box, ...
        box_params.Pbr_box,box_params.Pbr_box, box_params.Pbl_box,box_params.Pbl_box];


    x0 = 0;
    y0 = 0;
    theta0 = 0.1; 
    vx0 = 0;
    vy0 = 0;
    vtheta0 = 0;

V0 = [x0;y0;theta0;vx0;vy0;vtheta0];
t_in = 0;

my_rate_func = @(V_in) box_rate_func(t_in,V_in,box_params);

solver_params = struct();

[V_eq, exit_flag, num_calls] = multi_newton_solver(my_rate_func,V0,solver_params);

[J_approx, num_calls] = approximate_jacobian(my_rate_func,V0);

my_linear_rate = @(t_in,V_in) J_approx*(V_in-V_eq);

%small number used to scale initial perturbation
epsilon = .1;
V0 = V_eq + epsilon*[x0;y0;theta0;vx0;vy0;vtheta0]
tspan = [0 1];
h_ref = .05;
BT_struct = struct();
BT_struct.C = [0, 1];
BT_struct.B = [1/2, 1/2; 1, 0];
BT_struct.A = [0, 0; 1, 0];
p = 2;
error_desired = .01;

% run the integration of nonlinear system
my_rate_func_2 = @(t_in, V_in) box_rate_func(t_in,V_in,box_params);
[t_list_NL,V_list_NL] = explicit_RK_variable_step_integration(my_rate_func_2,tspan,V0,h_ref,BT_struct,p,error_desired);

x_list_NL = V_list_NL(:,1);
y_list_NL = V_list_NL(:,2);
theta_list_NL = V_list_NL(:,3);


%run the integration of linear system
[t_list_L,V_list_L,h_avg,num_evals, redo_count] = explicit_RK_variable_step_integration ...
(my_linear_rate,tspan,V0,h_ref,BT_struct,p,error_desired);

x_list_L = V_list_L(:,1);
y_list_L = V_list_L(:,2);
theta_list_L = V_list_L(:,3);


figure(1);
subplot(3, 1, 1)
hold on;
plot(t_list_NL,x_list_NL,"r",DisplayName='Non Linear');
plot(t_list_L,x_list_L,"b",DisplayName='Linear');

xlabel('$time\ (-)$', 'Interpreter', 'latex', 'FontSize', 16)
ylabel('$x\ (-)$', 'Interpreter', 'latex', 'FontSize', 17);
title({'Linear v. Non-linear X Values'}, 'Interpreter', 'latex', 'FontSize', 13)
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 10.5);

subplot(3, 1, 2)
hold on;
plot(t_list_NL,y_list_NL,"r");
plot(t_list_L,y_list_L,"b");

xlabel('$time\ (-)$', 'Interpreter', 'latex', 'FontSize', 16)
ylabel('$y\ (-)$', 'Interpreter', 'latex', 'FontSize', 17);
title({'Linear v. Non-linear Y Values'}, 'Interpreter', 'latex', 'FontSize', 13)
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 10.5);


subplot(3, 1, 3)
hold on;
plot(t_list_NL,theta_list_NL,"r");
plot(t_list_L,theta_list_L,"b");

xlabel('$time\ (-)$', 'Interpreter', 'latex', 'FontSize', 16)
ylabel('$theta\ (-)$', 'Interpreter', 'latex', 'FontSize', 17);
title({'Linear v. Non-linear Theta Values'}, 'Interpreter', 'latex', 'FontSize', 13)
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 10.5);




