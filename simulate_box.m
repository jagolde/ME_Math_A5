function simulate_box()
%define system parameters
box_params = struct();
box_params.m = %your code here
box_params.I = %your code here
box_params.g = %your code here
box_params.k_list = %your code here
box_params.l0_list = %your code here
box_params.P_world = %your code here
box_params.P_box = %your code here
%load the system parameters into the rate function
%via an anonymous function
my_rate_func = @(t_in,V_in) box_rate_func(t_in,V_in,box_params);
x0 = %your code here
y0 = %your code here
theta0 = %your code here
vx0 = %your code here
vy0 = %your code here
vtheta0 = %your code here

V0 = [x0;y0;theta0;vx0;vy0;vtheta0];
tspan = %your code here
%run the integration
% [tlist,Vlist] = your_integrator(my_rate_func,tspan,V0,...)


end

LW = 10; 
LH = 1; 
LG = 3;
m = 1; Ic = (1/12)*(LHˆ2+LWˆ2);
g = 1; k = 20; k_list = [.5*k,.5*k,2*k,5*k];
l0 = 1.5*LG;
Pbl_box = [-LW;-LH]/2;
Pbr_box = [LW;-LH]/2;
Ptl_box = [-LW;LH]/2;
Ptr_box = [LW;LH]/2;
boundary_pts = [Pbl_box,Pbr_box,Ptr_box,Ptl_box,Pbl_box];
Pbl1_world = Pbl_box + [-LG;-LG];
Pbl2_world = Pbl_box + [LG;-LG];
Pbr1_world = Pbr_box + [0;-l0];
Pbr2_world = Pbr_box + [l0;0];
P_world = [Pbl1_world,Pbl2_world,Pbr1_world,Pbr2_world];
P_box = [Pbl_box,Pbl_box,Pbr_box,Pbr_box];
%define system parameters
box_params = struct();
box_params.m = m;
box_params.I = Ic;
box_params.g = g;
box_params.k_list = k_list;
box_params.l0_list = l0*ones(size(P_world,2));
box_params.P_world = P_world;
box_params.P_box = P_box;
box_params.boundary_pts = boundary_pts;