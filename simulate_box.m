function simulate_box()



    %define system parameters
    box_params = struct();
    box_params.m = 3; % kg
    box_params.I = 5; % kg*m^2
    box_params.g = 9.81; % m/s^2

    %define per spring
    box_params.spring_number = 8;
    box_params.LG = 1.5; %m this is the length of spring compressed by gravity
    box_params.l0 = box_params.LG * 1.5;
    box_params.k_list = 50*(ones(1,box_params.spring_number)); % N/m
    box_params.l0_list = box_params.l0(ones(1,box_params.spring_number)); % m

    %define box parameters
    box_params.LW = 10; %m
    box_params.LH = 10; %m

    %define box boundaries
    box_params.Pbl_box = [-box_params.LW; -box_params.LH];
    box_params.Pbr_box = [ box_params.LW; -box_params.LH];
    box_params.Ptl_box = [-box_params.LW;  box_params.LH];
    box_params.Ptr_box = [ box_params.LW;  box_params.LH];
    box_params.boundary_pts = [box_params.Pbl_box, box_params.Pbr_box,box_params.Ptr_box, box_params.Ptl_box, box_params.Pbl_box];


    box_params.Ptl1_world = box_params.Ptl_box + [-box_params.LG;-box_params.LG];
    box_params.Ptl2_world = box_params.Ptl_box + [box_params.LG;-box_params.LG];
    box_params.Ptr1_world = box_params.Ptr_box + [0;(-box_params.l0)];
    box_params.Ptr2_world = box_params.Ptr_box + [box_params.l0;0];
    
    box_params.Pbl1_world = box_params.Pbl_box + [-box_params.LG;-box_params.LG];
    box_params.Pbl2_world = box_params.Pbl_box + [box_params.LG;-box_params.LG];
    box_params.Pbr1_world = box_params.Pbr_box + [0;(-box_params.l0)];
    box_params.Pbr2_world = box_params.Pbr_box + [box_params.l0;0];

    box_params.P_world = [box_params.Ptl1_world,box_params.Ptl2_world,box_params.Ptr1_world,box_params.Ptr2_world, ...
        box_params.Pbl1_world,box_params.Pbl2_world,box_params.Pbr1_world,box_params.Pbr2_world];

    box_params.P_box = [box_params.Ptl_box,box_params.Ptl_box,box_params.Ptr_box,box_params.Ptr_box, ...
        box_params.Pbl_box,box_params.Pbl_box,box_params.Pbr_box,box_params.Pbr_box];


    %load the system parameters into the rate function
    %via an anonymous function
    my_rate_func = @(t_in,V_in) box_rate_func(t_in,V_in,box_params);
    x0 = 0;
    y0 = 0;
    theta0 = 0.1; 
    vx0 = 0;
    vy0 = 0;
    vtheta0 = 0;
    
    V0 = [x0;y0;theta0;vx0;vy0;vtheta0];
    
    
    
    
    
    tspan = [0 10];
    h_ref = .05;
    BT_struct = struct();
    BT_struct.C = [0, 1];
    BT_struct.B = [1/2, 1/2; 1, 0];
    BT_struct.A = [0, 0; 1, 0];
    p = 2;
    error_desired = .01;
    
    
    [tlist,Vlist] = explicit_RK_variable_step_integration(my_rate_func,tspan,V0,h_ref,BT_struct,p,error_desired);
    % figure;
    % plot(tlist,Vlist(:,2), 'ro')
    % figure;
    % plot(tlist,Vlist(:,1), 'bo')
    
    
    num_zigs = 5;
    w = .1;
    hold on;
    spring_plot_struct_list = {};
    for i = 1:box_params.spring_number
        spring_plot_struct_list{i} = initialize_spring_plot(num_zigs,w);
    end
    
    box_plot_struct = initialize_box_plot();
    % axis equal; axis square;
    axis([-50,50,-50,50]);
    P1_list = box_params.P_box;
    P2_list = box_params.P_world;
    
    for i = 1:length(Vlist)
            
            x = Vlist(i, 1);
            y = Vlist(i, 2);
            theta = Vlist(i, 3);
            Plist_box = zeros(2, 4);

            Plist_box(1, :)  = [x-(box_params.LW/2), x-(box_params.LW/2), x+(box_params.LW/2), x+(box_params.LW/2)];
            Plist_box(2, :) = [y+(box_params.LH/2), y-(box_params.LH/2), y-(box_params.LH/2), y+(box_params.LH/2)];
            Plist_world = compute_rbt(x,y,theta,Plist_box);
           
            box_coords = struct();
            box_coords.x = Plist_world(1, :);
            box_coords.y = Plist_world(2, :);
            update_box_plot(box_plot_struct, box_coords);
            
            for j = 1:box_params.spring_number
                update_spring_plot(spring_plot_struct_list{j}, P1_list(:, j) ,P2_list(:,j))
            end
drawnow;
    end

    

end


%updates spring plotting object so that spring is plotted
%with ends located at points P1 and P2
function update_spring_plot(spring_plot_struct,P1,P2)
dP = P2-P1;
R = [dP(1),-dP(2)/norm(dP);dP(2),dP(1)/norm(dP)];
plot_pts = R*spring_plot_struct.zig_zag;
set(spring_plot_struct.line_plot,'xdata',plot_pts(1,:)+P1(1),'ydata',plot_pts(2,:)+P1(2));
set(spring_plot_struct.point_plot,'xdata',[P1(1),P2(1)],'ydata',[P1(2),P2(2)]);
end
%create a struct containing plotting info for a single spring
%INPUTS:
%num_zigs: number of zig zags in spring drawing
%w: width of the spring drawing
function spring_plot_struct = initialize_spring_plot(num_zigs,w)
spring_plot_struct = struct();
zig_ending = [.25,.75,1; ...
-1,1,0];
zig_zag = zeros(2,3+3*num_zigs);
zig_zag(:,1) = [-.5;0];
zig_zag(:,end) = [num_zigs+.5;0];
for n = 0:(num_zigs-1)
zig_zag(:,(3+3*n):2+3*(n+1)) = zig_ending + [n,n,n;0,0,0];
end

zig_zag(1,:)=(zig_zag(1,:)-zig_zag(1,1))/(zig_zag(1,end)-zig_zag(1,1));
zig_zag(2,:)=zig_zag(2,:)*w;
spring_plot_struct.zig_zag = zig_zag;
spring_plot_struct.line_plot = plot(0,0,'k','linewidth',2);
spring_plot_struct.point_plot = plot(0,0,'ro','markerfacecolor','r','markersize',7);
end