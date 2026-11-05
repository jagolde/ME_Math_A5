%test 


x = V(1);
y = V(2);
dxdt = V(4);
dydt = V(5);
dthetadt = V(6);

theta = V(3);
[d2xdt2,d2ydt2,d2thetadt2] = compute_accel(x,y,theta,box_params)
dVdt = [dxdt;dydt;dthetadt;d2xdt2;d2ydt2;d2thetadt2];