function u = f(x, u_max)
    
    K = 50;
    omega_safety = -K*x(5);
    ang_control = sign(omega_safety) * min(abs(omega_safety), u_max(2));
    
    u = [0; ang_control];
   
end