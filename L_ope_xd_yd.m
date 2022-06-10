function [L] = L_ope_xd_yd(z,Q,hd,hd1,h,psi,psid,a,dt)
u = z(1);
w = z(2);

J = [cos(psi) -a*sin(psi); 
     sin(psi)  a*cos(psi)];
     
v = [u,w]';
hk1 = dt*J*v+h;

L = (Fun(Q,hd,h,psi,psid)+Fun(Q,hd1,hk1,psi,psid))/2; %Trapecio
end


function [F] = Fun(Q,hd,h,psi,psid)
he = hd-h;
F = he'*Q*he+0*(psid-psi);
end
