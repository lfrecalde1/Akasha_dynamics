function [L] = L_ope(z,Q,hd,hd1,h,a,dt)
u = z(1);
w = z(2);
psi=ver180(h(3));

J = [cos(psi) -a*sin(psi); 
     sin(psi)  a*cos(psi);
         0       1];
     
v = [u,w]';
hk1 = dt*J*v+h;

L = (Fun(Q,hd,h)+Fun(Q,hd1,hk1))/2; %Trapecio
end


function [F] = Fun(Q,hd,h)
he = hd-h;
he(3)=ver180(he(3));
F = he'*Q*he;
end
