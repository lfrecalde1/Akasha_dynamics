function [L] = L_ope_Mani_Movi(z,Q,hd,hd1,h,q,l,dt)
   u = z(1);
   w = z(2);
q1_p = z(3);
q2_p = z(4);
q3_p = z(5);
q4_p = z(6);


 x = q(1);
 y = q(2);
th = q(3);
q1 = q(4);
q2 = q(5);
q3 = q(6);
q4 = q(7);

 a = l(1);
ha = l(2);
l2 = l(3);
l3 = l(4);
l4 = l(5);

%b)Matriz Jacobiana
    j11 = cos(th);
    j12 = -sin(q1+th)*(l3*cos(q2+q3)+l2*cos(q2)+l4*cos(q2+q3+q4))-a*sin(th);
    j13 = -sin(q1+th)*(l3*cos(q2+q3)+l2*cos(q2)+l4*cos(q2+q3+q4));
    j14 = -cos(q1+th)*(l3*sin(q2+q3)+l2*sin(q2)+l4*sin(q2+q3+q4));
    j15 = -cos(q1+th)*(l3*sin(q2+q3)+l4*sin(q2+q3+q4));
    j16 = -l4*sin(q2+q3+q4)*cos(q1+th);
        
    j21 = sin(th);
    j22 = cos(q1+th)*(l3*cos(q2+q3)+l2*cos(q2)+l4*cos(q2+q3+q4))+a*cos(th);
    j23 = cos(q1+th)*(l3*cos(q2+q3)+l2*cos(q2)+l4*cos(q2+q3+q4));
    j24 = -sin(q1+th)*(l3*sin(q2+q3)+l2*sin(q2)+l4*sin(q2+q3+q4));
    j25 = -sin(q1+th)*(l3*sin(q2+q3)+l4*sin(q2+q3+q4));
    j26 = -l4*sin(q2+q3+q4)*sin(q1+th);
        
    j31 = 0;
    j32 = 0;
    j33 = 0;
    j34 = l3*cos(q2+q3)+l2*cos(q2)+l4*cos(q2+q3+q4);
    j35 = l3*cos(q2+q3)+l4*cos(q2+q3+q4);
    j36 = l4*cos(q2+q3+q4);
     
    J = [j11 j12 j13 j14 j15 j16; 
         j21 j22 j23 j24 j25 j26; 
         j31 j32 j33 j34 j35 j36];
    
    v = [u,w,q1_p,q2_p,q3_p,q4_p]';
  hk1 = dt*J*v+h; 

    L = (Fun(Q,hd,h,q)+Fun(Q,hd1,hk1,q))/2; %Trapecio
end


function [F] = Fun(Q,hd,h,q)
    he = hd-h;
    F = he'*Q*he; 
end

