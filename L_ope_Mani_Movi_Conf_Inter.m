function [f] = L_ope_Mani_Movi_Conf_Inter(z,H,Q,hd,h,q,l,ts,N,k,variacion)
%1) Definici�n de variables

    if(k>N)
        delta=variacion(:,k-N:k);
    else
        delta=variacion;
    end

    delta_final=[delta(1,:),delta(2,:)];
    %a) Acciones de control (incognita)        
     v = z;       %z=[u,w,q1_p,q2_p,q3_p,q4_p]';

    %b) Estados de control
     x = q(1);
     y = q(2);
    th = q(3);
    q1 = q(4);
    q2 = q(5);
    q3 = q(6);
    q4 = q(7);
    
    %c) Par�mtros del Manipulador M�vil
     a = l(1);
    ha = l(2);
    l2 = l(3);
    l3 = l(4);
    l4 = l(5);
    
    L = [Fun(H,Q,hd(:,k),h(:,1))];
for i=1:1:N
%2) Matriz Jacobiana
    j11 = cos(th(i));
    j12 = -sin(q1(i)+th(i))*(l3*cos(q2(i)+q3(i))+l2*cos(q2(i))+l4*cos(q2(i)+q3(i)+q4(i)))-a*sin(th(i));
    j13 = -sin(q1(i)+th(i))*(l3*cos(q2(i)+q3(i))+l2*cos(q2(i))+l4*cos(q2(i)+q3(i)+q4(i)));
    j14 = -cos(q1(i)+th(i))*(l3*sin(q2(i)+q3(i))+l2*sin(q2(i))+l4*sin(q2(i)+q3(i)+q4(i)));
    j15 = -cos(q1(i)+th(i))*(l3*sin(q2(i)+q3(i))+l4*sin(q2(i)+q3(i)+q4(i)));
    j16 = -l4*sin(q2(i)+q3(i)+q4(i))*cos(q1(i)+th(i));
        
    j21 = sin(th(i));
    j22 = cos(q1(i)+th(i))*(l3*cos(q2(i)+q3(i))+l2*cos(q2(i))+l4*cos(q2(i)+q3(i)+q4(i)))+a*cos(th(i));
    j23 = cos(q1(i)+th(i))*(l3*cos(q2(i)+q3(i))+l2*cos(q2(i))+l4*cos(q2(i)+q3(i)+q4(i)));
    j24 = -sin(q1(i)+th(i))*(l3*sin(q2(i)+q3(i))+l2*sin(q2(i))+l4*sin(q2(i)+q3(i)+q4(i)));
    j25 = -sin(q1(i)+th(i))*(l3*sin(q2(i)+q3(i))+l4*sin(q2(i)+q3(i)+q4(i)));
    j26 = -l4*sin(q2(i)+q3(i)+q4(i))*sin(q1(i)+th(i));
        
    j31 = 0;
    j32 = 0;
    j33 = 0;
    j34 = l3*cos(q2(i)+q3(i))+l2*cos(q2(i))+l4*cos(q2(i)+q3(i)+q4(i));
    j35 = l3*cos(q2(i)+q3(i))+l4*cos(q2(i)+q3(i)+q4(i));
    j36 = l4*cos(q2(i)+q3(i)+q4(i));
     
    J = [j11 j12 j13 j14 j15 j16; 
         j21 j22 j23 j24 j25 j26; 
         j31 j32 j33 j34 j35 j36;
         0    0   1   0   0    0;
         0    0   0   1   0    0;
         0    0   0   0   1    0;
         0    0   0   0   0    1;];
    
%2) Posiciones deseados k+1    
    h(:,i+1)=ts*J*v+h(:,i);
    th(i+1) = th(i)+v(2)*ts;
    q1(i+1) = q1(i)+h(4,i+1);
    q2(i+1) = q2(i)+h(5,i+1);
    q3(i+1) = q3(i)+h(6,i+1);
    q4(i+1) = q4(i)+h(7,i+1);
    
    L=[L;(Fun(H,Q,hd(:,k+i),h(:,i+1)))];
    
end
%3) Integraci�n del Funcional 
R=1*eye(length(delta_final));
f=0.5*sum(L)+delta_final*R*delta_final';
end
function [F] = Fun(H,Q,hd,h)
    alpha = 1;    %Peso de errores de posici�n
    beta = 0.01;  %Peso de errores de configuraci�n del brazo
    he = hd-h;    %Errores de control
    F = alpha*he(1:3)'*H*he(1:3)+beta*he(4:7)'*Q*he(4:7);  
end

