%***********************************************************
%*******************CONTROL ÓPTIMO**************************
%***********************************************************
clear all; close all; clc;
dt=0.01; tf=30;
t=[0:dt:tf];
nt = length(t);

% Parámtros del robot
  a  = 0.175;            
  ha = 0.375;
  l2 = 0.275;            
  l3 = 0.275;          
  l4 = 0.15;
  l  = [a,ha,l2,l3,l4]';
  
% Condiciones iniciales
  x(1) = 0; % PLATAFORMA MÓVIL
  y(1) = 0;
  th(1)= 0*pi/180; 
  q1(1)= 40*pi/180; %BRAZO
  q2(1)= 45*pi/180;
  q3(1)=-45*pi/180;
  q4(1)= 60*pi/180;

  %Velocidades
  u(1) = 0;
  w(1) = 0;
  q1_p(1)= 0;
  q2_p(1)= 0; 
  q3_p(1)= 0;
  q4_p(1)= 0;
  
% Trayectoria deseada
  %Silla de montar         
   hxd = 1.5*cos(0.05*t)+1.75;    hxd_p = -1.5*0.05*sin(0.05*t);      
   hyd = 1.5*sin(0.05*t)+1.75;    hyd_p =  1.5*0.05*cos(0.05*t);     
   hzd = 0.15*sin(0.1*t)+0.6;     hzd_p =  0.15*0.1*cos(0.1*t);  
 
% Posición inicial del extremo operativo   
   hx(1) = x(1)+a*cos(th(1))+cos(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
   hy(1) = y(1)+a*sin(th(1))+sin(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
   hz(1) = ha+l2*sin(q2(1))+l3*sin(q2(1)+q3(1))+l4*sin(q2(1)+q3(1)+q4(1));
  
% Matriz de Peso
  Q = [1 0 0;
       0 1 0;
       0 0 1];

% Iterado inicial para el primer intervalo de muestreo
  z0 = [u(1),w(1),q1_p(1),q2_p(1),q3_p(1),q4_p(1)]';  

% Velocidades máximas
  lb = [-2.5,-2.5,-1,-1,-1,-1]';
  ub = [ 2.5, 2.5,1,1,1,1]';
  
  options = optimset('Algorithm','sqp','Display','off');
 tic
for k=1:length(t)-1
    aux = cputime;  
    
    q = [x(k),y(k),th(k),q1(k),q2(k),q3(k),q4(k)]';
   
    h   = [hx(k),   hy(k),   hz(k)]';
    hd  = [hxd(k),  hyd(k),  hzd(k)]';
    hd1 = [hxd(k+1),hyd(k+1),hzd(k+1)]';
    
    f_obj = @(z) L_ope_Mani_Movi(z,Q,hd,hd1,h,q,l,dt);
   
%b) Optimizador
    res = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
    u(k) = res(1);
    w(k) = res(2);
    q1_p(k)= res(3);
    q2_p(k)= res(4);
    q3_p(k)= res(5);
    q4_p(k)= res(6);
    
%c) Robot Manipulador Móvil
    xp = u(k)*cos(th(k))-a*w(k)*sin(th(k)); % cinemática
    yp = u(k)*sin(th(k))+a*w(k)*cos(th(k));
    
    x(k+1) =  dt*xp+x(k);   %euler móvil
    y(k+1) =  dt*yp+y(k);   
    th(k+1)=  dt*w(k)+th(k);

    q1(k+1)= q1(k)+q1_p(k)*dt; %euler brazo
    q2(k+1)= q2(k)+q2_p(k)*dt;
    q3(k+1)= q3(k)+q3_p(k)*dt;   
    q4(k+1)= q4(k)+q4_p(k)*dt;
    
    %)Posición del estremo operativo en K+1
    hx(k+1) = x(k+1)+a*cos(th(k+1))+cos(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
              l4*cos( q2(k+1)+q3(k+1)+q4(k+1)));
    hy(k+1) = y(k+1)+a*sin(th(k+1))+sin(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
              l4*cos(q2(k+1)+q3(k+1)+q4(k+1)));
    hz(k+1) = ha+l2*sin(q2(k+1))+l3*sin(q2(k+1)+q3(k+1))+...
              l4*sin(q2(k+1)+q3(k+1)+q4(k+1)); 

    z0 = [u(k),w(k),q1_p(k),q2_p(k),q3_p(k),q4_p(k)]';  %Vector de inicialización

    cpu(k) = cputime-aux;  %Tiempo de cálculo
end
toc
%************************************************************************** 
%***************************ANIMACIÓN****************************************
%% ************************************************************************** 
close all; paso=1; fig=figure;

%a) Parámetros del cuadro de animación
    set(fig,'position',[10 60 980 600]);
    cameratoolbar
    view(40,40); 
    
%b) Dimenciones del Robot
    DimensionesMovil();
    DimensionesManipulador(a,ha);

%c) Dibujo del Robot    
    G1=Movil3D(x(1),y(1),th(1));
    G2=Manipulador3D(x(1),y(1),th(1),q1(1),q2(1),q3(1),q4(1));

    G3=plot3(hx(1),hy(1),hz(1),'b','linewidth',2);hold on,grid on   
    G4=plot3(hxd(1),hyd(1),hzd(1),'r','linewidth',2);
    
axis equal; 
for k = 1:10:length(t)
    drawnow
    delete(G1);
    delete(G2);
    delete(G3);
    delete(G4);

    G1 = Movil3D(x(k),y(k),th(k));
    G2 = Manipulador3D(x(k),y(k),th(k),q1(k),q2(k),q3(k),q4(k));
    G3 = plot3(hx(1:k),hy(1:k),hz(1:k),'g','linewidth',2);
    G4 = plot3(hxd(1:k),hyd(1:k),hzd(1:k),'r','linewidth',2);
    xlabel('X[m]'), ylabel('Y[m]'), zlabel('Z[m]')
    
%     axis([-1 5 -1 4 0 1]);
end

%************************************************************************** 
%***************************GRÁFICAS****************************************
%% ************************************************************************** 


