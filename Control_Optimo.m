%***********************************************************
%*******************CONTROL ÓPTIMO**************************
%***********************************************************
clear all; close all; clc;
dt=0.1; tf=60;
t=[0:dt:tf];
nt = length(t);

% Condiciones iniciales
  %Posición
  x(1)  = -2;
  y(1)  = -2;
  psi(1)= -pi;
  a = .1;
  
  %Velocidades
  u(1) = 0;
  w(1) = 0;

% Trayectoria deseada
  %OCHO
  xd = 6*sin(2*t*pi/tf);   xdp = 6*cos(2*t*pi/tf)*2*pi/tf;
  yd = 6*sin(4*t*pi/tf);   ydp = 6*cos(4*t*pi/tf)*4*pi/tf;
  psid = atan2(ydp,xdp);
  
  %Seno en 'X'
%   xd = 0.1*t;             xdp = 0.1*ones(1,length(t));
%   yd = sin(.2*t);         ydp = .2*cos(.2*t);
%   psid = atan2(ydp,xdp);
 
  %RECTA XY
%   xd = 0.1*t;             xdp = 0.1*ones(1,length(t));
%   yd = 0.1*t;             ydp = 0.1*ones(1,length(t));
%   psid = atan2(ydp,xdp);
 
  %Circunferencia
%   xd = 3*cos(0.05*t)+1.75;    xdp = -3*0.05*sin(0.05*t);      
%   yd = 3*sin(0.05*t)+1.75;    ydp =  3*0.05*cos(0.05*t);      
%   psid = atan2(ydp,xdp);
  
% Matriz de Peso
  Q = [1 0 0;
       0 1 0;
       0 0 .01];

% Iterado inicial para el primer intervalo de muestreo
  z0 = [u(1),w(1)]';  

% Velocidades máximas
  lb = [-2.5,-2.5]';
  ub = [ 2.5, 2.5]';
  
  options = optimset('Algorithm','sqp','Display','off');
 
for k=1:length(t)-1
    aux = cputime;  
   
    h   = [x(k),   y(k),   psi(k)]';
    hd  = [xd(k),  yd(k),  psid(k)]';
    hd1 = [xd(k+1),yd(k+1),psid(k+1)]';
    
    f_obj = @(z) L_ope(z,Q,hd,hd1,h,a,dt);
   
%b) Optimizador
    res = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
    u(k) = res(1);
    w(k) = res(2);

%c) Robot móvil
    xp = u(k)*cos(psi(k))-a*w(k)*sin(psi(k)); % cinemática
    yp = u(k)*sin(psi(k))+a*w(k)*cos(psi(k));
    
    x(k+1) =  dt*xp+x(k);   %euler móvil
    y(k+1) =  dt*yp+y(k);   
    psi(k+1)= dt*w(k)+psi(k);
           
    z0 = [u(k),w(k)]';     %Vector de inicialización
    cpu(k) = cputime-aux;  %Tiempo de cálculo
end

%************************************************************************** 
%***************************ANIMACIÓN****************************************
%************************************************************************** 
close all; paso=1; fig=figure;
%a) Parámetros del cuadro de animación
    set(fig,'position',[10 60 980 600]);
    cameratoolbar
    view(40,40); 
    
%b) Dimenciones del Robot
    DimensionesMovil();
    
%c) Dibujo del Robot    
    G1=Movil3D(x(1),y(1),psi(1)); hold on
    G2=plot3(xd(1),yd(1),0,'r','linewidth',2); hold on
    G3=plot3(x(1),h(1),0,'g','linewidth',2); hold on
    
axis equal; 
for k = 1:10:length(t)
    drawnow
    delete(G1);
    delete(G2);
    delete(G3);
    
    G1 = Movil3D(x(k),y(k),psi(k)); hold on
    G2 = plot3(xd(1:k),yd(1:k),0*y(1:k),'r','linewidth',2);
    G3 = plot3(x(1:k),y(1:k),0*y(1:k),'g','linewidth',2);
    xlabel('X[m]'), ylabel('Y[m]')
    
%     axis([-1 5 -1 4 0 1]);
end

%************************************************************************** 
%***************************GR[AFICAS****************************************
%************************************************************************** 
figure
plot(xd,yd); hold on; plot(x,y,'r')