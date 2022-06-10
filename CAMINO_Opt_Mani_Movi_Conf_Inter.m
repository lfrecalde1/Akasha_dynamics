%**************************************************************************
%***************************CONTROL ÓPTIMO*********************************
%**************Manipulador Móvil + Configuración Interna*******************
%**************************************************************************
clear all; close all; clc;
ts=0.1; tf=160;
t=[0:ts:tf];

%1) Condiciones Iniciales del Manipulador Móvil
    %a)Posiones iniciales
       x(1) = 5.25; % PLATAFORMA MÓVIL
       y(1) = 0;
       th(1)= 90*pi/180; 
       q1(1)= 40*pi/180; %BRAZO 4DOF
       q2(1)= 45*pi/180;
       q3(1)=-45*pi/180;
       q4(1)= 60*pi/180;

    %b)Velocidades iniciales
       hx_p(1) = 0;
       hy_p(1) = 0;
       hz_p(1) = 0;
       
       u(1) = 0;
       w(1) = 0;
       q1_p(1)= 0;
       q2_p(1)= 0; 
       q3_p(1)= 0;
       q4_p(1)= 0;
       
    %c)Vector de inicialización para el optimizador
       z0 = [u(1),w(1),q1_p(1),q2_p(1),q3_p(1),q4_p(1)]';  

%2) Parámtros del Manipulador Móvil
    a  = 0.175;            
    ha = 0.375;
    l2 = 0.275;            
    l3 = 0.275;          
    l4 = 0.15;
    l  = [a,ha,l2,l3,l4]';
  
%3) Trayectoria Deseada del Manipulador Móvil
    %a)Silla de montar         
       hxd = 3.5*cos(0.05*t)+1.75;    hxd_p = -3.5*0.05*sin(0.05*t);      
       hyd = 3.5*sin(0.05*t)+1.75;    hyd_p =  3.5*0.05*cos(0.05*t);     
       hzd = 0.15*sin(0.1*t)+0.6;     hzd_p =  0.15*0.1*cos(0.1*t);  
 
       hpd_yx  = atan2(hyd_p,hxd_p);
       hpd_xyz = atan2(hzd_p,sqrt(hxd_p.^2+hyd_p.^2));

%4) Velocidad Deseada del Manipulador Móvil
       vd = .09;
       vd_x = 1*vd.*cos(hpd_xyz).*cos(hpd_yx);
       vd_y = 1*vd.*cos(hpd_xyz).*sin(hpd_yx);
       vd_z = 1*vd.*sin(hpd_xyz);
       VDD = sqrt(vd_x.^2+vd_y.^2+vd_z.^2);
        
%5) Configuración Deseada del BRAZO         
    q1d =  0*pi/180*ones(1,length(t));
    q2d = 60*pi/180*ones(1,length(t));
    q3d =-40*pi/180*ones(1,length(t));
    q4d =  0*pi/180*ones(1,length(t));

%6) Velocidades máximas
    lb = [-1.5,-1.5,-.5,-.5,-.5,-.5]';
    ub = [ 1.5, 1.5, .5, .5, .5, .5]';

%7) Posición Inicial del Extremo Operativo   
    hx(1) = x(1)+a*cos(th(1))+cos(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
    hy(1) = y(1)+a*sin(th(1))+sin(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
    hz(1) = ha+l2*sin(q2(1))+l3*sin(q2(1)+q3(1))+l4*sin(q2(1)+q3(1)+q4(1));
     
%8) Configuración del Optimizador  
    options = optimset('Algorithm','sqp','Display','off');

tic
for k=1:length(t)-1
aux = cputime;  

%1) MATRICES DE PESO
    %a)Errores de Posición del MANIPULADOR MÓVIL
       H = [1  0  0;
            0  1  0;
            0  0  1];
   
    %b)Configuración del BRAZO ROBÓTICO
       Q = [.5  0  0  0;
             0  1  0  0;
             0  0  1  0;
             0  0  0  1];

    %a)Errores de Velocidad del MANIPULADOR MÓVIL
       V = [1  0  0;
            0  1  0;
            0  0  1];
         
%2) VECTORES

% a) Busqueda del punto más cercano del camino
         mini = 100;    
         for i = 1:length(hxd)-1
             aux = sqrt((hx(k)-hxd(i))^2+(hy(k)-hyd(i))^2+(hz(k)-hzd(i))^2);
             if aux < mini 
             mini = aux;    
             pos = i;
             end
         end

    hx_e(k) = hxd(pos)-hx(k);
    hy_e(k) = hyd(pos)-hy(k);
    hz_e(k) = hzd(pos)-hz(k);
    rho(k) = sqrt(hx_e(k)^2+hy_e(k)^2+hz_e(k)^2);
    
    %b)Vector de estados
       q = [x(k),y(k),th(k),q1(k),q2(k),q3(k),q4(k)]';

    %c)Vectores del estremo operativo
       h   = [hx(k),     hy(k),      hz(k),      q1(k),    q2(k),    q3(k),     q4(k)]';
       hd  = [hxd(pos),  hyd(pos),   hzd(pos),   q1d(k),   q2d(k),   q3d(k),    q4d(k)]';
       hd1 = [hxd(pos+1),hyd(pos+1), hzd(pos+1), q1d(k+1), q2d(k+1), q3d(k+1),  q4d(k+1)]';

       hp   = [hx_p(k),     hy_p(k),      hz_p(k)]';
       hpd  = [vd_x(pos),   vd_y(pos),    vd_z(pos)]';
       hpd1 = [vd_x(pos+1), vd_y(pos+1),  vd_z(pos+1)]';

%3) FUNCIÓN OBJETIVO    
    f_obj = @(z) L_CAMINO_Mani_Movi_Conf_Inter(z,H,Q,V,hd,hd1,h,q,hp,hpd,hpd1,l,ts);
   
%2) LEY DE CONTROL - OPTIMIZADOR
    res = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
    u(k) = res(1);
    w(k) = res(2);
    q1_p(k)= res(3);
    q2_p(k)= res(4);
    q3_p(k)= res(5);
    q4_p(k)= res(6);
    
%3) ROBOT MANIPULADOT MÓVIL
    %a)Plataforma Móvil
       xp = u(k)*cos(th(k))-a*w(k)*sin(th(k)); % cinemática
       yp = u(k)*sin(th(k))+a*w(k)*cos(th(k));
    
       x(k+1) =  ts*xp+x(k);   %euler móvil
       y(k+1) =  ts*yp+y(k);   
       th(k+1)=  ts*w(k)+th(k);

    %b)Brazo Robótico
       q1(k+1)= q1(k)+q1_p(k)*ts; %euler brazo
       q2(k+1)= q2(k)+q2_p(k)*ts;
       q3(k+1)= q3(k)+q3_p(k)*ts;   
       q4(k+1)= q4(k)+q4_p(k)*ts;
    
    %c)Posición del estremo operativo en K+1
       hx(k+1) = x(k+1)+a*cos(th(k+1))+cos(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
                 l4*cos( q2(k+1)+q3(k+1)+q4(k+1)));
       hy(k+1) = y(k+1)+a*sin(th(k+1))+sin(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
                 l4*cos(q2(k+1)+q3(k+1)+q4(k+1)));
       hz(k+1) = ha+l2*sin(q2(k+1))+l3*sin(q2(k+1)+q3(k+1))+...
                 l4*sin(q2(k+1)+q3(k+1)+q4(k+1)); 

    %d)Vector de inicialización para el optimizador             
       z0 = [u(k),w(k),q1_p(k),q2_p(k),q3_p(k),q4_p(k)]';  

       qk1=[x(k+1),y(k+1),th(k+1),q1(k+1),q2(k+1),q3(k+1),q4(k+1)]';
       hp = Jacobiano(q,l)*res;
       hx_p(k+1) = hp(1);
       hy_p(k+1) = hp(2);
       hz_p(k+1) = hp(3);
              
       hp_EE(k) = sqrt(hx_p(k)^2+hy_p(k)^2+hz_p(k)^2);
       
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
for k = 1:10:length(t)-1
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
figure
subplot(3,1,1)
    plot(t(1:length(t)-1),hx_e,'g'); hold on; 
    plot(t(1:length(t)-1),hy_e,'b'); hold on; 
    plot(t(1:length(t)-1),hz_e,'y'); hold on;
    plot(t(1:length(t)-1),rho,'r'); grid  
    legend('e_X','e_Y','e_Z','\rho'), 
    title ('Errores del Extremo Operativo') 
    xlabel('Tiempo [s]'),ylabel('[m]')

subplot(3,1,2)
    plot(t,q1d-q1,'g'); hold on; 
    plot(t,q2d-q2,'b'); hold on; 
    plot(t,q3d-q3,'m'); hold on; 
    plot(t,q4d-q4,'k');grid
    legend('e_q_1','e_q_2','e_q_3','e_q_4'), 
    title ('Errores Configuración Deseada')
    xlabel('Tiempo [s]'),ylabel('[rad]')

subplot(3,1,3)
    plot(t(1:length(t)-1),cpu,'b'); hold on; 
    plot(t,ts*ones(1,length(t)),'g'); grid 
    legend('T_C_P_U','T_o'), 
    title ('Tiempo de CPU')
    xlabel('Tiempo [s]'),ylabel('[s]')

figure
subplot(3,1,1)
    plot(t(1:length(t)-1),u,'g'); hold on; 
    plot(t(1:length(t)-1),w,'b'); grid; 
    legend('u','w'), 
    title ('Acciones de control PLATAFORMA') 
    xlabel('Tiempo [s]'),ylabel('[m/s] ; [rad/s]')

subplot(3,1,2)
    plot(t(1:length(t)-1),q1_p,'g'); hold on; 
    plot(t(1:length(t)-1),q2_p,'b'); hold on; 
    plot(t(1:length(t)-1),q3_p,'m'); hold on; 
    plot(t(1:length(t)-1),q4_p,'k');grid
    legend('q1_p','q2_p','q3_p','q4_p'), 
    title ('Acciones de control BRAZO')
    xlabel('Tiempo [s]'),ylabel('[rad/s]')

subplot(3,1,3)
    plot(t(1:length(t)-1),hp_EE,'b'); hold on; 
    plot(t,vd*ones(1,length(t)),'g'); grid 
    legend('V_h','V_d'), 
    title ('Velocidad del Extremo Operativo')
    xlabel('Tiempo [s]'),ylabel('[m/s]')
