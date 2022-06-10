%**************************************************************************
%***************************CONTROL ÓPTIMO*********************************
%**************Manipulador Móvil + Configuración Interna*******************
%**************************************************************************
clear all; close all; clc;
ts=0.1; tf=120;
t=[0:ts:tf];

%1) Condiciones Iniciales del Manipulador Móvil
    %a)Posiones iniciales
       x(1) = 1; % PLATAFORMA MÓVIL
       y(1) = 1;
       th(1)= 0*pi/180; 
       q1(1)= 40*pi/180; %BRAZO 4DOF2
       q2(1)= 45*pi/180;
       q3(1)=-45*pi/180;
       q4(1)= 60*pi/180;

    %b)Velocidades iniciales 
       u(1) = 0;
       w(1) = 0;
       q1p(1)= 0;
       q2p(1)= 0; 
       q3p(1)= 0;
       q4p(1)= 0;
       
    %c)Vector de inicialización para el optimizador
       z0 = [u(1),w(1),q1p(1),q2p(1),q3p(1),q4p(1)]';  

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
 
%4) Configuración Deseada del BRAZO         
    q1d =  0*pi/180*ones(1,length(t))
    q2d = 60*pi/180*ones(1,length(t))
    q3d =-40*pi/180*ones(1,length(t))
    q4d = 0*pi/180*ones(1,length(t))

%5) Velocidades máximas
    lb = [-2.5,-2.5,-1,-1,-1,-1]';
    ub = [ 2.5, 2.5, 1, 1, 1, 1]';

%6) Posición Inicial del Extremo Operativo   
    hx(1) = x(1)+a*cos(th(1))+cos(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
    hy(1) = y(1)+a*sin(th(1))+sin(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
    hz(1) = ha+l2*sin(q2(1))+l3*sin(q2(1)+q3(1))+l4*sin(q2(1)+q3(1)+q4(1));
     
%7) Configuración del Optimizador  
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
         
%2) VECTORES
    %a)Vector de estados
       q = [x(k),y(k),th(k),q1(k),q2(k),q3(k),q4(k)]';

    %b)Vectores del estremo operativo
       h   = [hx(k),   hy(k),    hz(k),    q1(k),    q2(k),    q3(k),     q4(k)]';
       hd  = [hxd(k),  hyd(k),   hzd(k),   q1d(k),   q2d(k),   q3d(k),    q4d(k)]';
       hd1 = [hxd(k+1),hyd(k+1), hzd(k+1), q1d(k+1), q2d(k+1), q3d(k+1),  q4d(k+1)]';

%3) FUNCIÓN OBJETIVO    
    f_obj = @(z) L_ope_Mani_Movi_Conf_Inter(z,H,Q,hd,hd1,h,q,l,ts);
   
%2) LEY DE CONTROL - OPTIMIZADOR
    qref = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
    uref_c(k) = qref(1);
    wref_c(k) = qref(2);
    q1pref_c(k)= qref(3);
    q2pref_c(k)= qref(4);
    q3pref_c(k)= qref(5);
    q4p(k)= qref(6);
%3) ERRORES DE VELOCIDAD VREF
    ue(k)=u(k)-uref_c(k);
    we(k)=w(k)-wref_c(k);
    q1pe(k)=q1p(k)-q1pref_c(k);
    q2pe(k)=q2p(k)-q2pref_c(k);
    q3pe(k)=q3p(k)-q3pref_c(k);
    vref_e=[ue(k) we(k) q1pe(k) q2pe(k) q3pe(k)]';
%4) DERIVADAS DE LAS VREF DESEADAS
    vrefp_u=diff([uref_c uref_c(end)])/ts;
    vrefp_w=diff([wref_c wref_c(end)])/ts;
    vrefp_q1p=diff([q1pref_c q1pref_c(end)])/ts;
    vrefp_q2p=diff([q2pref_c q2pref_c(end)])/ts;
    vrefp_q3p=diff([q3pref_c q3pref_c(end)])/ts;

    vrefp=[vrefp_u(k) vrefp_w(k) vrefp_q1p(k) vrefp_q2p(k) vrefp_q3p(k)]';
 %5) PARTE DE DINAMICA
    v=[uref_c(k) wref_c(k) q1pref_c(k) q2pref_c(k) q3pref_c(k)]';
    q=[0 th(k) q1(k) q2(k) q3(k)]';
    Dinamica=COMPENSACION_DINAMICA_3GDL(vrefp,vref_e,v,q,ts);
 %6) VELOCIDADES NUEVAS DE PLATAFOMRA Y BRAZO  
    uref(k) = Dinamica(1);
    wref(k) = Dinamica(2);
    q1pref(k) = Dinamica(3);
    q2pref(k) = Dinamica(4);
    q3pref(k) = Dinamica(5);   
    vref =[uref(k) wref(k) q1pref(k) q2pref(k) q3pref(k)]';
 %7) MODELO DEL ROBOT AKASHA
    akasha = AKASHA_DINAMICA(vref,v,q,ts);
 %8) VELOCIDADES DEL ROBOT
   u(k+1)=akasha(1);
   w(k+1)=akasha(2);
   q1p(k+1)=akasha(3);
   q2p(k+1)=akasha(4);
   q3p(k+1)=akasha(5);
   
 %9) POSICIONES DEL ROBOT  
   th(k+1)=akasha(7);
   q1(k+1)=akasha(8);
   q2(k+1)=akasha(9);
   q3(k+1)=akasha(10);
   q4(k+1)=ts*q4p(k)+q4(k);
   
    %4) ROBOT MANIPULADOT MÓVIL
    %a)Plataforma Móvil
       xp = u(k+1)*cos(th(k+1))-a*w(k+1)*sin(th(k+1)); % cinemática
       yp = u(k+1)*sin(th(k+1))+a*w(k+1)*cos(th(k+1));
    
       x(k+1) =  ts*xp+x(k);   %euler móvil
       y(k+1) =  ts*yp+y(k);   
       th(k+1)=  ts*w(k)+th(k);


    
    %c)Posición del estremo operativo en K+1
       hx(k+1) = x(k+1)+a*cos(th(k+1))+cos(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
                 l4*cos( q2(k+1)+q3(k+1)+q4(k+1)));
       hy(k+1) = y(k+1)+a*sin(th(k+1))+sin(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
                 l4*cos(q2(k+1)+q3(k+1)+q4(k+1)));
       hz(k+1) = ha+l2*sin(q2(k+1))+l3*sin(q2(k+1)+q3(k+1))+...
                 l4*sin(q2(k+1)+q3(k+1)+q4(k+1)); 

    %d)Vector de inicialización para el optimizador             
       z0 = [u(k+1),w(k+1),q1p(k+1),q2p(k+1),q3p(k+1),q4p(k+1)]';  

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
figure
t=[0:ts:tf-ts];
subplot(2,1,1)
    plot(t,hxd-hx,'r'); hold on;
    plot(t,hyd-hy,'g'); hold on;
    plot(t,hzd-hz,'b');hold on;
    grid on
    legend('Error en hxe','Error en hye','Error en hze');
    title('Errores de Posicion')
    xlabel('Tiempo [s]'); ylabel('Error [m]'); 
subplot(2,1,2)
    plot(t,ue,'r'); hold on
    plot(t,we,'k'); hold on
    plot(t,q1pe,'g'); hold on
    plot(t,q2pe,'b'); hold on
    plot(t,q3pe,'y'); grid
    legend('Error ue','Error de we','Error de q1pe','Error de q2pe','Error de q3pe')
    title('Errores de Velocidad')
    xlabel('Tiempo [s]'); ylabel('Error [rad/s]');
figure
subplot(2,1,1)

tr=[0:ts:tf+ts];
    plot(tr,u,'r'); hold on
    plot(tr,w,'k'); hold on
    plot(tr,q1p,'g'); hold on
    plot(tr,q2p,'b'); hold on
    plot(tr,q3p,'y'); hold on
    plot(t,q4p,'m'); grid
    legend('Velocidad de u','Velocidad de w','Velocidad de q1p','Velocidad de q2p','Velocidad de q3p','Velocidad de q4p')
    title('Velocidades del Robot')
    xlabel('Tiempo [s]'); ylabel('[rad/s]');

subplot(2,1,2)
    plot(t,uref_c,'r'); hold on
    plot(t,wref_c,'k'); hold on
    plot(t,q1pref_c,'g'); hold on
    plot(t,q2pref_c,'b'); hold on
    plot(t,q3pref_c,'y'); grid
    legend('Velocidad uref_c','Velocidad wref_c','Velocidad q1pref_c','Velocidad q2pref_c','Velocidad q3pref_c')
    title('Velocidades de Control Cinemático')
    xlabel('Tiempo [s]'); ylabel('[rad/s]');
%% graficas de errores de configuracion
figure
subplot(4,1,1)
plot(t,q1d-q1,'g'); hold on; 
grid on
legend('e_q1'); 
xlabel(''),ylabel('Err q1[m]')
subplot(4,1,2)
plot(t,q2d-q2,'b'); hold on;
grid on
legend('e_q2');
xlabel(''),ylabel('Err q2[m]')
subplot(4,1,3)
plot(t,q3d-q3,'m'); hold on; 
grid on;
legend('e_q3');
xlabel(''),ylabel('Err q3[m]')
subplot(4,1,4)
plot(t,q4d-q4,'k');grid
grid on;
legend('e_q4');
xlabel('Time [s]'),ylabel('Err q4[m]')

figure
subplot(2,1,1)
    plot(t(1:length(t)-1),u,'g'); hold on; 
    legend('u');
    grid on
    %title ('Acciones de control PLATAFORMA') 
    xlabel(''),ylabel('[m/s]')

subplot(2,1,2)
    plot(t(1:length(t)-1),w,'b'); grid; 
    legend('w'); 
    grid on
    %title ('Acciones de control PLATAFORMA') 
    xlabel('Time [s]'),ylabel('[rad/s]')

figure
subplot(4,1,1)
    plot(t(1:length(t)-1),q1(1:length(t)-1),'g'); hold on; 
    grid on
    legend('q1'), 
    xlabel(''),ylabel('[rad]')
subplot(4,1,2)
    plot(t(1:length(t)-1),q2(1:length(t)-1),'b'); hold on; 
     grid on
    legend('q2'), 
    xlabel(''),ylabel('[rad]')
subplot(4,1,3)
   plot(t(1:length(t)-1),q3(1:length(t)-1),'m'); hold on;
    grid on
    legend('q3'), 
    xlabel(''),ylabel('[rad]')
subplot(4,1,4)
 plot(t(1:length(t)-1),q4(1:length(t)-1),'k');grid
  grid on
    legend('q4'), 
    xlabel('Time [s]'),ylabel('[rad]')
figure
plot(t(1:length(cpu)),cpu,'b');