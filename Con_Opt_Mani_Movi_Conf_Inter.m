%**************************************************************************
%***************************CONTROL �PTIMO*********************************
%**************Manipulador M�vil + Configuraci�n Interna*******************
%**************************************************************************
clear all; close all; clc;
ts=0.1; tf=40;
t=[0:ts:tf];

%1) Condiciones Iniciales del Manipulador M�vil
    %a)Posiones iniciales
       x(1) = 4; % PLATAFORMA M�VIL
       y(1) = 2;
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
       cambio=zeros(5,1);
       
    %c)Vector de inicializaci�n para el optimizador
       z0 = [u(1),w(1),q1p(1),q2p(1),q3p(1),q4p(1)]';  
       variacion=z0;
%2) Par�mtros del Manipulador M�vil
    a  = 0.175;            
    ha = 0.375;
    l2 = 0.275;            
    l3 = 0.275;          
    l4 = 0.15;
    l  = [a,ha,l2,l3,l4]';
  
%3) Trayectoria Deseada del Manipulador M�vil
    %a)Silla de montar         
       hxd = 3.5*cos(0.05*t)+1.75;    hxd_p = -3.5*0.05*sin(0.05*t);      
       hyd = 3.5*sin(0.05*t)+1.75;    hyd_p =  3.5*0.05*cos(0.05*t);     
       hzd = 0.15*sin(0.5*t)+0.6;     hzd_p =  0.15*0.5*cos(0.5*t);  
 
%4) Configuraci�n Deseada del BRAZO         
    q1d =  0*pi/180*ones(1,length(t));
    q2d = 60*pi/180*ones(1,length(t));
    q3d =-40*pi/180*ones(1,length(t));
    q4d = 0*pi/180*ones(1,length(t));
    
hd  = [hxd;  hyd;   hzd;   q1d;   q2d;   q3d;    q4d];
%5) Velocidades m�ximas
    lb = [-3,-3,-2,-2,-2,-2]';
    ub = [ 3, 3, 2, 2, 2, 2]';

%6) Posici�n Inicial del Extremo Operativo   
    hx(1) = x(1)+a*cos(th(1))+cos(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
    hy(1) = y(1)+a*sin(th(1))+sin(q1(1)+th(1))*(l2*cos(q2(1))+l3*cos(q2(1)+q3(1))+l4*cos(q2(1)+q3(1)+q4(1)));
    hz(1) = ha+l2*sin(q2(1))+l3*sin(q2(1)+q3(1))+l4*sin(q2(1)+q3(1)+q4(1));
     
%7) Configuraci�n del Optimizador  
    options = optimset('Algorithm','sqp','Display','off');

tic
N=5;
for k=1:length(t)-N
aux = cputime;  

%1) MATRICES DE PESO
    %a)Errores de Posici�n del MANIPULADOR M�VIL
       H = [1  0  0;
            0  1  0;
            0  0  1];
   
    %b)Configuraci�n del BRAZO ROB�TICO
       Q = [1  0  0  0;
             0  1  0  0;
             0  0  1  0;
             0  0  0  1];
       R = diag([10 10 1 1 1]);
         
%2) VECTORES
    %a)Vector de estados
       q = [x(k),y(k),th(k),q1(k),q2(k),q3(k),q4(k)]';

    %b)Vectores del estremo operativo
       h   = [hx(k),   hy(k),    hz(k),    q1(k),    q2(k),    q3(k),     q4(k)]';
       
       delta_u=cambio(:,k);
%3) FUNCI�N OBJETIVO    
    f_obj = @(z) L_ope_Mani_Movi_Conf_Inter(z,H,Q,hd,h,q,l,ts,N,k,variacion);
   
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
    
%     uref(k) = uref_c(k);
%     wref(k) = wref_c(k);
%     q1pref(k) = q1pref_c(k);
%     q2pref(k) = q2pref_c(k);
%     q3pref(k) = q3pref_c(k); 
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
  
   cambio(:,k+1)=[u(k+1)-u(k),w(k+1)-w(k),q1p(k+1)-q1p(k),q2p(k+1)-q2p(k),q3p(k+1)-q3p(k)]';
    %4) ROBOT MANIPULADOT M�VIL
    %a)Plataforma M�vil
       xp = u(k+1)*cos(th(k+1))-a*w(k+1)*sin(th(k+1)); % cinem�tica
       yp = u(k+1)*sin(th(k+1))+a*w(k+1)*cos(th(k+1));
    
       x(k+1) =  ts*xp+x(k);   %euler m�vil
       y(k+1) =  ts*yp+y(k);   
       th(k+1)=  ts*w(k+1)+th(k);


    
    %c)Posici�n del estremo operativo en K+1
       hx(k+1) = x(k+1)+a*cos(th(k+1))+cos(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
                 l4*cos( q2(k+1)+q3(k+1)+q4(k+1)));
       hy(k+1) = y(k+1)+a*sin(th(k+1))+sin(q1(k+1)+th(k+1))*(l2*cos(q2(k+1))+l3*cos(q2(k+1)+q3(k+1))+...
                 l4*cos(q2(k+1)+q3(k+1)+q4(k+1)));
       hz(k+1) = ha+l2*sin(q2(k+1))+l3*sin(q2(k+1)+q3(k+1))+...
                 l4*sin(q2(k+1)+q3(k+1)+q4(k+1)); 

    %d)Vector de inicializaci�n para el optimizador             
       z0 = [u(k+1),w(k+1),q1p(k+1),q2p(k+1),q3p(k+1),q4p(k)]';  
%        z0 = [uref_c(k),wref_c(k),q1pref_c(k),q2pref_c(k),q3pref_c(k),q4p(k)]';
       variacion(:,k+1)=[u(k+1),w(k+1),q1p(k+1),q2p(k+1),q3p(k+1),q4p(k)]';
cpu(k) = cputime-aux;  %Tiempo de c�lculo
end
toc
%************************************************************************** 
%***************************ANIMACI�N****************************************
%% ************************************************************************** 
close all; paso=1; 
%a) Par�metros del cuadro de animaci�n
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
h = light;
h.Color=[0.65,0.65,0.65];
h.Style = 'infinite';
%b) Dimenciones del Robot
    DimensionesMovil();
    DimensionesManipulador(a,ha);

%c) Dibujo del Robot    
    G1=Movil3D(x(1),y(1),th(1));
    G2=Manipulador3D(x(1),y(1),th(1),q1(1),q2(1),q3(1),q4(1));

    plot3(hx(1),hy(1),hz(1),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on   
    plot3(hxd(1),hyd(1),hzd(1),'Color',[32,185,29]/255,'linewidth',1.5);
    
axis equal; 
for k = 1:10:length(ue)
    drawnow
    delete(G1);
    delete(G2);
 

    G1 = Movil3D(x(k),y(k),th(k));
    G2 = Manipulador3D(x(k),y(k),th(k),q1(k),q2(k),q3(k),q4(k));
    plot3(hxd(1:k),hyd(1:k),hzd(1:k),'Color',[32,185,29]/255,'linewidth',1.5);
    plot3(hx(1:k),hy(1:k),hz(1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Mobile Manipulator}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
%     axis([-1 5 -1 5 0 1]);
end
print -dpng SIMULATION_1
print -depsc SIMULATION_1
%************************************************************************** 
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
    plot(t(1:length(hx)),hxd(1:length(hx))-hx,'Color',[226,76,44]/255,'linewidth',1); hold on;
    plot(t(1:length(hx)),hyd(1:length(hx))-hy,'Color',[46,188,89]/255,'linewidth',1); hold on;
    plot(t(1:length(hx)),hzd(1:length(hx))-hz,'Color',[26,115,160]/255,'linewidth',1);hold on;
    grid on;
    legend({'$\tilde{h_{x}}$','$\tilde{h_{y}}$','$\tilde{h_{z}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Evolution of Primary Control Errors}$','Interpreter','latex','FontSize',9);
     ylabel('$[m]$','Interpreter','latex','FontSize',9);
  
subplot(3,1,3)
    t=[0:ts:tf-ts];
    plot(t(1:length(ue)),ue,'Color',[223,67,85]/255,'linewidth',1); hold on
    plot(t(1:length(ue)),we,'Color',[56,171,217]/255,'linewidth',1); hold on
    plot(t(1:length(ue)),q1pe,'Color',[32,185,29]/255,'linewidth',1); hold on
    plot(t(1:length(ue)),q2pe,'Color',[217,204,30]/255,'linewidth',1); hold on
    plot(t(1:length(ue)),q3pe,'Color',[83,57,217]/255,'linewidth',1); grid
    legend({'$\tilde\mu$','$\tilde{\omega}$','$\tilde{\dot{q}}_{1}$','$\tilde{\dot{q}}_{2}$','$\tilde{\dot{q}}_{3}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Evolution of Dinamic Compensation Errors}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
subplot(3,1,2)
t=[0:ts:tf];
    plot(t(1:length(q1)),q1d(1:length(q1))-q1,'Color',[32,185,29]/255,'linewidth',1); hold on
    plot(t(1:length(q1)),q2d(1:length(q1))-q2,'Color',[217,204,30]/255,'linewidth',1); hold on
    plot(t(1:length(q1)),q3d(1:length(q1))-q3,'Color',[83,57,217]/255,'linewidth',1); grid
    legend({'$\tilde{q}_{1}$','$\tilde{q}_{2}$','$\tilde{q}_{3}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Evolution of Secundary Control Errors}$','Interpreter','latex','FontSize',9);
    ylabel('$[rad]$','Interpreter','latex','FontSize',9);
    
print -dpng CONTROL_ERRORS_1
print -depsc CONTROL_ERRORS_1

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
t=[0:ts:tf-ts];
    plot(t(1:length(uref_c)),uref_c,'Color',[223,67,85]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),wref_c,'Color',[56,171,217]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),q1pref_c,'Color',[32,185,29]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),q2pref_c,'Color',[217,204,30]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),q3pref_c,'Color',[83,57,217]/255,'linewidth',1); grid on
    grid on;
    legend({'$\mu_{ref_c}$','$\omega_{ref_c}$','$\dot{q}_{1_{ref_c}}$','$\dot{q}_{2_{ref_c}}$','$\dot{q}_{3_{ref_c}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Optimal Control Values }$','Interpreter','latex','FontSize',9);
    ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
subplot(2,1,2)
    plot(t(1:length(uref)),uref,'Color',[223,67,85]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),wref,'Color',[56,171,217]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),q1pref,'Color',[32,185,29]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),q2pref,'Color',[217,204,30]/255,'linewidth',1); hold on
    plot(t(1:length(uref_c)),q3pref,'Color',[83,57,217]/255,'linewidth',1); grid on
grid on;
    legend({'$\mu_{ref}$','$\omega_{ref}$','$\dot{q}_{1_{ref}}$','$\dot{q}_{2_{ref}}$','$\dot{q}_{3_{ref}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Dinamic Compensation Values Applied to the Mobile Manipulator}$','Interpreter','latex','FontSize',9);
    xlabel('$\textrm{Time }[s]$','Interpreter','latex','FontSize',9); ylabel('$[m/s][rad/s]$','Interpreter','latex','FontSize',9);
print -dpng CONTROL_VALUES_1
print -depsc CONTROL_VALUES_1

figure()
plot(t(1,1:length(cpu)),cpu)