function Robot_Trayectoria(dt,tf,tray)
% dt:  paso de muestreo
% tf:  tiempo final de simulación
% tray: tipo de trayectoria
%     tray = 1:  Circunferencia de radio 5
%     tray = 2:  Curva de Bowditch
%     tray = 3:  Trayectoria particular

[t,nt,xd,yd] = trayectoria(dt,tf,tray);

% Parámetros
a = 2;
Q = [2,0;0,2];

% Trayectoria a construir
xk = zeros(nt,1);
yk = zeros(nt,1);
xk(1) = .5;
yk(1) = -.5;

% Velocidades
mu_k = zeros(nt,1);
w_k  = zeros(nt,1);

% Ángulo psi
psi_k = zeros(nt,1);

% Psi inicial
psi = pi/2;
psi_k(1) = psi;
hk = [xk(1),yk(1)]';

% Iterado inicial para el primer intervalo de muestreo
z0 = [mu_k(1),w_k(1)]';

% Cotas
lb = [-2,-1.5]';
ub = [2,1.5]';

options = optimset('Algorithm','sqp','Display','off');

t_pm = 0;
tiempo_total = cputime;
for k=1:nt-1
    hdk  = [xd(k),yd(k)]';
    hdk1 = [xd(k+1),yd(k+1)]';
    f_obj = @(z) L_ope(z,Q,hdk,hdk1,hk,psi,a,dt);
    cpu = cputime;
    res = fmincon(f_obj,z0,[],[],[],[],lb,ub,[],options);
    cpu = cputime-cpu;
    t_pm = t_pm + cpu;
    mu = res(1);
    w = res(2);
    hk = h_k1(mu,w,psi,a,hk);
    xk(k+1) = hk(1);
    yk(k+1) = hk(2);
    mu_k(k+1) = mu;
    w_k(k+1)  = w;
    psi = dt*w + psi;
    psi_k(k+1) = psi;
    z0 = [mu,w]';
end
tiempo_total = cputime-tiempo_total;

texto = 'Tiempo Total de CPU: %4.6f segundos\n';
fprintf(texto,tiempo_total)
texto = 'Tiempo promedio por itervalo de muestreo: %4.6f segundos\n';
fprintf(texto,t_pm/(nt-1))

figure(1)
x_max = max(xd);
x_min = min(yd);
y_max = max(yd);
y_min = min(yd);
i_x = [x_min-0.5,x_max+0.5];
i_y = [y_min-0.5,y_max+0.5];

subplot(2,2,1)
plot(xd,yd,'.-')
title('Trayectoria Deseada')
grid on
axis('equal')
xlim(i_x)
ylim(i_y)

subplot(2,2,2)
plot(xk,yk,'.-')
title('Trayectoria Hallada')
grid on
axis('equal')
xlim(i_x)
ylim(i_y)

subplot(2,2,3:4)
ek = sqrt(abs(xd-xk).*2 + abs(yd-yk).*2);
semilogy(t,ek,'k')
title('Error')
xlabel('t')
grid on
xlim([0 tf+.5])
ylim([1.0e-8 10])

figure(2)
subplot(1,3,1)
plot(t,mu_k)
xlabel('t')
title('mu(t)')
grid on

subplot(1,3,2)
plot(t,w_k)
xlabel('t')
title('w(t)')
grid on

subplot(1,3,3)
plot(t,psi_k)
xlabel('t')
title('psi(t)')
grid on
end

%  Funciones necesarias

function [F] = Fun(Q,hd,h)
aux = hd-h;
F = aux'*(Q*aux);
end

function [q] = trapecio(Q,hdk,hdk1,hk,hk1,dt)
q = (Fun(Q,hdk,hk)+Fun(Q,hdk1,hk1))/2;
end

function [hh] = h_k1(mu,w,psi,a,hk)
cp = cos(psi);
sp = sin(psi);
J = [sp,-a*cp; cp,a*sp];
v = [mu,w]';
hh = J*v + hk;
end

function [L] = L_ope(z,Q,hdk,hdk1,hk,psi,a,dt)
mu = z(1);
w = z(2);
hk1 = h_k1(mu,w,psi,a,hk);
L = trapecio(Q,hdk,hdk1,hk,hk1,dt);
end

function [t,nt,xd,yd] = trayectoria(dt,tf,tray)
t = (0:dt:tf)';
nt = length(t);
if tray == 1
    xd = 5*cos(2*t*pi/tf);
    yd = 5*sin(2*t*pi/tf);
elseif tray == 2
    xd = sin(2*t*pi/tf);
    yd = sin(2*(2*t)*pi/tf);
elseif tray == 3
    x_aux = [];
    y_aux = [];
    n_c = fix(nt/10);
    for i=1:10
       if i==1
           xx = linspace(1,6,n_c+1);
           yy = ones(1,n_c+1);
       elseif i==2
           xx = 6*ones(1,n_c);
           yy = linspace(1,6,n_c);
       elseif i==3
           xx = 7-linspace(1,6,n_c);
           yy = 6*ones(1,n_c);
       elseif i==4
           xx = ones(1,n_c);
           yy = 7-linspace(1,5,n_c);
       elseif i==5
           xx = linspace(1,5,n_c);
           yy = 2*ones(1,n_c);
       elseif i==6
           xx = 5*ones(1,n_c);
           yy = linspace(2,5,n_c);
       elseif i==7
           xx = 7-linspace(2,5,n_c);
           yy = 5*ones(1,n_c);
       elseif i==8
           xx = 2*ones(1,n_c);
           yy = 7-linspace(2,4,n_c);
       elseif i==9
           xx = linspace(2,4,n_c);
           yy = 3*ones(1,n_c);
       elseif i==10
           xx = 4*ones(1,n_c);
           yy = linspace(3,4,n_c);
       end
       x_aux = [x_aux,xx];
       y_aux = [y_aux,yy];
    end
    xd = x_aux';
    yd = y_aux';
end    
end