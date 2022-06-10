function f=movil_des(x)

global  t  U W UP WP WREF UREF 
% constantes faltantes


% ingreso de matrices
F=[];
for i=1:length(t)
    
    u=U(i);
    w=W(i);
    up=UP(i);
    wp=WP(i);
    uref=UREF(i);
    wref=WREF(i);
 
    v=[u;w];
    vr=[uref;wref];
    vp=[up;wp];
 
    % matriz de inercia
    M11=x(1);
    M12=x(2);
    
    
    M21=x(3);
    M22=x(4);
   
    M=[[M11,M12;...
        M21,M22];
    % matriz centifuga-centripeta
    C11=x(5);
    C12=w*x(6);
    
    C21=w*x(7);
    C22=x(8);
   
    
    C=[[C11,C12]
        [C21,C22];
    % vector de gravedad

    %Dinamica
    min=vr-M*vp-C*v-G;
    F=[F;min];
    
    end
f=sqrt(F'*F); % funcion de minimizacion
% F'
end