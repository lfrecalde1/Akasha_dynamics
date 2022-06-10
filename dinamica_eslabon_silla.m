clc,clear all,close all;
syms m1 b thp thp_p Lcons q1 q1_p Kz thz thz_p Kl Kh qe Ta Tb B Kpa Kpb Rpa Kpt Kpr Kdt Kdr Ka Ra Kb Kp Kd  R r m2 g real



J11=m2*Lcons^2*(cos(q1))^2;

Mp=[J11];
MP=simplify(Mp);

JJ11=-m2*Lcons^2*cos(q1)*sin(q1)*q1_p;


Cp=[JJ11];

CP=simplify(Cp);

G11=m2*g*Lcons*cos(q1);

Gp=[G11];
GP=simplify(Gp);


S11=(Kz^2*thz+Kl*Kz)/(Kh*sin(q1+qe));

Sp=[S11];

SP=simplify(Sp);


Sp_p11=(Kh*sin(q1+qe)*thz_p-Kh*Kz*cos(q1+qe)*q1_p*(Kz*thz+Kl))/((Kh*sin(q1+qe))^2);

Sp_p=[Sp_p11];

SP_P=simplify(Sp_p);

Bp=[Ta*cos(Tb)*sin(Tb)*sin(B-q1)];
% matrices de constantes internas de los motores
Dp=[Ka/Ra];

Ep=[(Ka*Kb)/Ra];
% matrices de los pid  para la velocidad lineal y angular
Lp=[Kp];

Jp=[Kd];

M=pinv(Lp)*(pinv(Dp)*pinv(SP'*Bp)*SP'*MP*SP+Jp);
MO=simplify(M);

C=pinv(Lp)*(pinv(Dp)*pinv(SP'*Bp)*SP'*MP*SP_P+pinv(Dp)*pinv(SP'*Bp)*SP'*CP*SP+pinv(Dp)*Ep+Lp);
CO=simplify(C);

M_f=pinv(Lp)*(pinv(Dp)*pinv(Bp)*MP*SP+Jp);
MO_f=simplify(M_f);

C_f=pinv(Lp)*(pinv(Dp)*pinv(Bp)*MP*SP_P+pinv(Dp)*pinv(Bp)*CP*SP+pinv(Dp)*Ep+Lp);
CO_f=simplify(C_f);

G_f=pinv(Lp)*pinv(Dp)*pinv(Bp)*GP;
GO_f=simplify(G_f);