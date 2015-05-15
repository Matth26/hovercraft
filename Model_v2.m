clc;
clear all;
close all;

figure(1)
clf
hold on

axis equal
%grid on

%------------------------------------ MODELE AEROGLISSEUR ------------------------------------%

% CONSTANTE DE NOTRE SYSTEME %
global d k m_u m_v m_r du dv dr larg long
d = 2;                                      % distance entre le centre et l'axe des hélices de poussée
k = 1;                                      % constante de calcul du couple de l'hélice centrale

m_u = 1;                                    % masse du système selon l'axe u
m_v = 1;                                    % masse selon l'axe v, c'est la même que selon l'axe car nous sommes dans l'air
m_r = 1;                                    % masse appliquée au point centrale

du = .2;                                    % frottements selon u
%Yur =0*.02;                                   % effet de dérive (si u et r alors v)
dv = .2;                                    % frottements selon v
dr = .2;                                    % frottements selon r

larg = .4;                                      % largeur de l'hovercraft
long = .6;                                      % longueur de l'hovercraft


% VARIABLES %
u = 0;                                      % vitesse u dans le plan du robot
v = 0;                                      % vitesse v dans le plan du robot
r = 0;                                      % rotation r dans le plan du robot

x = 0;                                      % position du robot en x
y = 0;                                      % position du robot en y
theta = 0;                                  % orientation du robot

Vect_v_rob = [  u;
                v;
                r];
Rot = [ cos(theta),-sin(theta) 0;
        sin(theta),cos(theta) 0;
        0 0 1];
            
Vect_v_abs = Rot*Vect_v_rob;%[  0;0;0];

Vect_pos_abs = [    x;
                    y;
                    theta];

 q = [      Vect_v_abs(1);     % x_p
            Vect_pos_abs(1);   % x
            Vect_v_abs(2);     % y_p 
            Vect_pos_abs(2);   % y
            Vect_v_abs(3);     % theta_p
            Vect_pos_abs(3)    % theta
            ];              
    

% %------------------------------------ MATRICE D'ACTIONNEMENT ------------------------------------%
%
% Ma = [1 1 0;
%       0 0 0;
%       d -d k];
%
% %------------------------------------ MATRICE DYNAMIQUE ------------------------------------%
%
% Md = [m_u     0       0;
%       0       m_v     0;
%       0       0       m_r];
%
% Md_inv = inv(Md);
%
%
% Frottements = [du  0   0;
%                Yur dv  Yur;
%                0   0   dr];
%
%
% %------------------------------------ MATRICE DU MODELE CINEMATIQUE ------------------------------------%
%
% Mc = [cos(angle)  -sin(angle) 0;
%       sin(angle)  cos(angle)  0;
%       0           0           1];
%
% Mc_inv = inv(Mc);

%------------------------------------ VECTEUR DE FORCE APPLIQUEE ------------------------------------%

FG = 0.05;
FD = 0.1;
F1 = 0;

Vect_F =[FG; 
         FD; 
         F1];

%------------------------------------ CALCUL DE LA CHAINE DE MODELISATION ------------------------------------%

% calcul de du vecteur de Fb (forces dans le repère du robot, au centre de celui-ci)
Fb = MAct(Vect_F)

dt = 0.1;
tmax = 0.63;

% Plot initialisation
ptr = plot(Vect_pos_abs(1), Vect_pos_abs(2));
plot_hover(q, Vect_F, 1)


count = 0;

for T=0:dt:tmax*5
    count = count+1;
    %--------- Etat x y t xd yd td ----------%
    q = [   Vect_v_abs(1);     % x_p
            Vect_pos_abs(1);   % x
            Vect_v_abs(2);     % y_p
            Vect_pos_abs(2);   % y
            Vect_v_abs(3);     % theta_p
            Vect_pos_abs(3)    % theta
            ];
    plot_hover(q, Vect_F, 0)        
        
    % dessin
    %----------------- Dessin ---------------%
    set(ptr,'xdata',Vect_pos_abs(1),'ydata',Vect_pos_abs(2))
    plot(x,y);
    drawnow
    
    %------------ Calcul commande -----------%
    
     % Test de la fronde :
%     if(T>10 && T<10.2)
%         x3 = q(2);
%         y3 = q(4);
%     end 
%     if(T>=10.2)
%         Vect_F =[   -0.05;
%                     0.05;
%                     0];
%         Fb = MAct(Vect_F);   
%     end
    
    %----------------- ODE  ----------------%
    % On resout l'equa diff du second ordre
    % pour obtenir la vitesse du robot dans le plan absolu
    [t45, q45] = ode45(@(t,q) Acc2Vit2Pos(t, q, Fb),[0 dt], q);
    
    % On stock la trajectoire dans q_stock
    q_stock(:, count)=q;
    
    %------------- Etat = q45 --------------%
    Vect_v_abs(1) = q45(length(q45),1); % x_p
    Vect_v_abs(2) = q45(length(q45),3); % y_p
    Vect_v_abs(3) = q45(length(q45),5); % theta_p
    
    Vect_pos_abs(1) = q45(length(q45),2); % x
    Vect_pos_abs(2) = q45(length(q45),4); % y
    Vect_pos_abs(3) = q45(length(q45),6); % theta
    
    %------ x = etat(1)... td=etat(6) ------%
    x = Vect_pos_abs(1);
    y = Vect_pos_abs(2);
    theta = Vect_pos_abs(3);
end

% Plot de la trajectoire
figure(2)
hold on
xlabel('x(m)');
ylabel('y(m)');
title('Trajectoire');
plot(q_stock(2,:), q_stock(4,:));
%plot(x3, y3, 'rO');
hold off

