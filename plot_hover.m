function [  ] = plot_hover(q, Vect_F, mode)

global a larg long larg_mot long_mot
persistent h1 h2 h3 h5 h6 h7 h8

hold on
%grid on
axis equal

% Dimension du robot
larg= 0.035; % largeur chassis
long= 0.15; % longueur chassis

a = 0.015;
larg_mot = 0.02; % largeur moteur
long_mot = 0.01; % longueur moteur


x_pos = q(2);
y_pos = q(4);
theta = q(6);

% Calcul de u, v, r:
    Vect_v_abs = zeros(3,1);
    Vect_v_abs(1) = q(1); % x_p = q(1)
    Vect_v_abs(2) = q(3); % y_p = q(3)
    Vect_v_abs(3) = q(5); % theta_p = q(5)

% On récupère les vecteurs u et v :
Vect_v_rob = McInv(Vect_v_abs,q(6));
u = Vect_v_rob(1);
v = Vect_v_rob(2);

% Plot des vitesses :
ROT = [ cos(theta),-sin(theta);
        sin(theta),cos(theta)];

center = [  (long/2);
            0           ];
        
center_abs = ROT*center + [x_pos; y_pos];

uu = [u, 0]'; % vecteur u si l'hover était en position de base
uuu = ROT*uu; % on le passe dans le repère de l'hover

vv = [0, v]';
vvv = ROT*vv;

if (mode==1)    % si c'est la premièe fois on créer h5 et h6
    h5 = quiver(x_pos, y_pos, uuu(1), uuu(2));
    h6 = quiver(x_pos, y_pos, vvv(1), vvv(2));
else    % sinon on les actualise
     set(h5,'XData',center_abs(1),'YData', center_abs(2), 'Udata', uuu(1), 'Vdata', uuu(2));
     set(h6,'XData',center_abs(1),'YData', center_abs(2), 'Udata', vvv(1), 'Vdata', vvv(2));   
end

% Plot des forces :
pG = [  0.001;
        a+(larg_mot/2)-0.005];
        
pD = [  0.001;
        -a-(larg_mot/2)+0.005];
    
pG_abs = ROT*pG + [x_pos; y_pos];
pD_abs = ROT*pD + [x_pos; y_pos];

FFg = [-Vect_F(1), 0]';
FFFg = ROT*FFg;

FFd = [-Vect_F(2), 0]';
FFFd = ROT*FFd;

if (mode==1)
    h7 = quiver(pG(1), pG(2), FFFg(1), FFFg(2));
    h8 = quiver(pD(1), pD(2), FFFd(1), FFFd(2));
else
     set(h7,'XData',pG_abs(1),'YData', pG_abs(2), 'Udata', FFFg(1), 'Vdata', FFFg(2));
     set(h8,'XData',pD_abs(1),'YData', pD_abs(2), 'Udata', FFFd(1), 'Vdata', FFFd(2));   
end

% Définition de l'axe :
%scale = 0.5;
%axis([x_pos-1*scale x_pos+1*scale y_pos-1*scale y_pos+1*scale])
%axis([0 8 -8 0])

% Titre et axes label :
xlabel('x(m)');
ylabel('y(m)');
title('Simulateur'); 

% Chassis
x = [0, (11/15)*long, long, (11/15)*long, 0, 0];
y = [larg, larg, 0, -larg, -larg, larg];
aux = ROT*[x;y] + [x_pos*ones(size(x));y_pos*ones(size(x))];
X = aux(1,:);
Y = aux(2,:);

if (mode==1)
	h1=plot(X, Y,'b-');
	set(h1,'EraseMode','xor','LineWidth',2);
else
	set(h1,'XData',X,'YData',Y);
end

% Moteur droit
x = ([0,long_mot,long_mot,0,0]+0.001);
y = ([0,0,-long_mot,-long_mot,0]-a);
aux = ROT*[x;y] + [x_pos*ones(size(x));y_pos*ones(size(x))];
X = aux(1,:);
Y = aux(2,:);

if (mode==1)
 	h2=plot(X, Y,'k-');
	set(h2,'EraseMode','xor','LineWidth',2);
else
	set(h2,'XData',X,'YData',Y);
end

% Moteur gauche
x = ([0,long_mot,long_mot,0,0]+0.001);
y = ([0,0,long_mot,long_mot,0]+a);
aux = ROT*[x;y] + [x_pos*ones(size(x));y_pos*ones(size(x))];
X = aux(1,:);
Y = aux(2,:);

if (mode==1)
 	h3=plot(X, Y,'k-');
	set(h3,'EraseMode','xor','LineWidth',2);
else
 	set(h3,'XData',X,'YData',Y);
end