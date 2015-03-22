function [  ] = plot_hover(q, Vect_F, mode)

global a larg long larg_mot long_mot
persistent h1 h2 h3 h4 h5 h6 h7 h8

hold on
%grid on
axis equal
% Dimension du robot
larg= 0.035;				% largeur chassis
long= 0.15;

a = 0.015;
larg_mot = 0.02;
long_mot = 0.01;


x_pos = q(2);
y_pos = q(4);
theta = q(6);

% Calcul de u, v, r:
    Vect_v_abs = zeros(3,1);
    Vect_v_abs(1) = q(1); % x_p = q(1)
    Vect_v_abs(2) = q(3); % y_p = q(3)
    Vect_v_abs(3) = q(5); % theta_p = q(5)
    
Vect_v_rob = McInv(Vect_v_abs,q(6));
u = Vect_v_rob(1);
v = Vect_v_rob(2);


% Plot des vitesses :
ROT = [ cos(theta),-sin(theta);
        sin(theta),cos(theta)];

center = [  (long/2);
            0];
center_abs = ROT*center + [x_pos; y_pos];

uu = [u, 0]';
uuu = ROT*uu;

vv = [0, v]';
vvv = ROT*vv;

if (mode==1)
    h5 = quiver(x_pos, y_pos, uuu(1), uuu(2));
    h6 = quiver(x_pos, y_pos, vvv(1), vvv(2));
else
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
scale = 0.5;

%axis([x_pos-1*scale x_pos+1*scale y_pos-1*scale y_pos+1*scale])
%axis([0 8 -8 0])


% Repère 

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

% Roue gauche
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

% Affichage des vitesses :


% % Helice :
% numPoints=100; %Number of points making up the circle
% radius=(2/3)*larg;      %Radius of the circle
% 
% %Define circle in polar coordinates (angle and radius)
% theta=linspace(0,2*pi,numPoints); %100 evenly spaced points between 0 and 2pi
% rho=ones(1,numPoints)*radius; %Radius should be 1 for all 100 points
% 
% %Convert polar coordinates to Cartesian for plotting
% [X,Y] = pol2cart(theta,rho); 
% 
% if (mode==1)
%  	h4 = plot((X+long/2),Y,'b-','linewidth',2);
% 	set(h4,'EraseMode','xor','LineWidth',2);
% else
%  	set(h4,'XData',X,'YData',Y);
% end