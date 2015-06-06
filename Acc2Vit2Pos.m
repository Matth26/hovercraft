function [dq] = Acc2Vit2Pos(t, q, Fb)
    % q(1) = x_p      q(2) = x
    % q(3) = y_p      q(4) = y
    % q(5) = theta_p  q(6) = theta
    
    Vect_v_abs = zeros(3,1);
    Vect_v_abs(1) = q(1); % x_p = q(1)
    Vect_v_abs(2) = q(3); % y_p = q(3)
    Vect_v_abs(3) = q(5); % theta_p = q(5)
    
    %----- CHAINE DE RETOUR -----%
    % On réactualise le modele cinematique et on calcul (u, v, r):
    Vect_v_rob = McInv(Vect_v_abs,q(6));
    
    Frottements = Frottement(Vect_v_rob);
    % Frottements = Frottement(Vect_v_abs); % on aurait également pu
    % appliquer les frottement dans le repère absolu
    
    %------- SOUSTRACTEUR -------%
    % YUR=[0 Yur*Vect_v_rob(1)*Vect_v_rob(3) 0]';
    Diff = (Fb - Frottements);%+YUR;
    
    % Calcul de l'accélération dans le plan du robot (u_p, v_p, r_p)
    Vect_acc_rob = MdInv(Diff);
    
    % Calcul de l'accélation dans le repère absolu (x_pp, y_pp, theta_pp)
    Vect_acc_abs = Mc(Vect_acc_rob, q(6));

    % On scinde l'equa diff du second ordre :    
    dq = zeros(6,1);
    dq(1) = Vect_acc_abs(1);    % x_pp = Vect_acc_abs(1)
    dq(2) = q(1);               % x_p = q(1)
    dq(3) = Vect_acc_abs(2);    % y_pp = Vect_acc_abs(2)
    dq(4) = q(3);               % y_p = q(3)
    dq(5) = Vect_acc_abs(3);    % theta_pp = Vect_acc_abs(3)
    dq(6) = q(5);               % theta_p = q(5)

    
