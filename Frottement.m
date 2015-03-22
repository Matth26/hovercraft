function Fr = Frottement(V_rob)
global du  dv dr
 
 Fr =[  du * V_rob(1);
        dv * V_rob(2) ;
        dr * V_rob(3)];
 
end