function ModCineInv = McInv(V_abs, angle)

Mc = [cos(angle)    -sin(angle)     0;
      sin(angle)    cos(angle)      0;
      0             0               1];
  
%Mc_inv = inv(Mc);

ModCineInv = Mc\V_abs;

end

