function ModCineInv = McInv(V_abs, angle)

Mc = [cos(angle)    -sin(angle)     0;
      sin(angle)    cos(angle)      0;
      0             0               1];

ModCineInv = Mc\V_abs;

end

