function ModCine = Mc(Acc_rob,angle)

Mc = [cos(angle) -sin(angle) 0;
      sin(angle) cos(angle) 0;
      0 0 1];
  
ModCine = Mc * Acc_rob;

end

