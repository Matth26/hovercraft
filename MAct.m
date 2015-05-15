function MatActionnement = MAct(Vect_F)
global d k
Ma = [  1   1   0;
        0   0   0;
        -d   d  k];

MatActionnement = (Ma * Vect_F);
end

