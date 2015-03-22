function MDynaInv = MdInv(Vect)
global m_u m_v m_r

Md = [m_u   0       0;
      0     m_v     0;
      0     0       m_r];
  
Md_inv = inv(Md);

MDynaInv = Md_inv * Vect ;
end