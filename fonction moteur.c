int* calc_force(unsigned char x, unsigned char y)
{
	// Renormaliser x et y :
	
	//
	
	// Calcul de theta et norme: 
	theta = atan(y/x);
	coeff_puissance  = y/sin(theta); //sin(t) = x/norme
	pi = 3.141592;
	
	int tab[2];
	if(coeff_puissance<=0){
		tab[0] = 0;
		tab[1] = 0;
	}	
	else if(theta > (pi/2) && theta < pi)
	{
		tab[0] = coeff_puissance * (pi/2 - (theta-pi/2)*120/pi);	// pourcentage sur le moteur gauche, revoir le rapport pourles radians
		tab[1] = coeff_puissance * 90; 								// 90% sur le moteur droit
	}
	else if(theta < (pi/2) && theta < 0)
	{
		tab[0] = coeff_puissance * 90;								// 90% sur le moteur droit
		tab[1] = coeff_puissance * (30 + theta*120/pi); 			// 
	}
