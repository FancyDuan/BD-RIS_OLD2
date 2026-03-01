function Rieman_grad_rg = Calculate_Rieman_grad_rg_2(Euclid_grad_rg, Theta, M_bar, r, g)
    
    %% Riemanean Gradient 
    idx_g = (g-1)*M_bar+1 : g*M_bar;
    Theta_rg = Theta(idx_g,idx_g,r);
    Rieman_grad_rg = Euclid_grad_rg - Theta_rg*(Theta_rg'*Euclid_grad_rg + (Theta_rg'*Euclid_grad_rg)')/2;

end