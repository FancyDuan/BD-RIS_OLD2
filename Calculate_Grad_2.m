function Xi_rg = Calculate_Grad_2(Theta, r, g, M_bar, Rieman_grad_rg, Rieman_grad_rg_old, Xi_rg_old)

    %% miu_rg
    idx_g = (g-1)*M_bar+1 : g*M_bar;
    Theta_rg = Theta(idx_g,idx_g,r);
    tmp1 = Rieman_grad_rg_old - Theta_rg*(Theta_rg'*Rieman_grad_rg_old + (Theta_rg'*Rieman_grad_rg_old)')/2;
    A = Rieman_grad_rg'*(Rieman_grad_rg - tmp1);
    B = Rieman_grad_rg_old'*Rieman_grad_rg_old;
    miu_rg = real(trace(A))/trace(B);

    %% Xi_rg
    tmp2 = Xi_rg_old - Theta_rg*(Theta_rg'*Xi_rg_old + (Theta_rg'*Xi_rg_old)')/2;
    Xi_rg = - Rieman_grad_rg + miu_rg*tmp2;

end