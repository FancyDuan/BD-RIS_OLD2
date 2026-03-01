function Euclid_grad_rg = Calculate_Euclid_grad_rg(Theta, X, Y, Z, R, G, r, g, M_bar)

    %% Euclidean Gradient
    Euclid_grad_rg = 0;
    idx_g = (g-1)*M_bar+1 : g*M_bar;
    for p = 1:R
        for k = 1:G
            idx_k = (k-1)*M_bar+1 : k*M_bar;
            Zrp_gk = Z(idx_g,idx_k,r,p);
            Theta_pk = Theta(idx_k,idx_k,p);
            Ypr_kg = Y(idx_k,idx_g,p,r);
            Euclid_grad_rg = Euclid_grad_rg + Zrp_gk*Theta_pk*Ypr_kg;
        end
    end
    Euclid_grad_rg = Euclid_grad_rg - X(idx_g,idx_g,r)';

end