function Theta_rg_new = Update_Theta_rg_3(X, Y, Z, Xi_rg, Rieman_grad_rg, Theta, M_bar, r, g, R)
    
    %% slope = <Rieman_grad_rg, Xi_rg>
    slope = real(trace(Rieman_grad_rg'*Xi_rg));

    %% obj_old( - J_1,5 )
    obj_old = 0;
    for p = 1:R
        for q = 1:R
            tmp2 = Theta(:,:,p)*Y(:,:,p,q)*Theta(:,:,q)'*Z(:,:,q,p);
            obj_old = obj_old + trace(tmp2);
        end
    end
    tmp3 = 0;
    for i = 1:R
        tmp3 = tmp3 + real(trace(Theta(:,:,i)*X(:,:,i)));
    end
    obj_old = obj_old - tmp3;

    %% backtracking for delta_rg
    delta_rg = 1;
    c = 1e-4;
    idx_g = (g-1)*M_bar+1 : g*M_bar;
    Theta_rg = Theta(idx_g,idx_g,r);

    while true
        tmp1 = eye(M_bar) + delta_rg^2*(Xi_rg'*Xi_rg);
        InvSqrt = inv(sqrtm(tmp1));
        Theta_rg_trial = (Theta_rg + delta_rg*Xi_rg)*InvSqrt;
        Theta_trial = Theta;
        Theta_trial(idx_g,idx_g,r) = Theta_rg_trial;
    
        obj_new = 0;
        for p = 1:R
            for q = 1:R
                tmp2 = Theta_trial(:,:,p)*Y(:,:,p,q)*Theta_trial(:,:,q)'*Z(:,:,q,p);
                obj_new = obj_new + trace(tmp2);
            end
        end
        tmp3 = 0;
        for i = 1:R
            tmp3 = tmp3 + real(trace(Theta_trial(:,:,i)*X(:,:,i)));
        end
        obj_new = obj_new - tmp3;
    
        if obj_new <= obj_old + c*delta_rg*slope
            break;
        end
        delta_rg = delta_rg * 0.5;

        % 防止死循环
        if delta_rg < 1e-10
            break; 
        end
    end

    Theta_rg_new = Theta_rg_trial;

end