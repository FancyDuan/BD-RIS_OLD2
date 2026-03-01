function params_out = Optimize_Theta_BCD_2(params)
% 输入: 参数结构体params
% 输出: 新参数结构体params_out(更新了Theta)

    %% 提取参数
    B = params.B;                                   % AP数
    N = params.N;                                   % AP天线数
    K = params.K;                                   % UE数
    R = params.R;                                   % BD-RIS数
    M_bar = params.M_bar;                           % BD-RIS单组元素数
    G = params.G;                                   % BD-RIS元素分组数
    H = params.H;                                   % AP-RIS 莱斯信道
    h = params.h;                                   % RIS-UE 莱斯信道
    Theta = params.Theta;                           % RIS相移Theta
    
    %% 计算聚合信道kaba
    kaba = zeros(N, B, K);                          % 聚合信道kaba矩阵      kaba(:,b,k)∈C(N×1)
    for b = 1:B
        for k = 1:K
            for r = 1:R
                kaba(:,b,k) = kaba(:,b,k) + H(:,:,b,r)*Theta(:,:,r)*h(:,r,k);
            end
        end
    end
    
    %% 初始化迭代参数
    error_Theta = 100;
    iter_Theta = 1;
    obj_Theta_old = -100;

    %% 迭代优化算法
    % fprintf('Initial SumRate = %4.3f\n', Calculate_SINR_SumRate(params).SumRate);

    while error_Theta > 1e-3 && iter_Theta <= 5

        % BCD算法
        for r = 1:R
            for g = 1:G
                % fprintf('optimizing Theta(%d,%d)\n', r, g);
                % 初始化内层迭代参数
                error_Theta_rg = 100;
                iter_Theta_rg = 1;
                obj_Theta_rg_old = -100;
                while error_Theta_rg > 1e-3 && iter_Theta_rg <= 20

                    % 更新相关变量
                    [X, Y, Z] = Calculate_eps_rho_X_Y_Z(params);

                    % 计算欧几里得梯度
                    Euclid_grad_rg = Calculate_Euclid_grad_rg(params.Theta, X, Y, Z, R, G, r, g, M_bar);

                    % 计算黎曼梯度
                    Rieman_grad_rg = Calculate_Rieman_grad_rg_2(Euclid_grad_rg, params.Theta, M_bar, r, g);

                    % 更新搜索方向
                    if iter_Theta_rg == 1 
                        Xi_rg = -Rieman_grad_rg;
                    else
                        Xi_rg = Calculate_Grad_2(params.Theta, r, g, M_bar, Rieman_grad_rg, Rieman_grad_rg_old, Xi_rg_old);
                    end

                    % 更新反射系数矩阵
                    % Theta_rg_new = Update_Theta_rg(X, Y, Z, Xi_rg, Rieman_grad_rg, Theta, M_bar, r, g, R);
                    % Theta_rg_new = Update_Theta_rg_2(X, Y, Z, Xi_rg, Rieman_grad_rg, Theta, M_bar, r, g, R);
                    Theta_rg_new = Update_Theta_rg_3(X, Y, Z, Xi_rg, Rieman_grad_rg, params.Theta, M_bar, r, g, R);
                    params.Theta((g-1)*M_bar+1:g*M_bar,(g-1)*M_bar+1:g*M_bar,r) = Theta_rg_new;

                    obj_Theta_rg_new = Calculate_SINR_SumRate(params).SumRate;
                    error_Theta_rg = abs(obj_Theta_rg_new - obj_Theta_rg_old);
                    obj_Theta_rg_old = obj_Theta_rg_new;
                    % fprintf('iter = %2d, SumRate = %4.3f\n', iter_Theta_rg, obj_Theta_rg_new);

                    iter_Theta_rg = iter_Theta_rg + 1;
                    Rieman_grad_rg_old = Rieman_grad_rg;
                    Xi_rg_old = Xi_rg;
                end
            end
        end

        % 更新迭代误差
        obj_Theta_new = Calculate_SINR_SumRate(params).SumRate;
        error_Theta = abs(obj_Theta_new - obj_Theta_old);
        obj_Theta_old = obj_Theta_new;

        % 更新迭代次数
        iter_Theta = iter_Theta + 1;

        fprintf('SumRate = %4.3f\n', obj_Theta_new);

    end
    
    %% 返回新参数结构体params_out(更新了Theta)
    params_out = params;

end