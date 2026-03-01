function params_out = Optimize_P_CVX_Scaled(params)
% 输入: 参数结构体params
% 输出: 新参数结构体params_out(更新了功率分配P)

    %% 提取参数
    B = params.B;                                   % AP数
    K = params.K;                                   % UE数
    R = params.R;                                   % BD-RIS数
    H = params.H;                                   % AP-RIS 莱斯信道
    h = params.h;                                   % RIS-UE 莱斯信道
    W = params.W;                                   % 波束赋形W
    P = params.P;                                   % 功率分配P (W)
    Pmax = params.Pmax;                             % 用户最大发射功率 (W)
    Theta = params.Theta;                           % RIS相移Theta
    noise_power = params.noise_power;               % 噪声功率 (W)
    
    %% 初始化辅助变量
    z = zeros(K, K);
    NI = zeros(K, 1);
    for k = 1: K
        for i = 1: K
            for b = 1: B
                tmp1 = 0;
                for r = 1: R
                    tmp1 = tmp1 + H(:,:,b,r)*Theta(:,:,r)*h(:,r,i);
                end
                z(k,i) = z(k,i) + W(:,b,k)'*tmp1;
            end
            z(k,i) = abs(z(k,i))^2;
        end
        for b = 1: B
            NI(k) = NI(k) + norm(W(:,b,k))^2*noise_power;
        end
    end
    
    s_old = zeros(K, 1);
    t_old = zeros(K, 1);
    for k = 1: K
        t_old(k) = sum(P .* z(k,:).') -  P(k)*z(k, k) + NI(k);
        s_old(k) = P(k)*z(k, k) / t_old(k);
    end
    
    %% 缩放
    factor = 1e11; % 1e8 ~ 1e11
    NI = NI * factor;
    z = z * factor;
    t_old = t_old * factor;
    
    %% 初始化迭代参数
    error_P = 100;
    iter_P = 0;
    obj_P_old = -100;
    cvx_solver Mosek

    while error_P > 1e-3 && iter_P <= 5  
        cvx_begin quiet
        cvx_expert true
            variables s(K, 1) t(K, 1) p(K, 1)
            maximize(sum_log(1+s))
            subject to
                p >= 0;
                p <= Pmax;
                for k = 1: K
                    sum( p .* z(k,:).' ) - p(k)*z(k, k) + NI(k) <= t(k);
                    p(k)*z(k, k) >= s_old(k)/2/t_old(k)*t(k)^2 + t_old(k)/2/s_old(k)*s(k)^2;
                end
        cvx_end

        if ~ strcmp(cvx_status, 'Solved') && ~ strcmp(cvx_status, 'Inaccurate/Solved')
            fprintf(' cvx状态: %s, 退出循环\n', cvx_status);
        else
            params.P = p;
            fprintf('SumRate = %3.3f, %s\n', Calculate_SINR_SumRate(params).SumRate, cvx_status);
        end

        obj_P_new = sum(log(1+s));
        error_P = abs(obj_P_new - obj_P_old);
        obj_P_old = obj_P_new;
        iter_P = iter_P + 1;
        s_old = s;
        t_old = t;
    end
    
    %% 返回新参数结构体params_out(更新了功率分配P)
    params_out = params;

end