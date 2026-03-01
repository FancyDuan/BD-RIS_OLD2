function params_out = Optimize_W_BCD_Normalized(params)
% 输入: 参数结构体
% 输出: 新参数结构体(更新了波束赋形变量W)

    %% 提取参数
    B = params.B;                               % AP数
    K = params.K;                               % UE数

    %% 初始化迭代参数
    error_W = 1;                                % 迭代误差
    epsilon_W = 1e-3;                           % 迭代精度
    iter_W = 1;                                 % 迭代次数
    iter_max_W = 10;                            % 最大迭代次数
    obj_W_old = Calculate_SINR_SumRate(params).SumRate;
    fprintf('SumRate = %7.3f\n', obj_W_old);

    %% 迭代优化算法
    while error_W > epsilon_W && iter_W <= iter_max_W

        % BCD算法
        for k = 1:K
            for b = 1:B
                [alpha_bk, A_bk] = Calculate_mk_alphabk_Abk(params, b, k);
                params.W(:,b,k) = normalize(A_bk\alpha_bk,'norm');
            end
        end

        % 更新迭代误差
        obj_W_new = Calculate_SINR_SumRate(params).SumRate;
        error_W = abs(obj_W_new - obj_W_old);
        obj_W_old = obj_W_new;

        % 更新迭代次数
        iter_W = iter_W + 1;

        fprintf('SumRate = %7.3f\n', obj_W_new);

    end
    
    params = Calculate_SINR_SumRate(params);
    fprintf('SumRate = %7.3f\n', params.SumRate);

    %% 更新参数结构体
    params_out = params;

end