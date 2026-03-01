function params_out = Calculate_SINR_SumRate(params)
% 输入: 参数结构体
% 输出: 新参数结构体(新写入了SINR, SumRate)

    %% 提取参数
    B = params.B;                                   % AP数
    K = params.K;                                   % UE数
    R = params.R;                                   % BD-RIS数
    N = params.N;                                   % AP天线数
    H = params.H;                                   % AP-RIS 莱斯信道
    h = params.h;                                   % RIS-UE 莱斯信道
    W = params.W;                                   % 波束赋形W
    P = params.P;                                   % 功率分配P (W)
    Theta = params.Theta;                           % RIS相移Theta
    noise_power = params.noise_power;               % 噪声功率 (W)

    %% 预分配变量
    kaba = zeros(N, B, K);                          % 聚合信道kaba矩阵      kaba(:,b,k)∈C(N×1)
    [signal_power, interference_power, SINR] = deal(zeros(K, 1));% 信号功率(W) / 用户间干扰功率(W) / 信干噪比

    %% 计算聚合信道kaba
    for b = 1:B
        for k = 1:K
            for r = 1:R
                kaba(:,b,k) = kaba(:,b,k) + H(:,:,b,r)*Theta(:,:,r)*h(:,r,k);
            end
        end
    end
    
    %% 计算信号功率
    for k = 1:K
        tmp1 = 0;
        for b = 1:B
            tmp1 = tmp1 + W(:,b,k)'*kaba(:,b,k);
        end
        signal_power(k) = P(k)*abs(tmp1)^2;
    end

    %% 计算用户间干扰功率
    for k = 1:K
        for i = 1:K
            if i ~= k
                tmp2 = 0;
                for b = 1:B
                    tmp2 = tmp2 + W(:,b,k)'*kaba(:,b,i);
                end
                interference_power(k) = interference_power(k) + P(i)*abs(tmp2)^2;
            end
        end
    end

    %% 计算SINR
    for k = 1 : K
        tmp3 = 0;
        for b = 1:B
            tmp3 = tmp3 + norm(W(:,b,k)')^2*noise_power;
        end
        SINR(k) = signal_power(k)/(interference_power(k) + tmp3);
    end

    %% 计算速率、和速率、平均和速率
    Rate = log2( 1 + SINR );                        % 速率
    SumRate = sum(Rate);                            % 和速率

    %% 在返回前，将新数据写入结构体
    params_out = params;                            % 复制结构体
    params_out.SINR = SINR;                         % 写入信干噪比SINR
    params_out.Rate = Rate;                         % 写入速率Rate
    params_out.SumRate = SumRate;                  % 写入和速率SumRate

end