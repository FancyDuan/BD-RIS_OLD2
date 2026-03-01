function params_out = Optimize_W_ZF_Normalized(params)
% 输入: 参数结构体
% 输出: 新参数结构(使用归一化ZF优化了波束赋形)

    %% 提取参数
    B = params.B;                                   % AP数
    K = params.K;                                   % UE数
    R = params.R;                                   % BD-RIS数
    N = params.N;                                   % AP天线数
    H = params.H;                                   % AP-RIS 莱斯信道
    h = params.h;                                   % RIS-UE 莱斯信道
    Theta = params.Theta;                           % RIS相移Theta

    %% 预分配
    W = zeros(N,B,K);                               % W(:,b,k) =  wb,k ∈C(N×1)
    W_zf = zeros(N,K,B);                            % W_zf(:,:,b) = Wb = [wb,1 ... wb,K] ∈C(N×K)

    %% ZF
    kaba = zeros(N,B,K);                            % 聚合信道kaba矩阵      kaba(:,b,k)∈C(N×1)
    for b = 1:B
        for k = 1:K
            for r = 1:R
                kaba(:,b,k) = kaba(:,b,k) + H(:,:,b,r)*Theta(:,:,r)*h(:,r,k);
            end
        end
    end

    KABA = zeros(N,K,B);                            % KABA_b = (kaba_b,1 ... kaba_b,K) ∈C(N×K)
    for b = 1:B
        for k = 1:K
            KABA(:,k,b) = kaba(:,b,k);
        end
    end

    for b = 1:B
        W_zf(:,:,b) = KABA(:,:,b)*pinv(KABA(:,:,b)'*KABA(:,:,b));
    end

    %% 归一化
    for b = 1:B
        for k = 1:K
            W(:,b,k) = W_zf(:,k,b)/norm(W_zf(:,k,b));% w_bk = [W_b]的第k行进行归一化
        end
    end

    %% 更新参数结构体
    params_out = params;
    params_out.W = W;

    params_out = Calculate_SINR_SumRate(params_out);
    fprintf('SumRate = %7.3f\n', params_out.SumRate);

end