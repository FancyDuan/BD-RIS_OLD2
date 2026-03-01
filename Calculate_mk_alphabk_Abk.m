function [alpha_bb_kk, A_bb_kk] = Calculate_mk_alphabk_Abk(params, bb, kk)
% 输入: 参数结构体params, 当前UE索引k
% 输出: 辅助变量mk

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

    %% 聚合信道kaba
    kaba = zeros(N,B,K);                            % 聚合信道kaba矩阵      kaba(:,b,k)∈C(N×1)
    for b = 1:B
        for k = 1:K
            for r = 1:R
                kaba(:,b,k) = kaba(:,b,k) + H(:,:,b,r)*Theta(:,:,r)*h(:,r,k);
            end
        end
    end

    %% 计算m_kk
    tmp1 = 0;
    for b = 1:B
        tmp1 = tmp1 + W(:,b,kk)'*kaba(:,b,kk);
    end
    tmp1 = sqrt(P(kk))*tmp1;

    tmp2 = 0;
    for i = 1:K
        if i ~= kk
            tmp3 = 0;
            for b = 1:B
                tmp3 = tmp3 + W(:,b,kk)'*kaba(:,b,i);
            end
            tmp2 = tmp2 + P(i)*abs(tmp3)^2;
        end
    end

    tmp4 = 0;
    for b = 1:B
        tmp4 = tmp4 + norm(W(:,b,kk)')^2*noise_power;
    end

    m_kk = tmp1/(tmp2+tmp4);
    
    %% 计算alpha_bb_kk
    c = zeros(B,K,K);
    for b = 1:B
        for k = 1:K
            for i = 1:K
                for b_ = 1:B
                    if b_ ~= b
                        c(b,k,i) = c(b,k,i) + W(:,b_,k)'*kaba(:,b_,i);
                    end
                end
            end
        end
    end

    % tmp5 = 0;
    % for i = 1:K
    %     if i ~= k
    %         tmp5 = tmp5 + P(i)*kaba(:,bb,i)*c(bb,kk,i)';
    %     end
    % end
    tmp5 = 0;
    for i = 1:K
        if i ~= kk
            tmp5 = tmp5 + P(i)*kaba(:,bb,i)*c(bb,kk,i)';
        end
    end

    alpha_bb_kk = m_kk'*sqrt(P(kk))*kaba(:,bb,kk) - abs(m_kk)^2*tmp5;

    %% 计算A_bb_kk
    tmp6 = 0;
    for i = 1:K
        if i ~= kk
            tmp6 = tmp6 + P(i)*kaba(:,bb,i)*kaba(:,bb,i)';
        end
    end

    A_bb_kk = abs(m_kk)^2*(noise_power*eye(N) + tmp6);

end