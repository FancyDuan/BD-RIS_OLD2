function [X, Y, Z] = Calculate_eps_rho_X_Y_Z(params)
% 输入: 参数结构体
% 输出: 新参数结构体

    %% 提取参数
    B = params.B;                                   % AP数
    K = params.K;                                   % UE数
    R = params.R;                                   % BD-RIS数
    N = params.N;                                   % AP天线数
    M = params.M;                                   % BD-RIS元素数
    H = params.H;                                   % AP-RIS 莱斯信道
    h = params.h;                                   % RIS-UE 莱斯信道
    W = params.W;                                   % 波束赋形W
    P = params.P;                                   % 功率分配P (W)
    Theta = params.Theta;                           % RIS相移Theta
    noise_power = params.noise_power;               % 噪声功率 (W)

    %% eps∈R(K×1)
    eps = Calculate_SINR_SumRate(params).SINR;

    %% 聚合信道kaba
    kaba = zeros(N,B,K);                            % 聚合信道kaba矩阵      kaba(:,b,k)∈C(N×1)
    for b = 1:B
        for k = 1:K
            for r = 1:R
                kaba(:,b,k) = kaba(:,b,k) + H(:,:,b,r)*Theta(:,:,r)*h(:,r,k);
            end
        end
    end

    %% 计算rho∈R(K×1)
    rho = zeros(K,1);

    tmp1 = zeros(K,K);
    for k = 1:K
        for i = 1:K
            for b = 1:B
                tmp1(k,i) = tmp1(k,i) + W(:,b,k)'*kaba(:,b,i);
            end
        end
    end

    for k = 1:K
        rho(k) = sqrt((1+eps(k))*P(k))*tmp1(k,k)/(sum(P .* abs(tmp1(k,:)).^2.') + B*noise_power);
    end

    %% X(i)∈C(M×M)
    X = zeros(M,M,R);
    for i = 1:R
        for b = 1:B
            for k = 1:K
                X(:,:,i) = X(:,:,i) + 2*sqrt((1+eps(k))*P(k))*h(:,i,k)*rho(k)'*W(:,b,k)'*H(:,:,b,i);
            end
        end
    end

    %% Y(p,q)∈C(M×M)
    Y = zeros(M,M,R,R);
    for p = 1:R
        for q = 1:R
            for i = 1:K
                Y(:,:,p,q) = Y(:,:,p,q) + P(i)*h(:,p,i)*h(:,q,i)';
            end
        end
    end

    %% Z(q,p)∈C(M×M)
    a = zeros(1,M,K,R);
    Z = zeros(M,M,R,R);
    for k = 1:K
        for p = 1:R
            for b = 1:B
                a(:,:,k,p) = a(:,:,k,p) + W(:,b,k)'*H(:,:,b,p);
            end
        end
    end
    for q = 1:R
        for p= 1:R
            for k = 1:K
                Z(:,:,q,p) = Z(:,:,q,p) + abs(rho(k))^2*a(:,:,k,q)'*a(:,:,k,p);
            end
        end
    end

end