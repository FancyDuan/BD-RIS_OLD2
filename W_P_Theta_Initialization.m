function params_out = W_P_Theta_Initialization(params)
% 输入: 参数结构体
% 输出: 新参数结构体(新写入了波束赋形初始值, 功率分配初始值, RIS相移初始值)

    %% 提取参数
    B = params.B;                               % AP数
    K = params.K;                               % UE数
    R = params.R;                               % BD-RIS数
    G = params.G;                               % BD-RIS元素分组数
    N = params.N;                               % AP天线数
    M = params.M;                               % BD-RIS元素数
    M_bar = params.M_bar;                       % BD-RIS单组元素数
    Pmax = params.Pmax;                         % UE最大发射功率 (W)

    %% 初始化功率分配向量 P∈R(K×1)
    P = Pmax*ones(K,1); % W
    params.P = P;
    
    %% 初始化RIS反射系数矩阵 Theta(:,:,r)∈C(M×M)
    Theta = zeros(M,M,R);
    for r = 1:R
        Theta_r = zeros(M, M);
        for g = 1:G
            tmp_matrix = (randn(M_bar)+1i*randn(M_bar))/sqrt(2);
            [Qg,~] = qr(tmp_matrix,0);
            rows = (g-1)*M_bar + (1:M_bar);
            Theta_r(rows,rows) = Qg;
        end
        Theta(:,:,r) = Theta_r;
    end
    params.Theta = Theta;

    %% 初始化波束赋形 W(:,b,k)∈C(N×1), 满足归一化条件
    W = zeros(N, B, K);
    for b = 1:B
        for k = 1:K
            w_rand = randn(N,1) + 1i*rand(N,1);
            W(:,b,k) = w_rand/norm(w_rand);
        end
    end
    params.W = W;

    %% 复制结构体并输出
    params_out = params;

end