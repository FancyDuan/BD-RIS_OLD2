function params_out = Channel_Initialization(params)
% 输入: 参数结构体
% 输出: 新参数结构体(新写入了2个信道矩阵)
    
    %% 提取参数
    B = params.B;                               % AP数
    K = params.K;                               % UE数
    R = params.R;                               % BD-RIS数
    N = params.N;                               % AP天线数
    M = params.M;                               % BD-RIS元素数
    AP_pos = params.AP_pos;                     % AP位置
    UE_pos = params.UE_pos;                     % UE位置
    RIS_pos = params.RIS_pos;                   % BD-RIS位置
    beta = params.beta;                         % 参考距离为1m时的路径损耗
    Rician_AP_RIS = params.Rician_AP_RIS;       % AP-RIS信道的莱斯因子
    Rician_RIS_UE = params.Rician_RIS_UE;       % RIS-UE信道的莱斯因子
    alpha_AP_RIS = params.alpha_AP_RIS;         % AP-RIS的路径损耗指数
    alpha_RIS_UE = params.alpha_RIS_UE;         % RIS-UE的路径损耗指数

    %% 预分配信道矩阵
    H = zeros(N, M, B, R);                      % AP-RIS的信道, H(b,r)  维度N×M
    H_LoS = zeros(N, M, B, R);                  % H视距部分
    H_NLoS = zeros(N, M, B, R);                 % H非视距部分
    h = zeros(M, R, K);                         % RIS-UE的信道, h(r,k)  维度M×1
    h_LoS = zeros(M, R, K);                     % h视距部分
    h_NLoS = zeros(M, R, K);                    % h非视距部分

    %% 生成AP-RIS的莱斯信道H
    for b = 1:B
        for r = 1:R
            % 计算路损
            dis = norm(AP_pos(b,:) - RIS_pos(r,:));
            PL = beta*dis^(-alpha_AP_RIS);
            
            % LoS分量
            H_LoS(:,:,b,r) = exp(1i*2*pi*rand(N,M));
    
            % NLoS分量 randn() 正态分布 ~ N(0,1)
            H_NLoS(:,:,b,r) = (randn(N,M) + 1i*randn(N,M))/sqrt(2);
            
            % 信道H
            H(:,:,b,r) = sqrt(PL)*(sqrt(Rician_AP_RIS/(Rician_AP_RIS + 1))*H_LoS(:,:,b,r) + sqrt(1/(Rician_AP_RIS+1))*H_NLoS(:,:,b,r));
        end
    end

    %% 生成RIS-UE的莱斯信道h
    for r = 1:R
        for k = 1:K
            % 计算路损
            dis= norm(RIS_pos(r,:) - UE_pos(k,:));
            PL = beta*dis^(-alpha_RIS_UE);
    
            % LoS分量
            h_LoS(:,r,k) = exp(1i*2*pi*rand(M,1));
    
            % NLoS分量 randn() 正态分布
            h_NLoS(:,r,k) = (randn(M,1) + 1i*randn(M,1))/sqrt(2);

            % 信道h
            h(:,r,k) = sqrt(PL)*(sqrt(Rician_RIS_UE/(Rician_RIS_UE+1))*h_LoS(:,r,k) + sqrt(1/(Rician_RIS_UE+1))*h_NLoS(:,r,k));
        end
    end
    
    %% 在返回前, 将新数据写入结构体
    params_out = params;
    params_out.H = H;                           
    params_out.H_LoS = H_LoS;
    params_out.H_NLoS = H_NLoS;                        
    params_out.h = h;
    params_out.h_LoS = h_LoS;
    params_out.h_NLoS = h_NLoS;            
    
end