function params_out = Position_Initialization(params)
% 输入: params
% 输出: params_out(新增 AP, BD-RIS, UE 位置)

    %% 提取参数
    B = params.B;                                           % AP数
    K = params.K;                                           % UE数
    R = params.R;                                           % BD-RIS数
    AreaSize = params.AreaSize;                             % 区域边长 (m)

    %% AP 位置
    switch B
        case 2
            AP_pos = [0 0; AreaSize AreaSize];
        case 3
            AP_pos = [AreaSize/2 AreaSize; 0, AreaSize; AreaSize 0];
        case 4
            AP_pos = [0 0; 0, AreaSize; AreaSize 0; AreaSize AreaSize];
        case 8
            AP_pos = [0 0; 0, AreaSize; AreaSize 0; AreaSize AreaSize; AreaSize/2 0 ; AreaSize/2 AreaSize;  0  AreaSize/2;  AreaSize AreaSize/2 ];
        otherwise
            error('Unsupported B_BS = %d', B);
    end

    %% BD-RIS 位置
    switch R
        case 1
            RIS_pos = [AreaSize/5 AreaSize/5];
        case 2
            RIS_pos = [AreaSize/5 AreaSize/5; 4*AreaSize/5 4*AreaSize/5];
        case 3
            RIS_pos = [AreaSize/5 AreaSize/5; 4*AreaSize/5 4*AreaSize/5; AreaSize/5 4*AreaSize/5];
        case 4
            RIS_pos = [AreaSize/5 AreaSize/5; 4*AreaSize/5 4*AreaSize/5; AreaSize/5 4*AreaSize/5; 4*AreaSize/5 AreaSize/5];
        otherwise
            error('Unsupported R_RIS = %d', R);
    end
    
    %% UE 位置
    UserDisk_Radius = 25;
    UserDisk_POS = [AreaSize/2, AreaSize/2];
    UE_pos = zeros(K,2);
    for k = 1:K
        radius = UserDisk_Radius * sqrt(rand);
        UE_pos(k,:) = UserDisk_POS + [radius*cos(2*pi*rand), radius*sin(2*pi*rand)];
    end
    
    %% 在返回前，将新数据写入结构体
    params_out = params;                                    % 复制结构体
    params_out.AP_pos = AP_pos;                             % 写入AP位置
    params_out.RIS_pos = RIS_pos;                           % 写入BD-RIS位置
    params_out.UE_pos = UE_pos;                              % 写入UE位置
    
end