function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC)
%
%   LQR 기반 yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기.
%   Bicycle Model 상태공간을 속도별로 계산해 LQR 게인을 gain scheduling 한다.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s]
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.K_lqr, .K_target, .vx_prev)
%       CTRL       - 게인 파라미터 (사용 안 함 — Q/R 은 본 파일 내부에 하드코딩)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_YAW_RATE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad]
%       deltaAdd.yawMoment  - ESC yaw moment [Nm]
%       ctrlState           - 업데이트된 내부 상태

    %% 내부 상태 초기화
    if ~isfield(ctrlState, 'K_lqr');    ctrlState.K_lqr     = [0, 0]; end
    if ~isfield(ctrlState, 'K_target'); ctrlState.K_target  = [0, 0]; end
    if ~isfield(ctrlState, 'vx_prev');  ctrlState.vx_prev   = -999; end

    %% (1) Gain scheduling — 속도 변화가 크면 LQR 재계산
    vx_safe = max(vx, 2.0);

    % 속도가 1 m/s 이상 변한 경우에만 LQR 재계산 (매 스텝 계산 방지)
    if abs(vx_safe - ctrlState.vx_prev) > 1.0
        [A, B, ~, ~] = calc_bicycle_model(vx_safe, local_make_veh());

        % LQR 설계: 상태 x = [vy; yawRate], 입력 u = [steerAngle]
        % Q: yawRate 오차에 가중치, R: 조향 입력 패널티
        Q = diag([1, 25]);   % [vy, yawRate]
        R = 4.0;

        try
            ctrlState.K_target = lqr(A, B, Q, R);
        catch
            % lqr 실패 시 (vx 너무 낮은 등) 기존 목표 게인 유지
        end
        ctrlState.vx_prev = vx_safe;
    end

    % 게인 천이 평활화 — vx 가 연속적으로 변하는 시나리오(A4 등)에서 1m/s 단위로
    % 재계산된 새 게인이 한 스텝에 그대로 튀어 들어가면 steerRaw 가 불연속적으로
    % 점프해 진동을 유발한다. tau 시정수로 부드럽게 수렴시켜 점프를 없앤다.
    % (vx 가 거의 일정한 A1/A3/D1 등에서는 게인이 이미 정착돼 있어 응답 속도에
    %  영향을 주지 않음 — 빠른 step/DLC 보정은 그대로 즉시 반영됨)
    tau = 0.05;  % [s]
    alpha = min(dt / tau, 1.0);
    ctrlState.K_lqr = (1 - alpha) * ctrlState.K_lqr + alpha * ctrlState.K_target;

    K = ctrlState.K_lqr;

    %% (2) LQR AFS 보조 조향 — yawRate 오차 피드백 (vy 항은 추정 잡음이 커 제외)
    % yawRateRef 자체를 물리적으로 달성 가능한 한계(LIM.MAX_YAW_RATE)로 clamp.
    % calc_ref_yaw_rate 는 선형(소각) bicycle model 이라, driver(Stanley 등)가 급격한
    % 보정으로 큰 steer 각을 낼 때 타이어 마찰 한계를 훨씬 초과하는 비현실적 목표를
    % 낼 수 있다 — 이 경우 clamp 없이는 AFS 가 불가능한 목표를 쫓다 조향각이
    % 지속적으로 saturation 에 빠져 오히려 경로 추종을 악화시킨다.
    yawRateRefClamped = max(-LIM.MAX_YAW_RATE, min(LIM.MAX_YAW_RATE, yawRateRef));
    yawRate_err = yawRateRefClamped - yawRate;
    steerRaw = K(2) * yawRate_err;

    % Saturation
    steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steerRaw));

    %% (4) ESC — slip angle 임계 초과 시 yaw moment 인가
    beta_threshold = deg2rad(3.0);  % [rad] 3°
    K_beta = 20000;                 % [Nm/rad] ESC 게인

    % 속도 스케일 (고속에서 ESC 효과 강화)
    vx_scale = min(vx_safe / 20.0, 2.0);

    yawMoment = 0;
    if abs(slipAngle) > beta_threshold
        excess = abs(slipAngle) - beta_threshold;
        yawMoment = -K_beta * sign(slipAngle) * excess * vx_scale;
    end

    %% 출력
    deltaAdd.steerAngle = steerAngle;
    deltaAdd.yawMoment  = yawMoment;

end

%% ---------------------------------------------------------------
function VEH = local_make_veh()
% sim_params 기본값 (generic C-segment sedan)
    VEH.mass    = 1500;
    VEH.Iz      = 2500;
    VEH.lf      = 1.2;
    VEH.lr      = 1.4;
    VEH.Cf      = 80000;
    VEH.Cr      = 85000;
    VEH.track_f = 1.55;
    VEH.track_r = 1.55;
    VEH.rw      = 0.31;
end
