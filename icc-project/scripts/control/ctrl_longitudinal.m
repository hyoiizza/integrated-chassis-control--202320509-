function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .absActive)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동)
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소
%       3. 저크 제한 (LIM.MAX_JERK)
%       4. anti-windup

    %% 내부 상태 초기화
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end
    if ~isfield(ctrlState, 'prevForce')
        ctrlState.prevForce = 0;
    end
    if ~isfield(ctrlState, 'absActive')
        ctrlState.absActive = false;
    end
    if ~isfield(ctrlState, 'wheelSlip')
        ctrlState.wheelSlip = zeros(4,1);
    end
    if ~isfield(ctrlState, 'slipIntError')
        ctrlState.slipIntError = 0;
    end

    mass = 1500;  % [kg] — VEH.mass (sim_params 기본값)

    %% (1) 속도 추종 PI — 실제 제동 중(ax<-2.0)에는 cruise 보정을 끄고 ABS 전용으로 전환
    % (원래 속도로 되돌리려는 cruise 항이 ABS 추가 제동(Fx_absAdd)과 상쇄되는 것을 방지,
    %  적분기도 함께 freeze 해 제동 종료 후 windup kick 이 생기지 않도록 함)
    % 임계값 -2.0 m/s²: 코너링 시 마찰원 한계 근처에서 자연 발생하는 종방향 감속
    % (~0.5-1 m/s² 수준)과 실제 의도된 제동(B1 기준 평균 ~3.7 m/s² 이상)을 구분
    err = vxRef - vx;

    if ax >= -2.0
        ctrlState.intError = ctrlState.intError + err * dt;
        ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
        Fx_cruise = CTRL.LON.Kp * err + CTRL.LON.Ki * ctrlState.intError;
    else
        Fx_cruise = 0;
    end

    % 최대 가속도 제한
    Fx_raw = max(-mass * LIM.MAX_AX, min(mass * LIM.MAX_AX, Fx_cruise));

    %% (2) 슬립률 closed-loop ABS — 목표 슬립(κ_target≈0.10, peak-mu 근방)을
    % 유지하도록 추가 제동력을 PI 로 직접 생성. 시나리오가 강제하는 brake 가
    % 마찰력 한계보다 작은 경우 추가 제동으로 정지거리를 단축하고, 슬립이
    % 과도해지면(>κ_target) 자동으로 추가분을 줄인다 — 양방향 닫힌루프.
    kappa_max = max(abs(ctrlState.wheelSlip));
    kappa_target = 0.10;
    kappa_hard   = 0.12;

    if ax < -2.0
        slipErr = kappa_target - kappa_max;   % >0: 슬립 부족(여유 있음), <0: 과다
        ctrlState.slipIntError = ctrlState.slipIntError + slipErr * dt;
        ctrlState.slipIntError = max(-2.0, min(2.0, ctrlState.slipIntError));

        Kp_slip = 8000;  % [N per slip-ratio unit]
        Ki_slip = 2000;
        Fx_absAdd = -(Kp_slip * slipErr + Ki_slip * ctrlState.slipIntError);
        % Fx_absAdd: slipErr>0(여유)→음수(추가 제동), slipErr<0(과다)→양수(release 방향)
        Fx_absAdd = max(-mass * LIM.MAX_AX, min(mass * LIM.MAX_AX, Fx_absAdd));

        ctrlState.absActive = kappa_max > kappa_hard;
    else
        Fx_absAdd = 0;
        ctrlState.slipIntError = 0;
        ctrlState.absActive = false;
    end

    Fx_raw = Fx_raw + Fx_absAdd;
    Fx_raw = max(-mass * LIM.MAX_AX, min(mass * LIM.MAX_AX, Fx_raw));

    %% (3) 저크 제한
    dF_max = mass * LIM.MAX_JERK * dt;
    dF = Fx_raw - ctrlState.prevForce;
    dF = max(-dF_max, min(dF_max, dF));
    Fx_total = ctrlState.prevForce + dF;
    ctrlState.prevForce = Fx_total;

    %% (4) brakeRatio — coordinator 가 이 추가 Fx_total 을 brake 로 분배하므로
    % release 는 hard 임계 초과 시에만 보조적으로 사용 (이중 안전장치)
    if ax < -2.0
        if kappa_max > kappa_hard
            over = min((kappa_max - kappa_hard) / 0.10, 1.0);
            brakeRatio = 1.0 - 0.7 * over;
        else
            brakeRatio = 1.0;
        end
    else
        brakeRatio = 0.0;
    end

    forceCmd.Fx_total   = Fx_total;
    forceCmd.brakeRatio = brakeRatio;

end
