function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율 (1.0=유지, <1.0=ABS 개입으로 release)
%       verCmd            - 4×1 damping [Ns/m]
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad]
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR]
%                                   (시나리오 강제 brake 와 합산되는 보정값 —
%                                    ABS 개입 시 음수로 release 효과)
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   Actuator allocation:
%       종방향: 전60:후40 분배, 좌우 균등
%       ESC:    yaw moment → 좌/우 차동 brake (lever arm = track/2)
%       ABS:    brakeRatio<1 이면 외력 brake 를 release 하는 음의 보정 토크 추가
%               (run_icc_scenario 가 brake_total = brk_scenario + brakeTorque 로
%                합산하므로, 여기서 음수를 내야 실제 release 가 가능)

    rw = VEH.rw;  % 타이어 유효 반경 [m]

    %% (1) 종방향 제동 — 4륜 분배 (전 60% : 후 40%)
    brakeTorque = zeros(4,1);  % [FL; FR; RL; RR]

    if lonCmd.Fx_total < 0
        Fx_brake = abs(lonCmd.Fx_total);  % 양수 크기

        % 전후 분배
        Fx_front = Fx_brake * 0.60;
        Fx_rear  = Fx_brake * 0.40;

        % 좌우 균등 분배, force → torque
        T_fl = Fx_front / 2 * rw;
        T_fr = Fx_front / 2 * rw;
        T_rl = Fx_rear  / 2 * rw;
        T_rr = Fx_rear  / 2 * rw;

        brakeTorque = [T_fl; T_fr; T_rl; T_rr];
    end

    %% (1b) ABS release — 시나리오/외력으로 강제된 brake 까지 줄이기 위한 음의 보정
    % brakeRatio < 1 이면 (1-brakeRatio) 비율만큼 LIM.MAX_BRAKE_TRQ 기준으로 release
    if isfield(lonCmd, 'brakeRatio') && lonCmd.brakeRatio < 1.0 && lonCmd.brakeRatio > 0
        releaseFrac = 1.0 - lonCmd.brakeRatio;
        absRelease = releaseFrac * LIM.MAX_BRAKE_TRQ * 0.6;  % per-wheel release 한도
        brakeTorque = brakeTorque - absRelease;
    end

    %% (2) ESC yaw moment → 차동 brake
    % 양의 Mz (CCW, 반시계) → 차량 좌회전 → 좌측 wheel 감속 필요 → 좌측 brake 증가
    % lever arm: t/2
    Mz = latCmd.yawMoment;

    if abs(Mz) > 1  % [Nm] deadband
        ratio_f = 0.5;  % 전륜 배분 비율

        t_f = VEH.track_f;
        t_r = VEH.track_r;

        % 전/후 차동 토크 크기
        dT_f = abs(Mz) * ratio_f     / t_f * rw;
        dT_r = abs(Mz) * (1-ratio_f) / t_r * rw;

        if Mz > 0
            % CCW moment 필요 → 우측(+x 방향) 제동 증가, 좌측 감소
            % [FL; FR; RL; RR]: FR, RR 증가
            brakeTorque(1) = brakeTorque(1) - dT_f;  % FL 감소
            brakeTorque(2) = brakeTorque(2) + dT_f;  % FR 증가
            brakeTorque(3) = brakeTorque(3) - dT_r;  % RL 감소
            brakeTorque(4) = brakeTorque(4) + dT_r;  % RR 증가
        else
            % CW moment 필요 → 좌측 제동 증가
            brakeTorque(1) = brakeTorque(1) + dT_f;  % FL 증가
            brakeTorque(2) = brakeTorque(2) - dT_f;  % FR 감소
            brakeTorque(3) = brakeTorque(3) + dT_r;  % RL 증가
            brakeTorque(4) = brakeTorque(4) - dT_r;  % RR 감소
        end
    end

    %% (3) Saturation — [-MAX_BRAKE_TRQ, MAX_BRAKE_TRQ]
    % 하한을 0이 아닌 -MAX_BRAKE_TRQ 로 둔다: ABS release 보정(음수)이 run_icc_scenario
    % 에서 시나리오 강제 brake(brk_scenario, 항상 ≥0)와 합산되어야 실제 release 효과가
    % 나타나기 때문 (brake_total = brk_scenario + brakeTorque, 최종 클리핑은 runner 가 함)
    brakeTorque = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, brakeTorque));

    %% (4) AFS 조향각 — saturation
    steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    %% 출력
    actuatorCmd.steerAngle   = steerAngle;
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd;

end
