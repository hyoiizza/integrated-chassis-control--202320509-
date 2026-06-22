function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Unsprung velocity 기반 semi-active damping (groundhook 변형).
%
%   Inputs:
%       suspState - struct
%           .zu_dot(4)     - unsprung(wheel) mass 절대속도 [m/s] (위쪽 양수)
%           .zs_dot(4)     - sprung mass velocity (제공되는 경우, 참고용)
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   설계 근거:
%       run_icc_scenario 가 넘기는 suspState.zs/zs_dot 은 roll/pitch 로부터
%       근사한 값으로 heave(순수 상하) 성분이 빠져 있어, 표준 sprung-velocity
%       skyhook(zs_dot 기준)은 직진 single-bump 류 입력에서 신호가 거의 0이 되어
%       오히려 ride 를 악화시킨다. 대신 항상 신뢰 가능한 unsprung 절대속도
%       (zu_dot, plant state 에서 직접 옴) 만으로 판단하는 groundhook 변형을
%       사용한다: wheel 이 빠르게 움직일 때(=노면 충격이 막 전달되는 순간) 감쇠를
%       높여 wheel-hop 을 억제하고 충격을 흡수, 그 외에는 부드럽게 두어 body 로
%       전달되는 힘을 줄인다.

    %% 내부 상태 초기화
    if ~isfield(ctrlState, 'dampPrev')
        ctrlState.dampPrev = CTRL.VER.cMin * ones(4,1);
    end

    %% suspension 정보 없는 plant (bicycle/3dof 등) — passive damping 유지
    if ~isfield(suspState, 'zu_dot')
        dampingCmd = ctrlState.dampPrev;
        return;
    end

    %% Groundhook 변형 — unsprung 절대속도 크기에 비례한 연속 감쇠
    zu_dot = suspState.zu_dot(:);

    % |zu_dot| 이 클수록 cMax 쪽으로, 작을수록 cMin 쪽으로 연속 스케일링
    % (on-off 대신 연속 비례 — heave 신호 부재로 인한 토글 잡음을 줄임)
    v_ref = 0.15;  % [m/s] 정규화 기준 속도 (적당한 wheel-hop 속도 스케일)
    scale = min(abs(zu_dot) / v_ref, 1.0);

    dampingCmd = CTRL.VER.cMin + (CTRL.VER.cMax - CTRL.VER.cMin) * scale;

    %% cMin / cMax 범위 제한
    dampingCmd = max(CTRL.VER.cMin, min(CTRL.VER.cMax, dampingCmd));

    ctrlState.dampPrev = dampingCmd;

end
