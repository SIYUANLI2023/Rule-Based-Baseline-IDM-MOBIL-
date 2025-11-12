function baseline = run_case2_baseline_IDM_MOBIL()
% =====================================================================
%  Rule-based Baseline Simulation for Case 2
%  -------------------------------------------------------------
%  Scenario:
%    - Multi-lane (3-lane) highway with cars and trucks
%    - Ego Vehicle (EV): longitudinal IDM + lateral MOBIL
%    - SV1: reacts to EV lane change (accelerate + PD following)
%    - SV2: cruise
%    - TR1, TR2: trucks cruising in rightmost lane
%
%  Output:
%    baseline structure containing all configurations and vehicle logs
% =====================================================================

%% =================== CONFIGURATION PARAMETERS ===================
cfg.dt      = 0.15;         % low-level integration step (s)
cfg.Th      = 0.60;         % lateral decision interval (s)
cfg.Tsim    = 40.0;         % total simulation time (s)
cfg.wLane   = 4.0;          % lane width (m)
cfg.yc      = [4, 0, -4];   % lane center y-coordinates (L→R)
cfg.nLane   = 3;            % number of lanes
cfg.Lveh    = 4.5;          % vehicle length (m)
cfg.Tlc     = 8;           % lane-change duration (s)
cfg.truckLaneID       = 3;  % lane ID for trucks (rightmost)
cfg.truck_length_fact = 2.0;% truck length factor
cfg.ev_forbid_lane3   = true; % EV forbidden to enter truck lane
cfg.avoidLane3        = true; % optional bias penalty toward lane 3
cfg.biasLane3         = -0.3; % lane 3 bias (used in MOBIL gain)
cfg.dsafe_plot        = 60;   % visualization range for plotting

%% ---------------- SV1 Behavioral Parameters ----------------
cfg.sv1_trigger_on_EV  = true;   % trigger accel when EV starts LC
cfg.sv1_trigger_on_SV2 = false;  % trigger on SV2 (optional)
cfg.sv1_ax_on_trigger  = 2.5;    % acceleration after trigger (m/s^2)
cfg.sv1_vmax           = 38.0;   % max speed (m/s)

% TTC-based emergency braking (optional, it does not use in this situation)
cfg.sv1_use_TTC_EBS = false;

cfg.sv1_ebs.tau_soft = 3.0;   % soft TTC threshold (s)
cfg.sv1_ebs.tau_hard = 1.5;   % hard TTC threshold (s)
cfg.sv1_ebs.b_soft   = 4.0;   % soft braking (m/s^2)
cfg.sv1_ebs.a_hard   = 6.0;   % hard braking (m/s^2)

% Hysteretic PD-based following control for SV1
cfg.sv1_follow.enable = true;
cfg.sv1_trigger_delay = 2;
cfg.sv1_follow.s0     = 2.0;     % minimum spacing (m)
cfg.sv1_follow.Th     = 1.5;     % desired time headway (s)
cfg.sv1_follow.Kp     = 0.25;    % proportional gain
cfg.sv1_follow.Kd     = 0.90;    % derivative gain
cfg.sv1_follow.ax_min = -3.0;    % min acc (comfortable decel)
cfg.sv1_follow.ax_max =  2.0;    % max acc (comfortable accel)
cfg.sv1_follow.enter_margin = -0.0; % hysteresis enter threshold
cfg.sv1_follow.exit_margin  =  4.0;  % hysteresis exit threshold

%% ---------------- IDM Parameters----------------
idm.s0     = 2;       % minimum spacing
idm.T      = 1.5;     % desired time headway
idm.aMax   = 8.0;     % maximum acceleration
idm.bComf  = 4.0;     % comfortable deceleration
idm.v0     = 30.0;    % desired velocity
idm.delta  = 6.0;     % exponent in velocity term

%% ---------------- MOBIL Parameters (for EV only) ----------------
mobil.p         = 0.3;  % politeness factor
mobil.a_thr     = 0.10; % lane-change trigger threshold (m/s^2)
mobil.b_safe    = 3.0;  % safe braking limit for others (m/s^2)
mobil.bias_keep = 0.0;  % bias to stay in lane

%% =================== INITIAL CONDITIONS ===================
% Lane 1 (left): SV1 at rear
% Lane 2 (center): EV and SV2
% Lane 3 (right): two trucks
veh = repmat(veh_template(idm, cfg), 5, 1);
veh = initVeh(veh, 1, 'EV',  2,  35,  cfg.yc(2), 30);
veh = initVeh(veh, 2, 'SV1', 1,   0,  cfg.yc(1), 25);
veh = initVeh(veh, 3, 'SV2', 2, 150,  cfg.yc(2), 25);
veh = initVeh(veh, 4, 'TR1', 3, 120,  cfg.yc(3), 25);
veh = initVeh(veh, 5, 'TR2', 3, 160,  cfg.yc(3), 25);

% Double truck lengths for visualization realism
veh(4).L = cfg.Lveh * cfg.truck_length_fact;
veh(5).L = cfg.Lveh * cfg.truck_length_fact;

% Only EV uses MOBIL for lane-change decisions 
veh(1).is_EGOMOBIL = true;

% Initialize SV1 trigger and following states

veh(2).accel_override.active  = false;   % 
veh(2).accel_override.pending = false;   % 
veh(2).accel_override.t0      = 0;       %
veh(2).follow.active          = false;

%% =================== LOG ALLOCATION ===================
N = round(cfg.Tsim/cfg.dt)+1;
for i = 1:numel(veh)
    veh(i).log.t    = zeros(N,1);
    veh(i).log.x    = zeros(N,1);
    veh(i).log.y    = zeros(N,1);
    veh(i).log.v    = zeros(N,1);
    veh(i).log.ax   = zeros(N,1);
    veh(i).log.lane = zeros(N,1);
    veh(i).log.alat = zeros(N,1); 
end

%% =================== MAIN SIMULATION LOOP ===================
ndec = round(cfg.Th/cfg.dt); % decision-step counter
prevLCActive = arrayfun(@(s) s.lc.active, veh);  % previous LC active state

for n = 1:N
    t = (n-1)*cfg.dt;

    if veh(1).is_EGOMOBIL && ~veh(1).lc.active && (mod(n-1,ndec)==0)
        [lead_old, ~] = findLeadFollow(veh, 1, veh(1).lane, cfg);
        if ~isempty(lead_old)
            dist = lead_old.x - veh(1).x;      
            if dist < 95                      
                veh = mobil_decide_and_trigger(veh, 1, cfg, mobil, t);
            end
        end
    end

        nowLCActive = arrayfun(@(s) s.lc.active, veh);
        ev_started  = (~prevLCActive(1) && nowLCActive(1));
        sv2_started = (numel(veh)>=3) && (~prevLCActive(3) && nowLCActive(3));
        
        if ~veh(2).accel_override.active && ~veh(2).accel_override.pending && ...
           ( (cfg.sv1_trigger_on_EV  && ev_started) || ...
             (cfg.sv1_trigger_on_SV2 && sv2_started) )
            veh(2).accel_override.pending = true;   % pending
            veh(2).accel_override.t0      = t;      % record trigger time
        end
        prevLCActive = nowLCActive;
   
    if veh(2).accel_override.pending && ...
       (t - veh(2).accel_override.t0 >= cfg.sv1_trigger_delay)
        veh(2).accel_override.active  = true;   % acclerate now
        veh(2).accel_override.pending = false;
    end

    % —— Longitudinal acceleration for each vehicle
    for i = 1:numel(veh)
        if i==1   % EV: IDM (if lane changing, use target-lane leader)
            lane4IDM = veh(i).lane;
            if veh(i).lc.active
                lane4IDM = veh(i).lc.targetLane;
            end
            [lead, ~] = findLeadFollow(veh, i, lane4IDM, cfg);
            [a_idm, gap, dv] = idm_accel(veh(i), lead, cfg);
            veh(i).ax_cmd = a_idm;
            veh(i).gap    = gap; veh(i).dv = dv;

        elseif i==2 % SV1: after trigger; latch-based following (no TTC oscillation)
            % 1) Base acceleration
            ax_cmd = 0;
            if veh(i).accel_override.active
                ax_cmd = cfg.sv1_ax_on_trigger;  % +2
            end

            % 2) Target-lane leader (including cut-in consideration)
            lane4SV1 = veh(i).lane;
            if veh(i).lc.active
                lane4SV1 = veh(i).lc.targetLane; 
            end
            [leadSV1, ~] = findLeadFollow(veh, i, lane4SV1, cfg);

            % 3) Latch-based following (with hysteresis)
            if cfg.sv1_follow.enable && ~isempty(leadSV1)
                gap = max(0.1, leadSV1.x - leadSV1.L - veh(i).x);
                dv  = veh(i).v - leadSV1.v;                % positive = approaching
                s_des = cfg.sv1_follow.s0 + cfg.sv1_follow.Th * veh(i).v;
                e_gap = gap - s_des;                       % positive=too large; negative=too small

                % —— Enter/exit conditions (with hysteresis)
                if ~veh(i).follow.active
                    if e_gap < cfg.sv1_follow.enter_margin
                        veh(i).follow.active = true;       % too close → enter following mode
                    end
                else
                    % exit only when clearly back to safe distance and not closing (dv<=0)
                    if (e_gap > cfg.sv1_follow.exit_margin) && (dv <= 0)
                        veh(i).follow.active = false;
                    end
                end

                % 4) If in following mode: use PD control to override accel (suppress TTC jitter)
                if veh(i).follow.active
                    a_pd = cfg.sv1_follow.Kp * e_gap + ...
                           cfg.sv1_follow.Kd * (leadSV1.v - veh(i).v);
                    ax_cmd = clamp(a_pd, cfg.sv1_follow.ax_min, cfg.sv1_follow.ax_max);
                end
            end

            % 5) Extreme fallback TTC emergency brake
            if cfg.sv1_use_TTC_EBS && ~isempty(leadSV1)
                gap = max(0.1, leadSV1.x - leadSV1.L - veh(i).x);
                dv  = veh(i).v - leadSV1.v; % positive = approaching
                if dv > 0
                    TTC = gap / max(dv, 0.1);
                    if TTC < cfg.sv1_ebs.tau_hard
                        ax_cmd = min(ax_cmd, -cfg.sv1_ebs.a_hard);
                    elseif TTC < cfg.sv1_ebs.tau_soft
                        ax_cmd = min(ax_cmd, -cfg.sv1_ebs.b_soft);
                    end
                end
            end

            veh(i).ax_cmd = ax_cmd;
            veh(i).gap = NaN; veh(i).dv = NaN;

        else       % i>=3: SV2 and trucks: constant speed
            veh(i).ax_cmd = 0;
            veh(i).gap = NaN; veh(i).dv = NaN;
        end
    end

    % —— Integrate (x,v)
    for i = 1:numel(veh)
        lo = -5.0; hi = veh(i).idm.aMax;
        if i==2
            % comfort limits for following mode; allow stronger braking if TTC fallback enabled
            lo = cfg.sv1_follow.ax_min;
            if cfg.sv1_use_TTC_EBS
                lo = min(lo, -cfg.sv1_ebs.a_hard);
            end
            hi = max(hi, cfg.sv1_ax_on_trigger);  % allow up to +2
        end
        a  = clamp(veh(i).ax_cmd, lo, hi);

        veh(i).v = max(0.0, veh(i).v + cfg.dt*a);
        if i==2
            veh(i).v = min(veh(i).v, cfg.sv1_vmax);
        end  % blue car velocity upper limit
        veh(i).x = veh(i).x + cfg.dt*veh(i).v;
    end

    % —— Lateral update (if in lane-change)
    for i = 1:numel(veh)
        if veh(i).lc.active
            [veh(i), done] = lc_update_lateral(veh(i), cfg);
            if done
                veh(i).lane = veh(i).lc.targetLane;
                veh(i).lc.active = false;
            end
        else
            veh(i).y = cfg.yc(veh(i).lane);  % lane center
        end
    end

    % —— Log data
    for i = 1:numel(veh)
        veh(i).log.t(n)    = t;
        veh(i).log.x(n)    = veh(i).x;
        veh(i).log.y(n)    = veh(i).y;
        veh(i).log.v(n)    = veh(i).v;
        veh(i).log.ax(n)   = veh(i).ax_cmd;
        veh(i).log.lane(n) = veh(i).lane;
        veh(i).log.alat(n) = veh(i).lc.active * sign(veh(i).lc.dir);
    end
end

%% ========= OUTPUT =========
baseline.cfg   = cfg;
baseline.idm   = idm;
baseline.mobil = mobil;
baseline.veh   = veh;
end

%% ========================== LOCAL FUNCTIONS ==========================
function v = clamp(v, lo, hi)
v = max(lo, min(hi, v));
end

function v = laneCenterY(lane, cfg)
    v = cfg.yc(max(1, min(cfg.nLane, lane)));
end

function [lead, foll] = findLeadFollow(veh, iSelf, lane, cfg)
    lead = []; foll = [];
    xs   = veh(iSelf).x;
    for j = 1:numel(veh)
        if j==iSelf 
            continue;
        end
        if veh(j).lane ~= lane && ~veh(j).lc.active
            continue; 
        end
        % cars changing lane are considered as in their target lane
        if veh(j).lc.active && veh(j).lc.targetLane~=lane
            continue; 
        end
        if veh(j).x > xs
            if isempty(lead) || veh(j).x < lead.x
                lead = veh(j); 
            end
        else
            if isempty(foll) || veh(j).x > foll.x
                foll = veh(j); 
            end
        end
    end
end

function [a, gap, dv] = idm_accel(me, lead, cfg)
    p = me.idm;
    if isempty(lead)
        gap = inf; vlead = me.v;
    else
        gap = max(0.1, lead.x - lead.L - me.x);
        vlead = lead.v;
    end
    dv = me.v - vlead; % positive = approaching
    sStar = p.s0 + me.v*p.T + me.v*dv/(2*sqrt(p.aMax*p.bComf)+1e-6);
    a = p.aMax * (1 - (me.v/p.v0)^p.delta - (sStar/gap)^2);
    a = max(-5.0, min(a, p.aMax));
end

function veh = mobil_decide_and_trigger(veh, idx, cfg, mobil, t)
    ego = veh(idx);
    curLane = ego.lane;

    % Candidate target lanes (Lane3 excluded for EV)
    cand = [];
    if curLane>1
        cand = [cand, curLane-1];
    end   % left
    if curLane<cfg.nLane
        cand = [cand, curLane+1];
    end % right
    if cfg.ev_forbid_lane3 && idx==1
        cand = cand(cand ~= cfg.truckLaneID);      % exclude rightmost truck lane
    end

    bestGain = -inf;
    bestLane = curLane;
    bestDir  = 0;

    for laneTarget = cand
        dir = sign(laneTarget - curLane); % +1 left, -1 right

        % lead/follow in current and target lanes
        [lead_old, foll_old] = findLeadFollow(veh, idx, curLane,  cfg);
        [lead_new, foll_new] = findLeadFollow(veh, idx, laneTarget, cfg);

        % ego gain
        a_ego_old = idm_accel(ego, lead_old, cfg); 
        a_ego_old = a_ego_old(1);
        a_ego_new = idm_accel(ego, lead_new, cfg); 
        a_ego_new = a_ego_new(1);
        dA_ego = a_ego_new - a_ego_old;

        % target-lane follower’s gain/safety
        if isempty(foll_new)
            dA_fnew = 0;  
            a_fnew_after = inf;
        else
            a_fnew_before = idm_accel(foll_new, ...
                findLeadFollow_asLeader(veh, foll_new, laneTarget, cfg), cfg);
            a_fnew_before = a_fnew_before(1);
            a_fnew_after  = idm_accel(foll_new, ego, cfg);
            a_fnew_after  = a_fnew_after(1);
            dA_fnew = a_fnew_after - a_fnew_before;
        end

        % original-lane follower’s gain/safety
        if isempty(foll_old)
            dA_fold = 0;  
            a_fold_after = inf;
        else
            a_fold_before = idm_accel(foll_old, ego, cfg);
            a_fold_before = a_fold_before(1);
            a_fold_after  = idm_accel(foll_old, lead_old, cfg);
            a_fold_after  = a_fold_after(1);
            dA_fold = a_fold_after - a_fold_before;
        end

        % MOBIL criterion
        gain = dA_ego + mobil.p*(dA_fnew + dA_fold) - mobil.bias_keep;

        % bias when entering lane3 (soft penalty)
        if cfg.avoidLane3 && laneTarget==3
            gain = gain + cfg.biasLane3;
        end

        % safety constraint: follower braking not exceeding b_safe
        safeOK = true;
        if ~isinf(a_fnew_after)
            safeOK = safeOK && (a_fnew_after >= -mobil.b_safe); 
        end

        if ~isinf(a_fold_after)
            safeOK = safeOK && (a_fold_after >= -mobil.b_safe);
        end

        if (gain > mobil.a_thr) && safeOK && (gain > bestGain)
            bestGain = gain;
            bestLane = laneTarget; 
            bestDir = dir;
        end
    end

    % trigger lane change (duration controlled by cfg.Tlc)
    if bestLane ~= curLane
        veh(idx).lc.active     = true;
        veh(idx).lc.t0         = t;
        veh(idx).lc.dir        = bestDir;
        veh(idx).lc.y0         = veh(idx).y;
        veh(idx).lc.y1         = laneCenterY(bestLane, cfg);
        veh(idx).lc.targetLane = bestLane;
    end
end

function [leadAsLeader] = findLeadFollow_asLeader(veh, me, lane, cfg)
    xs = me.x;
    leadAsLeader = [];
    for j = 1:numel(veh)
        if veh(j).lane ~= lane && ~veh(j).lc.active
            continue;
        end
        if veh(j).lc.active && veh(j).lc.targetLane~=lane
            continue; 
        end
        if veh(j).x > xs
            if isempty(leadAsLeader) || veh(j).x < leadAsLeader.x
                leadAsLeader = veh(j);
            end
        end
    end
end

function [car, done] = lc_update_lateral(car, cfg)
% quintic trajectory (only y), from y0 -> y1 over Tlc; larger Tlc = slower LC
    done = false;
    if ~isfield(car.lc,'tau')
        car.lc.tau=0; 
    end
    car.lc.tau = car.lc.tau + cfg.dt;
    tau = max(0,min(1, car.lc.tau/cfg.Tlc));
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    car.y = (1-s)*car.lc.y0 + s*car.lc.y1;
    if tau >= 1-1e-6
        done = true;
        car.lc.tau = 0;
    end
end

function S = veh_template(idm, cfg)
S = struct( ...
  'name','', 'lane',1, 'x',0, 'y',0, 'v',0, ...
  'is_EGOMOBIL',false, ...
  'idm', idm, 'ax_cmd',0, 'L',cfg.Lveh, 'v0_schedule',[], ...
  'lc', struct('active',false,'t0',0,'dir',0,'T',cfg.Tlc, ...
               'y0',0,'y1',0,'x0',0,'targetLane',1,'tau',0), ...
  'gap',NaN,'dv',NaN, ...
  'log', struct('t',[],'x',[],'y',[],'v',[],'ax',[],'lane',[],'alat',[]) );
end

function veh = initVeh(veh, idx, name, lane, x0, y0, v0)
veh(idx).name = name;
veh(idx).lane = lane;
veh(idx).x = x0;  veh(idx).y = y0;  veh(idx).v = v0;
veh(idx).idm.v0 = max(v0, veh(idx).idm.v0);   % desired speed slightly higher than current
veh(idx).lc.targetLane = lane;
end
