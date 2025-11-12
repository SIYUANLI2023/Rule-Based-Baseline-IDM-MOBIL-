function playback_lanechange_from_baseline(baseline, gifname, drawSpeedFig)
% Robust playback (GIF + optional speed figure) for baseline IDM+MOBIL logs

% Allow zero-arg call 
if nargin < 1 || isempty(baseline)
    if evalin('base','exist(''baseline'',''var'')')
        baseline = evalin('base','baseline');
    else
        error('lack baseline。run baseline = run_case2_baseline_IDM_MOBIL();');
    end
end

if nargin < 2 || isempty(gifname)
    gifname = 'gif/play_back.gif'; 
end

if nargin < 3 || isempty(drawSpeedFig)
    drawSpeedFig = true; 
end

[fileplace,~,~] = fileparts(gifname); 
if ~exist(fileplace,'dir')
    mkdir(fileplace);
end

veh = baseline.veh;
cfg = baseline.cfg;

% ---------- Map baseline to an example-like structure ----------
ego_vehicle    = makeVehicleLikeExample(veh(1), cfg);
other_vehicles = arrayfun(@(v) makeVehicleLikeExample(v, cfg), veh(2:end));

% ---------- Colors ----------
nVeh = numel(other_vehicles);
color_fixed = [ 0.00 0.45 0.74; 0.47 0.67 0.19; 0.30 0.30 0.30; 0.45 0.20 0.65 ];
color_auto  = lines(max(nVeh,7));

% ---------- Timeline / frame count ----------
if isfield(cfg,'dt') && ~isempty(cfg.dt)
    dt = cfg.dt;
else
    tlog = veh(1).log.t(:);
    dt = max(0.001, median(diff(tlog)));
end

% Use the longest length for playback; vehicles with shorter logs hold the last frame
if isfield(cfg,'dt') && ~isempty(cfg.dt)
    dt = cfg.dt;
else
    tlog = veh(1).log.t(:);
    dt = max(0.001, median(diff(tlog)));
end

% If it's a cell, flatten; then force a 1×N row vector to avoid horzcat mismatch
if iscell(other_vehicles)
    other_vehicles = [other_vehicles{:}]; 
end
other_vehicles = other_vehicles(:).';  

% Use the longest length for playback; if shorter, “freeze” at the last frame
len_all = size(ego_vehicle.state_log, 2);
for j = 1:numel(other_vehicles)
    if ~isempty(other_vehicles(j).state_log)
        len_all(end+1) = size(other_vehicles(j).state_log, 2); %#ok<AGROW>
    end
end
nFrame = max(len_all);


% ===== Lane lines (centers + lane width)=====
w      = cfg.wLane;
yc     = cfg.yc(:)';                 % e.g. [4, 0, -4]
y_top  = max(yc) + w/2;              % 6
y_bot  = min(yc) - w/2;              % -6
y_in   = (yc(1:end-1) + yc(2:end))/2;% [2, -2]
y_lines = [y_bot, sort(y_in,'ascend'), y_top];   


hFig = figure('Name','Lane-Change Playback','Units','pixels', ...
              'Position',[200 100 1600 400], 'Color','w','Resize','on');
filename = gifname;

for idx = 1:nFrame
    clf(hFig);
    ax = axes('Position',[0.03 0.25 0.94 0.45]); hold(ax,'on'); box(ax,'on');
    axis(ax,'equal'); pbaspect(ax,[10 1 1]);
    xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
    set(ax,'FontName','Times New Roman','FontSize',26,'LineWidth',5);

    % Lane boundaries: outer solid, inner dashed
    yticks(ax, y_lines); ylim(ax,[y_bot y_top]);
    for y = y_lines
        if y==y_bot || y==y_top
            plot(ax,[-150 1500],[y y],'k','LineWidth',5,'HandleVisibility','off');
        else
            plot(ax,[-150 1500],[y y],'k--','LineWidth',5,'HandleVisibility','off');
        end
    end

    % Other vehicles (safe frame)
    for j = 1:nVeh
        thisColor = (j<=size(color_fixed,1)) .* color_fixed(j,:) + ...
                    (j>size(color_fixed,1))  .* color_auto(j,:);
        if j>=3, Ldraw = 12.0; else
            Ldraw = other_vehicles(j).param.l_fc + other_vehicles(j).param.l_rc;
        end
        st = safe_frame(other_vehicles(j).state_log, idx);
        plot_car_noshift(st, Ldraw, other_vehicles(j).param.width, thisColor);
    end

    % Ego
    stEgo = safe_frame(ego_vehicle.state_log, idx);
    plot_car_noshift(stEgo, ego_vehicle.param.l_fc + ego_vehicle.param.l_rc, ...
                     ego_vehicle.param.width, [1 0 0]);

    % Viewport follows Ego
    xE = stEgo(1); xlim(ax,[xE-80 , xE+50]);

    % Write GIF frame
    drawnow;
    t=(idx-1)*dt
    frame   = getframe(hFig);
    [A,map] = rgb2ind(frame2im(frame),256);
    if idx==1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',max(0.01,dt));
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',max(0.01,dt));
    end
end

% Keep the following two lines unchanged
yticks(ax, y_lines);
ylim(ax,[y_bot y_top]);

if drawSpeedFig
    key = [];
    if evalin('base','exist(''key'',''var'')'), key = evalin('base','key'); end
    plot_velocity_from_baseline(baseline, key);
end
end

% ====== Utilities ======
function st = safe_frame(state_log, idx)
% Safe frame extraction: if index exceeds length, use the last frame
n = size(state_log,2);
st = state_log(:, min(idx,n));
end

function V = makeVehicleLikeExample(v, cfg)
L  = cfg.Lveh;
fc = L/2; rc = L/2; wid = 1.9;
x  = v.log.x(:)';              % 1×N
y  = v.log.y(:)';              % 1×N
vx = max(v.log.v(:)', 1e-3);   % 1×N
% Estimate lateral speed via finite difference of y and compute heading
t  = v.log.t(:)'; dt = max(0.001, median(diff(t)));
vy = [diff(y)/dt, 0];
theta = atan2(vy, vx);

V.param.l_fc   = fc;
V.param.l_rc   = rc;
V.param.width  = wid;
V.lanes.lane_width = cfg.wLane;
V.state_log    = [x; y; theta; vx];
end

function plot_car_noshift(state,L,H,color)
theta = state(3);
R     = [cos(theta) -sin(theta); sin(theta) cos(theta)];
base  = [-L/2 -H/2; L/2 -H/2; L/2 H/2; -L/2 H/2]';
verts = R*base + [state(1); state(2)];
patch('XData',verts(1,:), 'YData',verts(2,:), ...
      'EdgeColor','k','FaceColor',color,'LineWidth',5,'HandleVisibility','off');
end
