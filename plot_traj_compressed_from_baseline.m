function hFig = plot_traj_compressed_from_baseline(baseline, varargin)
% Compressed trajectory plot compatible with baseline structure
% (lane lines derived from cfg.yc / cfg.wLane)
% Name-Value:
%   'k'       x compression factor (default 7)
%   'y_off'   y offset (default 0) is not used in this situation.
%   'stepE'   Ego sampling step (default 1)
%   'stepTV'  TV sampling step (default 20)
%   'worldX'  real-world x display range [xmin xmax] (default [0 1250])

% ----------- Parse options -----------
p = inputParser;
p.addParameter('k',7,@(x)isnumeric(x)&&isscalar(x));
p.addParameter('y_off',0,@(x)isnumeric(x)&&isscalar(x));
p.addParameter('stepE',1,@(x)isnumeric(x)&&isscalar(x));
p.addParameter('stepTV',20,@(x)isnumeric(x)&&isscalar(x));
p.addParameter('worldX',[0 1250],@(x)isnumeric(x)&&numel(x)==2);
p.parse(varargin{:});
k      = p.Results.k;
y_off  = p.Results.y_off;
stepE  = p.Results.stepE;
stepTV = p.Results.stepTV;
worldX = p.Results.worldX(:)';   % [xmin xmax]

% ----------- Colors -----------
cE0   = [1.00 0.70 0.70];
cE1   = [0.80 0.00 0.00];
colTV1 = [0.00 0.45 0.74];  % blue
colTV2 = [0.47 0.67 0.19];  % green

% ----------- Data -----------
veh = baseline.veh;
cfg = baseline.cfg;

% Vehicles (logs)
xE = veh(1).log.x(:)'; 
yE = veh(1).log.y(:)';

xT = []; yT = [];
if numel(veh)>=2 && ~isempty(veh(2).log.x)
    xT = veh(2).log.x(:)'; yT = veh(2).log.y(:)';
end

xG = []; yG = [];
if numel(veh)>=3 && ~isempty(veh(3).log.x)
    xG = veh(3).log.x(:)'; yG = veh(3).log.y(:)';
end

% Compress/shift (vehicles)
xE = (xE - worldX(1))./k;   
yE = yE - y_off;
if ~isempty(xT)
    xT = (xT - worldX(1))./k;
    yT = yT - y_off; 
end
if ~isempty(xG)
    xG = (xG - worldX(1))./k; 
    yG = yG - y_off; 
end

% Lane geometry
w    = cfg.wLane;
yc   = cfg.yc(:)';                   % e.g. [4 0 -4]
y_top = max(yc) + w/2;               % +6
y_bot = min(yc) - w/2;               % -6
y_in  = (yc(1:end-1)+yc(2:end))/2;   % internal dashed lines: [+2, -2]

% Compressed x-range
xlimC = (worldX - worldX(1))./k;
epsx  = 0.01*diff(xlimC);            % small right margin so right border is visible

% ----------- Figure / Axes -----------
hFig = figure('Position',[200 200 1100 220],'Color','w');
ax   = axes('Parent',hFig); hold(ax,'on'); box(ax,'on');

% Borders: top/bottom (solid) + left/right vertical edges
plot(ax, xlimC, [1 1]*(y_bot - y_off), 'k', 'LineWidth',3);                  % bottom
plot(ax, xlimC, [1 1]*(y_top - y_off), 'k', 'LineWidth',3);                  % top
plot(ax, [xlimC(1) xlimC(1)], [y_bot y_top]-y_off, 'k', 'LineWidth',3);      % left  (FIX)
plot(ax, [xlimC(2)+epsx xlimC(2)+epsx], [y_bot y_top]-y_off, 'k', 'LineWidth',3); % right

% Internal dashed lines (lane separators)
for jj = 1:numel(y_in)
    plot(ax, xlimC, [1 1]*(y_in(jj) - y_off), 'k--', 'LineWidth',3);
end

% Ego trajectory with color gradient
N = numel(xE);
grad  = (0:N-1)'./max(N-1,1);
cmapE = cE0 + grad.*(cE1 - cE0);
for ii = 1:stepE:N
    scatter(ax, xE(ii), yE(ii), 45, 'filled', ...
        'MarkerFaceColor',cmapE(ii,:), ...
        'MarkerEdgeColor','k','LineWidth',1);
end

% TV1 / TV2
if ~isempty(xT)
    scatter(ax, xT(1:stepTV:end), yT(1:stepTV:end), ...
        65, colTV1, 'filled', 'MarkerEdgeColor','k','LineWidth',1);
end
if ~isempty(xG)
    scatter(ax, xG(1:stepTV:end), yG(1:stepTV:end), ...
        65, colTV2, 'filled', 'MarkerEdgeColor','k','LineWidth',1);
end

% ----------- Axes appearance -----------
xlim(ax, [xlimC(1), xlimC(2)+epsx]);
ylim(ax, [y_bot y_top] - y_off);
pbaspect(ax, [10 1 1]);

% X ticks: map to real-world x
tick_world = niceTicks(worldX(1), worldX(2), 6);
tick_comp  = (tick_world - worldX(1))./k;
set(ax, 'XTick', tick_comp, 'XTickLabel', compose('%g', tick_world));

% Y ticks: exactly at [-6, -2, +2, +6]
tick_vals = sort([y_bot, y_in, y_top]);
set(ax, 'YTick', tick_vals - y_off, 'YTickLabel', compose('%g', tick_vals));

set(ax, 'FontName','Times New Roman', 'FontSize',16, ...
        'LineWidth',3, 'TickLength',[0 0], 'TickDir','out');

xlabel(ax,'$x$ (m)','Interpreter','latex','FontSize',16);
ylabel(ax,'$y$ (m)','Interpreter','latex','FontSize',16);
end

% --------- helper: “nice” linear ticks ---------
function ticks = niceTicks(a,b,nTarget)
    if a>b
        [a,b] = deal(b,a);
    end
    span = b-a;
    if span<=0
        ticks = [a b];
        return; 
    end
    raw = span/max(nTarget-1,1);
    mag = 10^floor(log10(raw));
    steps = [1 2 2.5 5 10]*mag;
    [~,i] = min(abs(steps-raw));
    step = steps(i);
    first = ceil(a/step)*step;
    last  = floor(b/step)*step;
    ticks = first:step:last;
    if isempty(ticks)
        ticks = [a b];
    end
end
