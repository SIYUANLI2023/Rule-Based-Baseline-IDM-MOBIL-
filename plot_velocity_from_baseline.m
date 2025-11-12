function hFig = plot_velocity_from_baseline(baseline, keyPts)
% Plot velocity profiles of Ego and surrounding vehicles from baseline results
% Optional: mark key timestamps and corresponding velocity points

veh = baseline.veh;
dt  = baseline.cfg.dt;

% --- Extract length of each vehicle log 
nE = numel(veh(1).log.v);
n1 = (numel(veh)>=2) * numel(veh(2).log.v);
n2 = (numel(veh)>=3) * numel(veh(3).log.v);
nMax = max([nE, n1, n2]);

% --- Unified time axis
time_vec = (0:nMax-1) * dt;

% --- Fill velocity series 
Vego = nan(1,nMax); Vego(1:nE) = veh(1).log.v(:)';
if n1>0
    V1 = nan(1,nMax);
    V1(1:n1) = veh(2).log.v(:)'; 
else
    V1 = []; 
end
if n2>0
    V2 = nan(1,nMax);
    V2(1:n2) = veh(3).log.v(:)';
else
    V2 = []; 
end

% --- Colors
colEgo = [1 0 0];            % red (Ego)
colTV1 = [0.00 0.45 0.74];   % blue (TV1)
colTV2 = [0.47 0.67 0.19];   % green (TV2)

% --- Figure settings
hFig = figure('Name','Speed Curves','Units','pixels', ...
                 'Position',[300 480 1100 500],'Color','w');
axS = axes('Parent',hFig); hold(axS,'on'); box(axS,'on'); grid(axS,'off');
set(axS,'FontName','Times New Roman','FontSize',16, ...
        'LineWidth',3, ...
        'TickLength',[0.01 0], ...          % keep subtle tick marks
        'TickDir','in');

% --- Plot velocity curves
plot(axS, time_vec, Vego, 'Color',colEgo, 'LineWidth',5);
lgd = {'Ego'};
if ~isempty(V1)
    plot(axS, time_vec, V1,   'Color',colTV1,'LineWidth',5);
    lgd{end+1}='TV1';
end
if ~isempty(V2)
    plot(axS, time_vec, V2,   'Color',colTV2,'LineWidth',5);
    lgd{end+1}='TV2';
end

% --- Axes limits and labels
xMax = ceil(time_vec(end))-1;
xlim(axS,[0 xMax]); ylim(axS,[20 40]);
xlabel(axS,'Time (s)'); ylabel(axS,'Velocity (m/s)');
legend(axS, lgd, 'Location','northwest');

% --- Plot key points if provided
if nargin>=2 && ~isempty(keyPts) && isfield(keyPts,'T') && isfield(keyPts,'V')
    pT = keyPts.T(:)'; pV = keyPts.V(:)'; 
    plot(axS, pT, pV, 'o', 'MarkerSize',9, ...
         'MarkerFaceColor',[1 0.85 0.2], 'MarkerEdgeColor','k', ...
         'LineWidth',2, 'HandleVisibility','off');
     for i = 2:6
        plot(axS, [pT(i) pT(i)], [pV(i) 20], 'k--', 'LineWidth',2, ...
             'HandleVisibility','off');
        text(axS, pT(i)+1, 21.5, sprintf('%.2f',pT(i)), ...
             'HorizontalAlignment','center','VerticalAlignment','top', ...
             'FontName','Times New Roman','FontSize',12);   
     end    
end
end
