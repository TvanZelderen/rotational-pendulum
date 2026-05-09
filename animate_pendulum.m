function animate_pendulum(simout, p)
% Animate rotational pendulum simulation output.
%
% Usage:  animate_pendulum(simout, p)
%   simout — Timeseries, 5 columns: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg]
%   p      — parameter struct from pendulum_params.m  (needs p.l1, p.l2)

%% Extract data
t   = simout.Time;
y   = simout.Data;
th1 = y(:,1);
th2 = y(:,3);
psi = y(:,5);
N   = length(t);
h   = mean(diff(t));

%% Frame decimation for ~25 fps real-time playback
fps       = 25;
frameSkip = max(1, round(1 / (fps * h)));
frameIdx  = 1:frameSkip:N;
nFrames   = length(frameIdx);

%% Precompute arm endpoint positions [m]
th1r = deg2rad(th1);
th2r = deg2rad(th2);
jx   = p.l1 * sin(th1r);                     % joint x
jy   = -p.l1 * cos(th1r);                    % joint y
tx   = jx + p.l2 * sin(th1r + th2r);         % tip x
ty   = jy - p.l2 * cos(th1r + th2r);         % tip y

%% Figure and layout
fig = uifigure('Name', 'Pendulum Animation', 'Position', [80 80 1200 640]);
fig.Color = [0.13 0.13 0.13];

gl = uigridlayout(fig, [2 2]);
gl.RowHeight      = {'1x', 56};
gl.ColumnWidth    = {'2x', '3x'};
gl.Padding        = [12 12 12 8];
gl.RowSpacing     = 8;
gl.ColumnSpacing  = 12;
gl.BackgroundColor = [0.13 0.13 0.13];

%% Animation axes (left)
axA = uiaxes(gl);
axA.Layout.Row = 1;  axA.Layout.Column = 1;
hold(axA, 'on');
lim = (p.l1 + p.l2) * 1.25;
xlim(axA, [-lim lim]);
ylim(axA, [-lim lim]);
axis(axA, 'equal');
grid(axA, 'on');
axA.Color      = [0.10 0.10 0.10];
axA.XColor     = [0.65 0.65 0.65];
axA.YColor     = [0.65 0.65 0.65];
axA.GridColor  = [0.28 0.28 0.28];
axA.GridAlpha  = 1;
axA.XLabel.String = 'x [m]';
axA.YLabel.String = 'y [m]';
axA.Title.String  = 't = 0.00 s';
axA.Title.Color   = [0.85 0.85 0.85];
axA.FontSize      = 10;

% Stick figure: two arm lines + three balls
hA1  = line(axA, [0, jx(1)],    [0, jy(1)],    'Color', [0.35 0.70 1.00], 'LineWidth', 3);
hA2  = line(axA, [jx(1), tx(1)],[jy(1), ty(1)], 'Color', [1.00 0.55 0.15], 'LineWidth', 3);
hOrg = plot(axA, 0,     0,     'o', 'MarkerSize', 11, ...
    'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerEdgeColor', 'none');  %#ok<NASGU>
hJnt = plot(axA, jx(1), jy(1), 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.35 0.70 1.00], 'MarkerEdgeColor', 'none');
hTip = plot(axA, tx(1), ty(1), 'o', 'MarkerSize', 11, ...
    'MarkerFaceColor', [1.00 0.55 0.15], 'MarkerEdgeColor', 'none');

%% Time-series axes (right, stacked)
glR = uigridlayout(gl, [3 1]);
glR.Layout.Row = 1;  glR.Layout.Column = 2;
glR.RowHeight   = {'1x','1x','1x'};
glR.RowSpacing  = 6;
glR.Padding     = [0 0 0 0];
glR.BackgroundColor = [0.13 0.13 0.13];

axTh1 = uiaxes(glR);  axTh1.Layout.Row = 1;
axTh2 = uiaxes(glR);  axTh2.Layout.Row = 2;
axPsi = uiaxes(glR);  axPsi.Layout.Row = 3;

colours = {[0.35 0.70 1.00], [0.20 0.90 0.45], [1.00 0.55 0.15]};
labels  = {'\theta_1 [deg]', '\theta_2 [deg]', '\psi [deg]'};
signals = {th1, th2, psi};
tsAxes  = [axTh1, axTh2, axPsi];

for i = 1:3
    ax = tsAxes(i);
    hold(ax, 'on');
    ax.Color     = [0.10 0.10 0.10];
    ax.XColor    = [0.65 0.65 0.65];
    ax.YColor    = [0.65 0.65 0.65];
    ax.GridColor = [0.28 0.28 0.28];
    ax.GridAlpha = 1;
    ax.FontSize  = 9;
    grid(ax, 'on');
    xlim(ax, [t(1) t(end)]);
    plot(ax, t, signals{i}, 'Color', colours{i}, 'LineWidth', 1.2);
    ax.YLabel.String = labels{i};
    ax.YLabel.Color  = [0.75 0.75 0.75];
    ax.XLabel.Color  = [0.65 0.65 0.65];
    if i < 3
        ax.XTickLabel = {};
    else
        ax.XLabel.String = 'Time [s]';
    end
end

% Time cursors — vertical white dashed lines
ypad = 1e1;
cTh1 = plot(axTh1, [t(1) t(1)], [-ypad  ypad], '--', 'Color', [1 1 1 0.6], 'LineWidth', 1);
cTh2 = plot(axTh2, [t(1) t(1)], [-ypad  ypad], '--', 'Color', [1 1 1 0.6], 'LineWidth', 1);
cPsi = plot(axPsi,  [t(1) t(1)], [-ypad  ypad], '--', 'Color', [1 1 1 0.6], 'LineWidth', 1);

%% Controls (bottom, spanning both columns)
glC = uigridlayout(gl, [1 3]);
glC.Layout.Row    = 2;
glC.Layout.Column = [1 2];
glC.ColumnWidth   = {90, '1x', 70};
glC.Padding       = [4 6 4 6];
glC.BackgroundColor = [0.13 0.13 0.13];

btnPlay = uibutton(glC, 'push', 'Text', '▶  Play');
btnPlay.Layout.Row    = 1;
btnPlay.Layout.Column = 1;
btnPlay.BackgroundColor = [0.22 0.22 0.22];
btnPlay.FontColor       = [0.90 0.90 0.90];
btnPlay.FontSize        = 12;

sld = uislider(glC, 'Limits', [1 nFrames], 'Value', 1, ...
    'MajorTicks', [], 'MinorTicks', []);
sld.Layout.Row    = 1;
sld.Layout.Column = 2;

lblT = uilabel(glC, 'Text', sprintf('%.2f s', t(1)), ...
    'HorizontalAlignment', 'center', 'FontColor', [0.80 0.80 0.80], 'FontSize', 11);
lblT.Layout.Row    = 1;
lblT.Layout.Column = 3;

%% Shared mutable state
state.frame   = 1;
state.playing = false;
state.tmr     = [];

%% Wire callbacks (defined as nested functions below)
btnPlay.ButtonPushedFcn = @onPlayPause;
sld.ValueChangedFcn     = @onSlider;
fig.CloseRequestFcn     = @onClose;

updateFrame(1);

% =========================================================================
%  Nested functions — share workspace with animate_pendulum
% =========================================================================

    function updateFrame(f)
        f = max(1, min(nFrames, round(f)));
        state.frame = f;
        k = frameIdx(f);

        % Stick figure
        hA1.XData  = [0,    jx(k)];    hA1.YData  = [0,    jy(k)];
        hA2.XData  = [jx(k), tx(k)];   hA2.YData  = [jy(k), ty(k)];
        hJnt.XData = jx(k);             hJnt.YData = jy(k);
        hTip.XData = tx(k);             hTip.YData = ty(k);
        axA.Title.String = sprintf('t = %.2f s', t(k));

        % Cursors
        cTh1.XData = [t(k) t(k)];
        cTh2.XData = [t(k) t(k)];
        cPsi.XData  = [t(k) t(k)];

        sld.Value  = f;
        lblT.Text  = sprintf('%.2f s', t(k));

        drawnow limitrate;
    end

    function timerStep(~, ~)
        if ~isvalid(fig)
            stop(state.tmr);
            return;
        end
        next = state.frame + 1;
        if next > nFrames
            stop(state.tmr);
            btnPlay.Text  = '▶  Play';
            state.playing = false;
        else
            updateFrame(next);
        end
    end

    function onPlayPause(~, ~)
        if state.playing
            stop(state.tmr);
            btnPlay.Text  = '▶  Play';
            state.playing = false;
        else
            if state.frame >= nFrames
                updateFrame(1);
            end
            if isempty(state.tmr) || ~isvalid(state.tmr)
                state.tmr = timer('ExecutionMode', 'fixedRate', ...
                    'Period', max(0.04, round(1/fps, 3)), ...
                    'TimerFcn', @timerStep);
            end
            start(state.tmr);
            btnPlay.Text  = '⏸  Pause';
            state.playing = true;
        end
    end

    function onSlider(~, ~)
        if state.playing
            stop(state.tmr);
            btnPlay.Text  = '▶  Play';
            state.playing = false;
        end
        updateFrame(round(sld.Value));
    end

    function onClose(~, ~)
        if ~isempty(state.tmr) && isvalid(state.tmr)
            stop(state.tmr);
            delete(state.tmr);
        end
        delete(fig);
    end

end
