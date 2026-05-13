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
fps        = 25;
frame_skip = max(1, round(1 / (fps * h)));
frame_idx  = 1:frame_skip:N;
n_frames   = length(frame_idx);

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
ax_anim = uiaxes(gl);
ax_anim.Layout.Row = 1;  ax_anim.Layout.Column = 1;
hold(ax_anim, 'on');
lim = (p.l1 + p.l2) * 1.25;
xlim(ax_anim, [-lim lim]);
ylim(ax_anim, [-lim lim]);
axis(ax_anim, 'equal');
grid(ax_anim, 'on');
ax_anim.Color      = [0.10 0.10 0.10];
ax_anim.XColor     = [0.65 0.65 0.65];
ax_anim.YColor     = [0.65 0.65 0.65];
ax_anim.GridColor  = [0.28 0.28 0.28];
ax_anim.GridAlpha  = 1;
ax_anim.XLabel.String = 'x [m]';
ax_anim.YLabel.String = 'y [m]';
ax_anim.Title.String  = 't = 0.00 s';
ax_anim.Title.Color   = [0.85 0.85 0.85];
ax_anim.FontSize      = 10;

% Stick figure: two arm lines + three balls
h_arm1  = line(ax_anim, [0, jx(1)],    [0, jy(1)],    'Color', [0.35 0.70 1.00], 'LineWidth', 3);
h_arm2  = line(ax_anim, [jx(1), tx(1)],[jy(1), ty(1)], 'Color', [1.00 0.55 0.15], 'LineWidth', 3);
h_org = plot(ax_anim, 0,     0,     'o', 'MarkerSize', 11, ...
    'MarkerFaceColor', [0.85 0.85 0.85], 'MarkerEdgeColor', 'none');  %#ok<NASGU>
h_jnt = plot(ax_anim, jx(1), jy(1), 'o', 'MarkerSize', 9, ...
    'MarkerFaceColor', [0.35 0.70 1.00], 'MarkerEdgeColor', 'none');
h_tip = plot(ax_anim, tx(1), ty(1), 'o', 'MarkerSize', 11, ...
    'MarkerFaceColor', [1.00 0.55 0.15], 'MarkerEdgeColor', 'none');

%% Time-series axes (right, stacked)
gl_right = uigridlayout(gl, [3 1]);
gl_right.Layout.Row = 1;  gl_right.Layout.Column = 2;
gl_right.RowHeight   = {'1x','1x','1x'};
gl_right.RowSpacing  = 6;
gl_right.Padding     = [0 0 0 0];
gl_right.BackgroundColor = [0.13 0.13 0.13];

ax_th1 = uiaxes(gl_right);  ax_th1.Layout.Row = 1;
ax_th2 = uiaxes(gl_right);  ax_th2.Layout.Row = 2;
ax_psi = uiaxes(gl_right);  ax_psi.Layout.Row = 3;

colours = {[0.35 0.70 1.00], [0.20 0.90 0.45], [1.00 0.55 0.15]};
labels  = {'\theta_1 [deg]', '\theta_2 [deg]', '\psi [deg]'};
signals = {th1, th2, psi};
ts_axes  = [ax_th1, ax_th2, ax_psi];

for i = 1:3
    ax = ts_axes(i);
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
cTh1 = plot(ax_th1, [t(1) t(1)], [-ypad  ypad], '--', 'Color', [1 1 1 0.6], 'LineWidth', 1);
cTh2 = plot(ax_th2, [t(1) t(1)], [-ypad  ypad], '--', 'Color', [1 1 1 0.6], 'LineWidth', 1);
cPsi = plot(ax_psi,  [t(1) t(1)], [-ypad  ypad], '--', 'Color', [1 1 1 0.6], 'LineWidth', 1);

%% Controls (bottom, spanning both columns)
gl_ctrl = uigridlayout(gl, [1 3]);
gl_ctrl.Layout.Row    = 2;
gl_ctrl.Layout.Column = [1 2];
gl_ctrl.ColumnWidth   = {90, '1x', 70};
gl_ctrl.Padding       = [4 6 4 6];
gl_ctrl.BackgroundColor = [0.13 0.13 0.13];

btn_play = uibutton(gl_ctrl, 'push', 'Text', '▶  Play');
btn_play.Layout.Row    = 1;
btn_play.Layout.Column = 1;
btn_play.BackgroundColor = [0.22 0.22 0.22];
btn_play.FontColor       = [0.90 0.90 0.90];
btn_play.FontSize        = 12;

sld = uislider(gl_ctrl, 'Limits', [1 n_frames], 'Value', 1, ...
    'MajorTicks', [], 'MinorTicks', []);
sld.Layout.Row    = 1;
sld.Layout.Column = 2;

lbl_time = uilabel(gl_ctrl, 'Text', sprintf('%.2f s', t(1)), ...
    'HorizontalAlignment', 'center', 'FontColor', [0.80 0.80 0.80], 'FontSize', 11);
lbl_time.Layout.Row    = 1;
lbl_time.Layout.Column = 3;

%% Shared mutable state
state.frame   = 1;
state.playing = false;
state.tmr     = [];

%% Wire callbacks (defined as nested functions below)
btn_play.ButtonPushedFcn = @onPlayPause;
sld.ValueChangedFcn     = @onSlider;
fig.CloseRequestFcn     = @onClose;

updateFrame(1);

% =========================================================================
%  Nested functions — share workspace with animate_pendulum
% =========================================================================

    function updateFrame(f)
        f = max(1, min(n_frames, round(f)));
        state.frame = f;
        k = frame_idx(f);

        % Stick figure
        h_arm1.XData  = [0,    jx(k)];    h_arm1.YData  = [0,    jy(k)];
        h_arm2.XData  = [jx(k), tx(k)];   h_arm2.YData  = [jy(k), ty(k)];
        h_jnt.XData = jx(k);             h_jnt.YData = jy(k);
        h_tip.XData = tx(k);             h_tip.YData = ty(k);
        ax_anim.Title.String = sprintf('t = %.2f s', t(k));

        % Cursors
        cTh1.XData = [t(k) t(k)];
        cTh2.XData = [t(k) t(k)];
        cPsi.XData  = [t(k) t(k)];

        sld.Value  = f;
        lbl_time.Text  = sprintf('%.2f s', t(k));

        drawnow limitrate;
    end

    function timerStep(~, ~)
        if ~isvalid(fig)
            stop(state.tmr);
            return;
        end
        next = state.frame + 1;
        if next > n_frames
            stop(state.tmr);
            btn_play.Text  = '▶  Play';
            state.playing = false;
        else
            updateFrame(next);
        end
    end

    function onPlayPause(~, ~)
        if state.playing
            stop(state.tmr);
            btn_play.Text  = '▶  Play';
            state.playing = false;
        else
            if state.frame >= n_frames
                updateFrame(1);
            end
            if isempty(state.tmr) || ~isvalid(state.tmr)
                state.tmr = timer('ExecutionMode', 'fixedRate', ...
                    'Period', max(0.04, round(1/fps, 3)), ...
                    'TimerFcn', @timerStep);
            end
            start(state.tmr);
            btn_play.Text  = '⏸  Pause';
            state.playing = true;
        end
    end

    function onSlider(~, ~)
        if state.playing
            stop(state.tmr);
            btn_play.Text  = '▶  Play';
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
