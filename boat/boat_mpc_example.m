clear;

global safe_rect goal_rect unsafe_rects Ts
safe_rect = [-5000, -6*pi, -2, -5, 5000, 6*pi, 1, 15];
unsafe_rects = [-5000, -6*pi, 4, -4, 5000, 6*pi, 5, -2;
    -5000, -6*pi, 7, 2, 5000, 6*pi, 8, 4;
    -5000, -6*pi, 2, 10, 5000, 6*pi, 3, 12;
    -5000, -6*pi, -5, -5, 5000, 6*pi, -2, 15;
    -5000, -6*pi, 12, -5, 5000, 6*pi, 15, 15];

goal_pt = [0, 0, 10.5, 5];
goal_bound = [5000, pi, 1.5, 10];
goal_rect = [goal_pt - goal_bound, goal_pt + goal_bound];

auto_iterate = true;
plot_full_traj = false;
write_video = false;
debug = false;
write_tex = false;
write_svg = false;

Ts = 0.2;
mean_flow = 6;

u_max = [500, pi/2];
safety_controller = @(x, u) boat_safety_control(x, u);
actual_system = @(t, x, u) boat_in_river(t, x, u, mean_flow);
explore_cost_function = @(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval) dist(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval);
exploit_cost_function = @(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval) dist(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval);


if ~all(goal_rect(5:end) >= goal_rect(1:4))
    error('goal rect defined incorrectly!')
elseif ~all(safe_rect(5:end) >= safe_rect(1:4))
    error('safe rect defined incorrectly!')
else
    num_unsafe = size(unsafe_rects);
    num_unsafe = num_unsafe(1);
    for u=1:num_unsafe
        if ~all(unsafe_rects(u, 5:end) >= unsafe_rects(u, 1:4))
            error('an unsafe rect defined incorrectly!')
        end
    end
end
[sampling_points_X, sampling_points_Y] = meshgrid(-2:12, -6:16);
sampling_points_X = sampling_points_X(:);
sampling_points_Y = sampling_points_Y(:);
sampling_points_U = disturbance_x(sampling_points_X);
sampling_points_V = disturbance_y(sampling_points_Y) - mean_flow;


x_vec = [-1, 0, 1];
y_vec = [13, 14, 15];
obs_x = [];
obs_y = [];
for i = 1:numel(x_vec)
    obs_x = [obs_x; x_vec(i), disturbance_x(x_vec(i))];
    obs_y = [obs_y; y_vec(i), disturbance_y(y_vec(i))];
end
x_space = [-2:0.25:12];
y_space = [-7:0.25:17];

%% MPC
nx = 8; % number of state variables
ny = 1; % number of output variables
nu = 2; % number of input variables
nlobj = nlmpc(nx,ny,nu);

% horizons
blocking_interval = 6; % number of timesteps to execute each control action
control_actions = 5; % number of control actions
nlobj.Ts = Ts/blocking_interval; % these should set prediction horizon to match Ts of control actions
nlobj.PredictionHorizon = control_actions*blocking_interval;
nlobj.ControlHorizon = [ones(1,control_actions) * blocking_interval];

% model system dynamics
nlobj.Model.StateFcn = @(x, u) boat_emb_mpc(x, u, obs_x, obs_y, mean_flow);
% nlobj.Model.StateFcn = @inv_pen_emb_mpc;
% If true, MATLAB automatically discretizes the model for optimization
% using Ts, else Ts is the sample time
nlobj.Model.IsContinuousTime = true; 
nlobj.Model.OutputFcn = @(x,u) x(1);

% cost function
nlobj.Optimization.CustomCostFcn = @(X,U, e, data) -1*explore_cost_function(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval);

% control input limits (MV = manipulated variable)
nlobj.MV(1).Min = -u_max(1);
nlobj.MV(1).Max = u_max(1);
nlobj.MV(2).Min = -u_max(2);
nlobj.MV(2).Max = u_max(2);


% other constraints
% nlobj.Optimization.CustomEqConFcn = @eq_constraints;
nlobj.Optimization.CustomIneqConFcn = @leq_constraints_goal;
nlobj.Optimization.UseSuboptimalSolution = true; % if max number of iterations reached w/out solving, still give suboptimal
nlobjBest.Optimization.SolverOptions.UseParallel = true; % enable parallelization

% define initial conditions, verify correct construction
x0 = [0; 0; -1; 14]; %x0 = [0; 0; 0; 14];
u0 = [0; 0];
state_uncertainty = [0.1; 0.1; 0; 0];%[0.01; 0.01; 0.01; 0.01; 0.01; 0.01];
x0_rect = [x0-state_uncertainty; x0+state_uncertainty];
validateFcns(nlobj, x0_rect, u0);
cur_x = x0;

% initial control strategy is entirely safety actions
planned_actions = zeros(control_actions, 2);
% boolean to trigger safety action, as the actual value needs to be calculated when executed
trigger_safety = ones(control_actions, 1); 
x_traj = [];
storedXopt = [];
step = 1;
next_step = step;
path_to_goal_found = false;

if write_video
    % Video creation objects
    writer1 = VideoWriter('state_boat_both', 'UNCOMPRESSED AVI');
    writer1.FrameRate = 1;

    open(writer1);
end

nlobj.Optimization.SolverOptions.Display = 'iter';
% nlobj.Optimization.SolverOptions.Algorithm = 'active-set';
nlobj.Optimization.SolverOptions.UseParallel = true;

for i = 1:100
    x0_rect = [cur_x-state_uncertainty; cur_x+state_uncertainty];
    
    
    % plot and precompute GP
    if numel(obs_x(1, 1:end-1)) == 1
        plot_GP(2, obs_x, x_space, "x", @disturbance_x) % 1d
        plot_GP(3, obs_y, y_space, "y", @disturbance_y) % 1d
    else
        error("obs_y has too many independent variables")
    end  
    
    if ~path_to_goal_found
        tic
        % set constraint to goal
        nlobj.Optimization.CustomCostFcn = @(X, U, e, data) -1*exploit_cost_function(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval);
        nlobj.Optimization.CustomIneqConFcn = @leq_constraints_goal;
        [mv, opt, info] = nlmpcmove(nlobj, x0_rect, u0);
        exitFlag = info.ExitFlag >= 0;
        if ~exitFlag % if reaching goal infeasible, try safety
            fprintf('Reaching Goal Infeasible on Step %d, Will Prioritize Learning.\n', i);
            nlobj.Optimization.CustomCostFcn = @(X,U, e, data)  -1*explore_cost_function(X, U, e, data, obs_x, obs_y, goal_pt, blocking_interval);
            nlobj.Optimization.CustomIneqConFcn = @leq_constraints_safety;
            [mv, opt, info] = nlmpcmove(nlobj, x0_rect, u0);
        else
            path_to_goal_found = true; % set this line to true if you want immedate execution of actions to goal, else set to false
            fprintf('PATH TO GOAL FOUND ON STEP %d.\n', i)
            planned_actions = info.MVopt(1:blocking_interval:end-1, :);
            trigger_safety = zeros(control_actions, 1);
            storedXopt = info.Xopt;
            step = 1;
        end
        toc
    end
   
    %  if feasible solution found, implement that control strategy
    exitFlag = info.ExitFlag >= 0;
%     exitFlag = all(leq_constraints(info.Xopt, info.MVopt, 0, 0) <= 0);
    if exitFlag & ~path_to_goal_found
        planned_actions = info.MVopt(1:blocking_interval:end-1, :);
        trigger_safety = zeros(control_actions, 1);
        storedXopt = info.Xopt;
        step = 1;
    else
    % if not, add safety action onto end of existing strategy
        planned_actions = [planned_actions; [0, 0]];
        trigger_safety = [trigger_safety; 1];
        next_step = step + blocking_interval;
    end
    
    % isolate first planned action and remove it from the list
    action = planned_actions(1, :);
    safety = trigger_safety(1);
    planned_actions = planned_actions(2:end, :);
    trigger_safety = trigger_safety(2:end);
    
    if safety
        action = safety_controller(cur_x, u_max);
    end
    
    
    % execute planned action
    [t, x_step] = ode45(@(t, x) actual_system(t, x, action), [0, Ts], cur_x);
    
    
    cur_x = x_step(end, :)';
    
    % plot system behavior
    figure(1)
    clf(1);
    
    
    if i > 1
        plot(x_traj(:,3), x_traj(:, 4), 'k');
    end
    safe_color = 'b';
    if ~exitFlag
        safe_color = 'r';
    end
    step_color = 'g';
    if safety
        step_color = 'r';
    end
    
    
    hold on
    quiver(sampling_points_X, sampling_points_Y, sampling_points_U, sampling_points_V);
    plot(x_step(:,3), x_step(:, 4), step_color);
    scatter(x_step(1,3), x_step(1, 4), 'kx')
    plot_rect_from_state(1, safe_rect, safe_color, '--');
    plot_rect_from_state(1, goal_rect, safe_color, '-');
    plot_rect_from_state(1, unsafe_rects, 'k', '--');
    if ~isempty(storedXopt)
        for j = step:nlobj.PredictionHorizon+1;
            plot_rect_from_state(1, storedXopt(j, :), 'k', '-');
        end
        plot(storedXopt(step:end,3), storedXopt(step:end, 4), 'k:');
        plot(storedXopt(step:end,7), storedXopt(step:end, 8), 'k:');
    end
    
    if debug
        for j = 1:nlobj.PredictionHorizon+1;
                plot_rect_from_state(1, info.Xopt(j, :), 'k', '-');
        end
        plot(info.Xopt(step:end,3), info.Xopt(step:end, 4), 'k:');
        plot(info.Xopt(step:end,7), info.Xopt(step:end, 8), 'k:');
    end
    
    axis([-2, 12, -6, 16])
    xlabel('X');
    ylabel("Y");
    title("System Behavior");
    
    grid on
    hold off
    x_traj = [x_traj; x_step];
    
    
    % plot system v behavior
    figure(5)
    clf(5)
    
    if i > 1
        plot(x_traj(:,1), x_traj(:, 2), 'k');
    end
    safe_color = 'b';
    if ~exitFlag
        safe_color = 'r';
    end
    step_color = 'g';
    if safety
        step_color = 'r';
    end
    hold on
    plot(x_step(:,1), x_step(:, 2), step_color);
    scatter(x_step(1,1), x_step(1, 2), 'kx')
    plot_rect2_from_state(5, safe_rect, safe_color, '--');
    plot_rect2_from_state(5, goal_rect, safe_color, '-');
    if ~isempty(storedXopt)
        for j = step:nlobj.PredictionHorizon+1;
            plot_rect2_from_state(5, storedXopt(j, :), 'k', '-');
        end
        plot(storedXopt(step:end,1), storedXopt(step:end, 2), 'k:');
        plot(storedXopt(step:end,5), storedXopt(step:end, 6), 'k:');
    end
    
    if debug
        for j = 1:nlobj.PredictionHorizon+1;
                plot_rect2_from_state(5, info.Xopt(j, :), 'k', '-');
        end
        plot(info.Xopt(step:end,1), info.Xopt(step:end, 2), 'k:');
        plot(info.Xopt(step:end,5), info.Xopt(step:end, 6), 'k:');
    end
    
    xlabel('v');
    ylabel("psi");
    title("System Behavior");
    
    grid on
    hold off
%     
%     % plot system angle behavior
%     figure(6)
%     clf(6)
%     
%     if i > 1
%         plot(x_traj(:,5), x_traj(:, 6), 'k');
%     end
%     safe_color = 'b';
%     if ~exitFlag
%         safe_color = 'r';
%     end
%     step_color = 'g';
%     if safety
%         step_color = 'r';
%     end
%     hold on
%     plot(x_step(:,5), x_step(:, 6), step_color);
%     scatter(x_step(1,5), x_step(1, 6), 'kx')
%     plot_rect3_from_state(6, safe_rect, safe_color, '--');
%     plot_rect3_from_state(6, goal_rect, safe_color, '-');
%     if ~isempty(storedXopt)
%         for j = step:nlobj.PredictionHorizon+1;
%             plot_rect3_from_state(6, storedXopt(j, :), 'k', '-');
%         end
%         plot(storedXopt(step:end,5), storedXopt(step:end, 6), 'k:');
%         plot(storedXopt(step:end,11), storedXopt(step:end, 12), 'k:');
%     end
%     
%     if debug
%         for j = 1:nlobj.PredictionHorizon+1;
%                 plot_rect3_from_state(6, info.Xopt(j, :), 'k', '-');
%         end
%         plot(info.Xopt(step:end,5), info.Xopt(step:end, 6), 'k:');
%         plot(info.Xopt(step:end,11), info.Xopt(step:end, 12), 'k:');
%     end
%     
% %     axis([-1, 1, -2, 2])
%     xlabel('\theta');
%     ylabel("\omega");
%     title("System Behavior");
%     
%     grid on
%     hold off
    
    % plot planned actions
    figure(4)
    clf(4)
    hold on
    subplot(2, 1, 1)
    stairs(([1:numel(planned_actions(:, 1))+1] - 1) * Ts ,[action(1), planned_actions(:, 1)'],'o-');
    xlim([0, (numel(planned_actions(:, 1))+1)* Ts]);
    subplot(2, 1, 2)
    stairs(([1:numel(planned_actions(:, 2))+1] - 1) * Ts ,[action(2), planned_actions(:, 2)'],'o-');
    xlim([0, (numel(planned_actions(:, 2))+1)* Ts]);
    hold off
    
    % update obs
    obs_x = [obs_x; cur_x(3), disturbance_x(cur_x(3))]; % 1d
    obs_y = [obs_y; cur_x(4), disturbance_y(cur_x(4))]; % 1d
    
    nlobj.Model.StateFcn = @(x, u) boat_emb_mpc(x, u, obs_x, obs_y, mean_flow);
%     nlobj.Optimization.CustomCostFcn = @(X,U, e, data) -1*sigma_altitude(X, U, e, data, obs_y, obs_z, blocking_interval);
    nlobj.Optimization.CustomCostFcn = @(X,U, e, data) -1*altitude(X, U, e, data);
    u0 = action;
%     figure(2)
%     hold on
%     scatter3(cur_x(1), cur_x(2), disturbance_origstudy(cur_x, action), 'r', 'LineWidth', 5);
%     legend('Actual', 'Estimated Mean', 'Confidence Bound', 'Confidence Bound', 'Next obs_yervation');
% %     legend('Actual', 'Estimated Mean', 'Confidence Bound', 'Confidence Bound', 'obs_yervations (u may differ)', 'Next obs_yervation');
% %     axis([x_space(1), x_space(end), -20, 20])
%     hold off

    if write_video
        % write video frames
        F = getframe(1);
        writeVideo(writer1,F);
    end

    

    if ~auto_iterate
        fprintf('Iteration %d Complete, awaiting button press...', i);
        w = waitforbuttonpress;
        fprintf('registered.\n');
    else
        if write_tex
            figure(1)
            filename = ['fig/boat_' , num2str(i) , '.tex'];
            matlab2tikz(filename);
        end
        if write_svg
            filename = ['fig/boat_' , num2str(i) , '.svg'];
            saveas(1, filename);
        end
        pause(0.4);
    end
    
    [in_goal, ~] = SE_order(goal_rect, [cur_x; cur_x]');
    if in_goal
        fprintf('Goal Reached, Terminating.\n');
        break;
    end
    step = next_step;
end

if write_video
    close(writer1);
end

%% Constraints for MPC
% equality constraints
function eq = eq_constraints(X, U, data)
    eq = [];
end

% center of final hyperrectangle is 0 (unused)
function eq = end_zero(X, U, data)
    eq = [X(end, 1) + X(end, 3); X(end, 2) + X(end, 4)];
end

% inequality constraints
function leq = leq_constraints_safety(X, U, e, data)
    leq = [];
    leq = [leq; preserve_order(X, U, e, data)];
    leq = [leq; end_in_safety_or_goal(X, U, e, data)];
    leq = [leq; never_enter_unsafe(X, U, e, data)];
end

function leq = leq_constraints_goal(X, U, e, data)
    leq = [];
    leq = [leq; preserve_order(X, U, e, data)];
    leq = [leq; end_in_goal(X, U, e, data)];
    leq = [leq; never_enter_unsafe(X, U, e, data)];
end

% preserve order of mixed monotonicity
function leq = preserve_order(X, U, e, data)
    leq = [X(:, 1) - X(:, 5); 
        X(:, 2) - X(:, 6); 
        X(:, 3) - X(:, 7); 
        X(:, 4) - X(:, 8); ];
end

% ensure final hyperrectangle is within safety or goal region (defined earlier, assumed union of hyperrectangles)
function leq = end_in_safety_or_goal(X, U, e, data)
   global safe_rect goal_rect;
   final_rect = X(end, :);
   [~, val_safe] = SE_order(safe_rect, final_rect);  % returns positive values if final rect is within safe rect
   [~, val_goal] = SE_order(goal_rect, final_rect);  % returns positive values if final rect is within goal rect
   leq_safe = -val_safe';
   leq_goal = -val_goal';
   
   % if goal is met, return that instead
   if all(leq_goal < 0)
       leq = leq_goal;
   else
       leq = leq_safe;
   end
end

function leq = end_in_goal(X, U, e, data)
   global goal_rect;
   final_rect = X(end, :);
   [~, val_goal] = SE_order(goal_rect, final_rect);  % returns positive values if final rect is within goal rect
   leq = -val_goal';
   
end


function leq = never_enter_unsafe(X, U, e, data)
global unsafe_rects
num_unsafe = size(unsafe_rects);
num_unsafe = num_unsafe(1);


num_rects = size(X);
num_rects = num_rects(1);
leq = [];


for u = 1:num_unsafe
    % Version 1: directly check rectangles
    for i = 1:num_rects
        flipped_unsafe = [unsafe_rects(u, 7:8), unsafe_rects(u, 3:4)];
        rect = X(i, :);
        [~, val] = SE_order([rect(3:4), rect(7:8)], flipped_unsafe);  % returns positive values if unsafe rect is to the SE of the rectangle
        leq = [leq; min(val)];
        % also check intermediate points
%         intermed_step = 1;% set this step to whatever desired, if >= 1 will skip
%         if i < num_rects && intermed_step < 1
%             for j = 0:intermed_step:1 
%                 next_rect = X(i + 1, :);
%                 half_rect = (j*rect + (1-j)*next_rect);
%                 [~, val] = SE_order([half_rect(1:6), half_rect(7:end)], flipped_unsafe);  
%                 leq = [leq; min(val)];
%             end
%         end
    end

end

% toc
end

%% Helper Functions
function [se, val] = SE_order(xin, yin)
    if numel(xin) ~= numel(yin)
        error('attempted to compare two vectors of unequal length');
    elseif mod(numel(xin), 2) ~= 0
        error('vectors have an odd number of elements; have you made sure to concatenate x and hat?');
    end
    
    x = xin(1:end/2);
    xhat = xin((end/2)+1:end);
    
    y = yin(1:end/2);
    yhat = yin((end/2)+1:end);
    
    val = [y - x, xhat - yhat]; % returns all positive values if southeast order preserved, i.e. y is contained in x
    se = all(x <= y) && all(xhat >= yhat);
end

% plots hyperrectangle (assumes x = [x1_min, x2_min, x1_max, x2_max])
function plot_rect_from_state(fig, x, color, style)
    num_rects = size(x);
    num_rects = num_rects(1);
    
    figure(fig)
    hold on
    for i = 1:num_rects
        rect = x(i, :);
        corner = [min(rect(3), rect(7)), min(rect(4), rect(8))];
        sizes = [abs(rect(7) - rect(3)), abs(rect(8) - rect(4))];
        rectangle('Position',[corner, sizes], 'LineStyle', style, 'EdgeColor', color);
    end
end

function plot_rect2_from_state(fig, x, color, style)
    num_rects = size(x);
    num_rects = num_rects(1);
    
    figure(fig)
    hold on
    for i = 1:num_rects
        rect = x(i, :);
        corner = [min(rect(1), rect(5)), min(rect(2), rect(6))];
        sizes = [abs(rect(1) - rect(5)), abs(rect(2) - rect(6))];
        rectangle('Position',[corner, sizes], 'LineStyle', style, 'EdgeColor', color);
    end
end
% 
% function plot_rect3_from_state(fig, x, color, style)
%     figure(fig)
%     hold on
%     corner = [min(x(5), x(11)), min(x(6), x(12))];
%     sizes = [abs(x(11) - x(5)), abs(x(12) - x(6))];
%     rectangle('Position',[corner, sizes], 'LineStyle', style, 'EdgeColor', color);
% end

function res = plot_GP(fig, obs_y, x_space, state, dist_fn)
    sigma_confidence = 3;
    [f_bar_star, cov_f_star] = fit_params(obs_y(:, 1), obs_y(:, 2), x_space');
    figure(fig)
    plot(x_space, dist_fn(x_space'), 'k-'); % actual
    hold on
    scatter(obs_y(:, 1), obs_y(:, 2), 'ko'); % obs_yervations
    plot(x_space, f_bar_star, 'b--'); % mean
    std_f_star = sqrt(diag(cov_f_star));
    lower_x = (f_bar_star + sigma_confidence * std_f_star)';
    upper_x = (f_bar_star - sigma_confidence * std_f_star)';
    fill_Ax = [x_space, fliplr(x_space)];
    fill_Bx = [lower_x, fliplr(upper_x)];
    fill(fill_Ax, fill_Bx, 'k', 'facealpha', 0.2, 'edgealpha', 0);
    title(["River Disturbance in " + state + " direction"])
    xlabel(state)
    ylabel("magnitude")
    legend('Actual', 'observations', 'Estimated Mean', 'Confidence Bound');
    hold off
end

function res = plot_GP3D(fig, contour_fig, obs_y, x1_space, x2_space, dist_fn)
    sigma_confidence = 3;
    [x1_vec, x2_vec] = meshgrid(x1_space, x2_space);
    x_space = [reshape(x1_vec, [], 1), reshape(x2_vec, [], 1)];
    num_points = numel(x1_space)*numel(x2_space);
    [f_bar_star, cov_f_star] = fit_params(obs_y(:, 1:end-1), obs_y(:, end), x_space);
    figure(fig)
    actual = zeros(num_points, 1);
    for i = 1:num_points
        actual(i) = dist_fn([x_space(i, 1); x_space(i, 2)]);
    end
    x1_surf = reshape(x_space(:, 1), numel(x1_space), numel(x2_space));
    x2_surf = reshape(x_space(:, 2), numel(x1_space), numel(x2_space));
    actual = reshape(actual, numel(x1_space), numel(x2_space));
    surf(x1_surf,x2_surf,1 - actual,'FaceColor','g'); % actual
    hold on
    std_f_star = sqrt(diag(cov_f_star));
    est_mean = reshape(f_bar_star, numel(x1_space), numel(x2_space));
    surf(x1_surf,x2_surf, 1 - est_mean,'FaceColor','b') % mean
    lower_x = 1 - (f_bar_star + sigma_confidence * std_f_star)';
    upper_x = 1 - (f_bar_star - sigma_confidence * std_f_star)';
    
    est_upper = reshape(upper_x, numel(x1_space), numel(x2_space));
    est_lower = reshape(lower_x, numel(x1_space), numel(x2_space));
    surf(x1_surf,x2_surf,real(est_upper), 'FaceColor','k', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.2) % upper
    surf(x1_surf,x2_surf,real(est_lower), 'FaceColor','k', 'FaceAlpha', 0.6, 'EdgeAlpha', 0.6) % lower

    scatter3(obs_y(:, 1), obs_y(:, 2), 1 - obs_y(:, end),  'c', 'LineWidth', 5); % obs_yervations
    title(["Friction Ratio"])
    xlabel("x_1")
    ylabel("x_2")
    zlabel("Ratio")
    legend('Actual', 'Estimated Mean', 'Confidence Bound', 'Confidence Bound', 'obs_yervations');
    hold off
    
    figure(contour_fig)
    hold on
    contour(x1_surf,x2_surf,1 - actual, [1 1]);
    
    lower_grid = [x_space, lower_x'];
    upper_grid = [x_space, upper_x'];
    
    res = zeros([size(lower_grid), 2]);
    res(:, :, 1) = lower_grid;
    res(:, :, 2) = upper_grid;
end