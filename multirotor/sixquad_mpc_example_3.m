clear;

global  goal_rect unsafe_rects Ts blocking_interval prob_to_happen
unsafe_rects = [-6, -100, -1, -100, -2*pi, -2*pi, -5, 100, 10, 100, 2*pi, 2*pi;
    5, -100, -1, -100, -2*pi, -2*pi, 6, 100, 10, 100, 2*pi, 2*pi;
    -1, -100, 4, -100, -2*pi, -2*pi, 1, 100, 7, 100, 2*pi, 2*pi;];

goal_pt = [0, 0, 5, 0, 0, 0];
goal_bound = [1, 8, 5, 8, pi, pi];
goal_rect = [goal_pt - goal_bound, goal_pt + goal_bound];
prob_to_happen = 0.4;

auto_iterate = true;
plot_full_traj = false;
write_video = false; %WARNING: does not iterate yet!
save_images = false;
debug = false;
write_tex = false;
num_runs = 100;

Ts = 0.2;

u_max = [100, 6*pi];

results_file = fopen("results.txt", 'w');
fprintf(results_file, 'Run\tSeed\tOpt1\tOpt05\tOpt01\n');

for r = 1:num_runs 
    
    seed = 500 + r;
    rng(seed);
    y_dist_obs = [[-2:2:12]',rand(8, 1)*8 - 4];
    z_dist_obs = [[-2:2:12]',rand(8, 1)*4 - 2];

    % dist_y_func = @disturbance_y;
    % dist_z_func = @disturbance_z;
    dist_y_func = @(z)disturbance_y_obs(z, y_dist_obs);
    dist_z_func = @(z)disturbance_z_obs(z, z_dist_obs);
    actual_system = @(t, x, u) sixquad_windy(t, x, u, dist_y_func, dist_z_func);

    if ~all(goal_rect(7:end) >= goal_rect(1:6))
        error('goal rect defined incorrectly!')
    else
        num_unsafe = size(unsafe_rects);
        num_unsafe = num_unsafe(1);
        for u=1:num_unsafe
            if ~all(unsafe_rects(u, 7:end) >= unsafe_rects(u, 1:6))
                error('an unsafe rect defined incorrectly!')
            end
        end
    end
    [sampling_points_X, sampling_points_Y] = meshgrid(-6:6, -2:12);
    sampling_points_X = sampling_points_X(:);
    sampling_points_Y = sampling_points_Y(:);
    sampling_points_U = dist_y_func(sampling_points_Y);
    sampling_points_V = dist_z_func(sampling_points_Y);


    z_vec = [0, 1, 2, 3];
    obs_y = [];
    obs_z = [];
    for i = 1:numel(z_vec)
        obs_y = [obs_y; z_vec(i), dist_y_func(z_vec(i))];
        obs_z = [obs_z; z_vec(i), dist_z_func(z_vec(i))];
    end
    z_space = [-2:0.25:12];

    %% MPC
    nx = 12; % number of state variables
    ny = 1; % number of output variables
    nu = 4; % number of input variables
    nlobj = nlmpc(nx,ny,nu);

    % horizons
    blocking_interval = 2; % number of timesteps to execute each control action
    control_actions = 7; % number of control actions
    nlobj.Ts = Ts/blocking_interval; % these should set prediction horizon to match Ts of control actions
    nlobj.PredictionHorizon = control_actions*blocking_interval;
    nlobj.ControlHorizon = [ones(1,control_actions) * blocking_interval];

    % model system dynamics
    nlobj.Model.StateFcn = @(x, u) sixquad_emb_mpc_vardist(x, u, obs_y, obs_z);
    % nlobj.Model.StateFcn = @(x, u) sixquad_emb_mpc(x, u, obs_y, obs_z);
    % If true, MATLAB automatically discretizes the model for optimization
    % using Ts, else Ts is the sample time
    nlobj.Model.IsContinuousTime = true; 
    nlobj.Model.OutputFcn = @(x,u) x(1);

    % cost function
    nlobj.Optimization.CustomCostFcn = @(X,U, e, data) applied_input(X, U, e, data, blocking_interval);

    % control input limits (MV = manipulated variable)
    nlobj.MV(1).Min = -u_max(1);
    nlobj.MV(1).Max = u_max(1);
    nlobj.MV(2).Min = -u_max(2);
    nlobj.MV(2).Max = u_max(2);
    nlobj.MV(3).Min = 0;
    nlobj.MV(3).Max = 3;
    nlobj.MV(4).Min = 0;
    nlobj.MV(4).Max = 3;


    % other constraints
    nlobj.Optimization.CustomEqConFcn = @eq_constraints;
    nlobj.Optimization.CustomIneqConFcn = @leq_constraints_goal;
    nlobj.Optimization.UseSuboptimalSolution = true; % if max number of iterations reached w/out solving, still give suboptimal

    % define initial conditions, verify correct construction
    x0 = [-4.5; 0; 3; 0; 0; 0];
    u0 = [0; 0; 3; 0];
    u02 = u0;
    u0B = u0;
    state_uncertainty = 0;%[0.01; 0.01; 0.01; 0.01; 0.01; 0.01];
    x0_rect = [x0-state_uncertainty; x0+state_uncertainty];
    validateFcns(nlobj, x0_rect, u0);
    cur_x = x0;

    x_traj = [];
    storedXopt = [];
    storedMVopt = [];

    x_traj2 = [];
    storedXopt2 = [];
    storedMVopt2 = [];
    
    x_trajB = [];
    storedXoptB = [];
    storedMVoptB = [];

    step = 1;
    next_step = step;
    in_goal = [false, false, false]; % set to true if you don't want a strat to run
    total_cost = [0, 0, 0];
    
    

    if write_video
        % Video creation objects
        writer1 = VideoWriter('state_6quadfull_5', 'UNCOMPRESSED AVI');
        writer1.FrameRate = 1;
        writer2 = VideoWriter('dist_y_opt_5', 'UNCOMPRESSED AVI');
        writer2.FrameRate = 1;
        writer3 = VideoWriter('dist_z_opt_5', 'UNCOMPRESSED AVI');
        writer3.FrameRate = 1;
        writer4 = VideoWriter('dist_y_pes_5', 'UNCOMPRESSED AVI');
        writer4.FrameRate = 1;
        writer5 = VideoWriter('dist_z_pes_5', 'UNCOMPRESSED AVI');
        writer5.FrameRate = 1;

        open(writer1);
        open(writer2);
        open(writer3);
        open(writer4);
        open(writer5);
    end

    nlobj.Optimization.SolverOptions.Display = 'iter';
    nlobj.Optimization.SolverOptions.Algorithm = 'sqp'; 
    % interior-point (default), trust-region-reflective, sqp, sqp-legacy, active-set
    nlobj.Optimization.SolverOptions.UseParallel = true;
    nlobj.Optimization.SolverOptions.MaxIterations = 600; % default 400

    % we'll say optimistic strat is x, baseline is x2
    obs_z2 = obs_z;
    obs_y2 = obs_y;
    obs_z3 = obs_z;
    obs_y3 = obs_y;
    obs_zB = obs_z;
    obs_yB = obs_y;
    cur_x2 = cur_x;
    cur_xB = cur_x;
  

    for i = 1:100
        tic
        % set constraint to goal
        nlobj.Optimization.CustomCostFcn = @(X,U, e, data) applied_input(X, U, e, data, blocking_interval);
        nlobjBest.Optimization.CustomCostFcn = @(X,U, e, data) applied_input(X, U, e, data, blocking_interval);
        if i > 1
            nlobj.Optimization.CustomIneqConFcn = @leq_constraints_hitgoal;
        else
            nlobj.Optimization.CustomIneqConFcn = @leq_constraints_goal;
        end
        nlobj.Optimization.CustomEqConFcn = @eq_constraints;
        nlobj.MV(3).Max = 3;
        nlobj.MV(4).Max = 3;
            
        if ~in_goal(2)
            prob_to_happen = 0.05;
            x0_rect2 = [cur_x2-state_uncertainty; cur_x2+state_uncertainty];
            nlobj.MV(3).Min = 0;
            nlobj.MV(4).Min = 0;
            nlobj.Model.StateFcn = @(x, u) sixquad_emb_mpc_vardist(x, u, obs_y2, obs_z2);
            [mv2, opt2, info2] = nlmpcmove(nlobj, x0_rect2, u02);
            exitFlag = info2.ExitFlag >= 0;
            % if reaching goal infeasible, check if any actions are stored and perform the next one
            if ~exitFlag
                fprintf('Reaching Goal Infeasible (Opt).\n');
                if isempty(storedMVopt2)
                    fprintf('No moves available, terminating.\n');
                    break;
                end
            else
                storedMVopt2 = info2.MVopt;
                storedXopt2 = info2.Xopt;
            end

            action2 = storedMVopt2(1, :);
            u02 = action2;
            total_cost(2) = total_cost(2) + sum(abs(forces(action2(1:2))));
            [t, x_step2] = ode45(@(t, x) actual_system(t, x, action2), [0, Ts], cur_x2);
            cur_x2 = x_step2(end, :)';
        else
            x_step2 = cur_x2';
        end

        if ~in_goal(1)
            prob_to_happen = 0.1;
            x0_rect = [cur_x-state_uncertainty; cur_x+state_uncertainty];
            nlobj.MV(3).Min = 0;
            nlobj.MV(4).Min = 0;
            nlobj.Model.StateFcn = @(x, u) sixquad_emb_mpc_vardist(x, u, obs_y, obs_z);
            [mv, opt, info] = nlmpcmove(nlobj, x0_rect, u0);
            exitFlag = info.ExitFlag >= 0;
            % if reaching goal infeasible, check if any actions are stored and perform the next one
            if ~exitFlag
                fprintf('Reaching Goal Infeasible (Opt).\n');
                if isempty(storedMVopt)
                    fprintf('No moves available, terminating.\n');
                    break;
                end
            else
                storedMVopt = info.MVopt;
                storedXopt = info.Xopt;
            end

            action = storedMVopt(1, :);
            u0 = action;
            total_cost(1) = total_cost(1) + sum(abs(forces(action(1:2))));
            [t, x_step] = ode45(@(t, x) actual_system(t, x, action), [0, Ts], cur_x);
            cur_x = x_step(end, :)';
        else
            x_step = cur_x';
        end

        
       if ~in_goal(3)
           prob_to_happen = 0.01;
            x0_rectB = [cur_xB-state_uncertainty; cur_xB+state_uncertainty];
            nlobj.MV(3).Min = 0;
            nlobj.MV(4).Min = 0;
            nlobj.Model.StateFcn = @(x, u) sixquad_emb_mpc_vardist(x, u, obs_yB, obs_zB);
            [mvB, optB, infoB] = nlmpcmove(nlobj, x0_rectB, u0B);
            exitFlag = infoB.ExitFlag >= 0;
            % if reaching goal infeasible, check if any actions are stored and perform the next one
            if ~exitFlag
                fprintf('Reaching Goal Infeasible (Opt).\n');
                if isempty(storedMVoptB)
                    fprintf('No moves available, terminating.\n');
                    break;
                end
            else
                storedMVoptB = infoB.MVopt;
                storedXoptB = infoB.Xopt;
            end

            actionB = storedMVoptB(1, :);
            u0B = actionB;
            total_cost(3) = total_cost(3) + sum(abs(forces(actionB(1:2))));
            [t, x_stepB] = ode45(@(t, x) actual_system(t, x, actionB), [0, Ts], cur_xB);
            cur_xB = x_stepB(end, :)';
        else
            x_stepB = cur_xB';
        end

        toc    

        % plot system behavior
        figure(1)
        clf(1);
        quiver(sampling_points_X, sampling_points_Y, sampling_points_U, sampling_points_V);

        hold on
        if ~isempty(x_traj) 
            plot(x_traj(:,1), x_traj(:, 3), 'b');
        end
        if ~isempty(x_traj2)
            plot(x_traj2(:,1), x_traj2(:, 3), 'r');
        end
        if ~isempty(x_trajB)
            plot(x_trajB(:,1), x_trajB(:, 3), 'k');
        end

        plot(x_step(:,1), x_step(:, 3), 'b--');
        scatter(x_step(1,1), x_step(1, 3), 'bx')
        plot(x_step2(:,1), x_step2(:, 3), 'r--');
        scatter(x_step2(1,1), x_step2(1, 3), 'rx')
        plot(x_stepB(:,1), x_stepB(:, 3), 'k--');
        scatter(x_stepB(1,1), x_stepB(1, 3), 'kx')
        plot_rect_from_state(1, goal_rect, 'g', '-');
    %     plot_rect_from_state(1, unsafe_rects, 'k', '--');
        if ~isempty(storedXopt)
            plot_rect_from_state(1, storedXopt, 'b', '-');
            plot(storedXopt(:,1), storedXopt(:, 3), 'b:');
            plot(storedXopt(:,7), storedXopt(:, 9), 'b:');
        end

        if ~isempty(storedXopt2)
            plot_rect_from_state(1, storedXopt2, 'r', '-');
            plot(storedXopt2(:,1), storedXopt2(:, 3), 'r:');
            plot(storedXopt2(:,7), storedXopt2(:, 9), 'r:');
        end
        
        if ~isempty(storedXoptB)
            plot_rect_from_state(1, storedXoptB, 'k', '-');
            plot(storedXoptB(:,1), storedXoptB(:, 3), 'k:');
            plot(storedXoptB(:,7), storedXoptB(:, 9), 'k:');
        end

    %     if debug
    %         for j = 1:nlobj.PredictionHorizon+1;
    %                 plot_rect_from_state(1, info2.Xopt(j, :), 'k', '-');
    %         end
    %         plot(info2.Xopt(step:end,1), info2.Xopt(step:end, 3), 'k:');
    %         plot(info2.Xopt(step:end,7), info2.Xopt(step:end, 9), 'k:');
    %     end

        axis([-6, 6, -2, 11])
        xlabel('Y');
        ylabel("Z");
        title("System Behavior");

        grid on
        hold off

        x_traj = [x_traj; x_step];
        x_traj2 = [x_traj2; x_step2];
        x_trajB = [x_trajB; x_stepB];

    %     % plot system v behavior
    %     figure(5)
    %     clf(5)
    %     
    %     if i > 1
    %         plot(x_traj(:,2), x_traj(:, 4), 'k');
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
    %     plot(x_step(:,2), x_step(:, 4), step_color);
    %     scatter(x_step(1,2), x_step(1, 4), 'kx')
    %     plot_rect2_from_state(5, goal_rect, safe_color, '-');
    %     if ~isempty(storedXopt)
    %         for j = step:nlobj.PredictionHorizon+1;
    %             plot_rect2_from_state(5, storedXopt(j, :), 'k', '-');
    %         end
    %         plot(storedXopt(step:end,2), storedXopt(step:end, 4), 'k:');
    %         plot(storedXopt(step:end,8), storedXopt(step:end, 10), 'k:');
    %     end
    %     
    %     if debug
    %         for j = 1:nlobj.PredictionHorizon+1;
    %                 plot_rect2_from_state(5, info.Xopt(j, :), 'k', '-');
    %         end
    %         plot(info.Xopt(step:end,2), info.Xopt(step:end, 4), 'k:');
    %         plot(info.Xopt(step:end,8), info.Xopt(step:end, 10), 'k:');
    %     end
    %     
    %     xlabel('vy');
    %     ylabel("vz");
    %     title("System Behavior");
    %     
    %     grid on
    %     hold off
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


        % plot gp 
        lower_sig = action(3);
        upper_sig = action(4);
        plot_GP(2, obs_y, z_space, "y", dist_y_func, lower_sig, upper_sig) % 1d
        plot_GP(3, obs_z, z_space, "z", dist_z_func, lower_sig, upper_sig) % 1d

        % update obs
        obs_y = [obs_y; cur_x(3), dist_y_func(cur_x(3))]; % 1d
        obs_z = [obs_z; cur_x(3), dist_z_func(cur_x(3))]; % 1d

        % plot gp 
        lower_sig = action2(3);
        upper_sig = action2(4);
        plot_GP(4, obs_y2, z_space, "y", dist_y_func, lower_sig, upper_sig) % 1d
        plot_GP(5, obs_z2, z_space, "z", dist_z_func, lower_sig, upper_sig) % 1d

        % update obs
        obs_y2 = [obs_y2; cur_x2(3), dist_y_func(cur_x2(3))]; % 1d
        obs_z2 = [obs_z2; cur_x2(3), dist_z_func(cur_x2(3))]; % 1d
        
        
        % plot gp 
        lower_sig = actionB(3);
        upper_sig = actionB(4);
        plot_GP(6, obs_yB, z_space, "y", dist_y_func, lower_sig, upper_sig) % 1d
        plot_GP(7, obs_zB, z_space, "z", dist_z_func, lower_sig, upper_sig) % 1d
        
        obs_yB = [obs_yB; cur_xB(3), dist_y_func(cur_xB(3))]; % 1d
        obs_zB = [obs_zB; cur_xB(3), dist_z_func(cur_xB(3))]; % 1d


        if write_video
            % write video frames
            F = getframe(1);
            writeVideo(writer1,F);
            F = getframe(2);
            writeVideo(writer2,F);
            F = getframe(3);
            writeVideo(writer3,F);
            F = getframe(4);
            writeVideo(writer4,F);
            F = getframe(5);
            writeVideo(writer5,F);
        end

        if all(in_goal)
            break;
        end

        if ~auto_iterate
            fprintf('Iteration %d Complete, awaiting button press...', i);
            w = waitforbuttonpress;
            fprintf('registered.\n');
        else
            if write_tex
                figure(1)
                filename = ['fig/sixquad_' , num2str(i) , '.tex'];
                matlab2tikz(filename);
            end
            pause(0.4);
        end

        [inside, ~] = SE_order(goal_rect, [cur_x; cur_x]');
        if inside
            fprintf('(Opt2) Goal Reached.\n');
            in_goal(1) = true;
        else
            storedXopt = storedXopt(blocking_interval+1:end, :);
            storedMVopt = storedMVopt(blocking_interval+1:end, :);
        end

        [inside, ~] = SE_order(goal_rect, [cur_x2; cur_x2]');
        if inside
            fprintf('(Opt3) Goal Reached.\n');
            in_goal(2) = true;
        else
            storedXopt2 = storedXopt2(blocking_interval+1:end, :);
            storedMVopt2 = storedMVopt2(blocking_interval+1:end, :);
        end
        
        [inside, in_vals] = SE_order(goal_rect, [cur_xB; cur_xB]');
        if inside % all(in_vals + 1e-4 >= 0)
            fprintf('(Opt5) Goal Reached.\n');
            in_goal(3) = true;
        else
            storedXoptB = storedXoptB(blocking_interval+1:end, :);
            storedMVoptB = storedMVoptB(blocking_interval+1:end, :);
        end

    end

    if write_video
        close(writer1);
        close(writer2);
        close(writer3);
        close(writer4);
        close(writer5);
    end
    
    if save_images
        saveas(1, ["traj/trajectories_" + num2str(r) + ".png"]);
    end
    
    fprintf(results_file, [num2str(r) + "\t" + num2str(seed) + "\t" + num2str(total_cost(1), '%.2f') + "\t" + num2str(total_cost(2), '%.2f') + "\t" + num2str(total_cost(3), '%.2f') + "\n"]);
end

%% Constraints for MPC
% equality constraints
function eq = eq_constraints(X, U, data)
    eq = [];
    eq = [eq; sig_conf(X, U, data)];
end

% sig confidences are all equal
function eq = sig_conf(X, U, data)
    eq = U(:, 3) - U(1, 3);
    eq = [eq; U(:, 4) - U(1, 4)];
end

% inequality constraints
function leq = leq_constraints_goal(X, U, e, data)
    leq = [];
    leq = [leq; preserve_order(X, U, e, data)];
    leq = [leq; end_in_goal(X, U, e, data, 0)];
%     leq = [leq; never_enter_unsafe(X, U, e, data)];
    leq = [leq; likely_to_happen(X, U, e, data)];
end

function leq = leq_constraints_hitgoal(X, U, e, data)
    leq = [];
    leq = [leq; preserve_order(X, U, e, data)];
    leq = [leq; hit_goal(X, U, e, data, 0)];
%     leq = [leq; never_enter_unsafe(X, U, e, data)];
    leq = [leq; likely_to_happen(X, U, e, data)];
end

function leq = leq_constraints_goalB(X, U, e, data)
    leq = [];
    leq = [leq; preserve_order(X, U, e, data)];
    leq = [leq; end_in_goal(X, U, e, data, 0)];
end

function leq = leq_constraints_hitgoalB(X, U, e, data)
    leq = [];
    leq = [leq; preserve_order(X, U, e, data)];
    leq = [leq; hit_goal(X, U, e, data, 0)];
end

% preserve order of mixed monotonicity
function leq = preserve_order(X, U, e, data)
    leq = [X(:, 1) - X(:, 7); 
        X(:, 2) - X(:, 8); 
        X(:, 3) - X(:, 9); 
        X(:, 4) - X(:, 10); 
        X(:, 5) - X(:, 11); 
        X(:, 6) - X(:, 12)];
end

% ensure final hyperrectangle is within goal region (defined earlier, assumed union of hyperrectangles)
function leq = end_in_goal(X, U, e, data, tol)
   global goal_rect;
   final_rect = X(end, :);
   [~, val_goal] = SE_order(goal_rect, final_rect);  % returns positive values if final rect is within goal rect
   leq = -val_goal' + tol;
   
end

function leq = hit_goal(X, U, e, data, tol)
   global goal_rect blocking_interval;
   X = X(1:blocking_interval:end, :);
    num_rects = size(X);
    num_rects = num_rects(1);
    leq = [];
    for i = 1:num_rects
       [~, val_goal] = SE_order(goal_rect, X(i, :));  % returns positive values if final rect is within goal rect
       leq = [leq; max(-val_goal')];
    end
    leq = min(leq) + tol;
   
end

function leq = likely_to_happen(X, U, e, data)
% tic
    global prob_to_happen
    sig_lower = U(:, end-1);
    sig_upper = U(:, end);
    prob = fastcdf(sig_upper) - fastcdf(-sig_lower);
    leq = prob_to_happen - prob; % want probability at least 0.x
% toc
    leq = [leq; -(sig_upper + sig_lower)]; % also want to ensure they don't "cross" over each other
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
        flipped_unsafe = [unsafe_rects(u, [7, 9]), unsafe_rects(u, [1, 3])];
        rect = X(i, :);
        [~, val] = SE_order([rect([1, 3]), rect([7, 9])], flipped_unsafe);  % returns positive values if unsafe rect is to the SE of the rectangle
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

function F = forces(inputs)
    L = 1/2; % MAKE SURE THIS L MATCHES THE ONE USED IN COST FUNCTION
    A = [1, 1; L/2, -L/2];
    b = [inputs(1); inputs(2)];
    F = A \ b;
end

% plots hyperrectangle (assumes x = [x1_min, x2_min, x1_max, x2_max])
function plot_rect_from_state(fig, x, color, style)
    num_rects = size(x);
    num_rects = num_rects(1);
    
    figure(fig)
    hold on
    for i = 1:num_rects
        rect = x(i, :);
        corner = [min(rect(1), rect(7)), min(rect(3), rect(9))];
        sizes = [abs(rect(7) - rect(1)), abs(rect(9) - rect(3))];
        rectangle('Position',[corner, sizes], 'LineStyle', style, 'EdgeColor', color);
    end
end

function plot_rect2_from_state(fig, x, color, style)
    figure(fig)
    hold on
    corner = [min(x(2), x(8)), min(x(4), x(10))];
    sizes = [abs(x(8) - x(2)), abs(x(10) - x(4))];
    rectangle('Position',[corner, sizes], 'LineStyle', style, 'EdgeColor', color);
end

function plot_rect3_from_state(fig, x, color, style)
    figure(fig)
    hold on
    corner = [min(x(5), x(11)), min(x(6), x(12))];
    sizes = [abs(x(11) - x(5)), abs(x(12) - x(6))];
    rectangle('Position',[corner, sizes], 'LineStyle', style, 'EdgeColor', color);
end

function res = plot_GP(fig, obs_y, x_space, state, dist_fn, lower_sig, upper_sig)
    sigma_confidence = 3;
    [f_bar_star, cov_f_star] = fit_params(obs_y(:, 1), obs_y(:, 2), x_space');
    figure(fig)
    plot(x_space, dist_fn(x_space'), 'k-'); % actual
    hold on
    scatter(obs_y(:, 1), obs_y(:, 2), 'ko'); % obs_yervations
    plot(x_space, f_bar_star, 'b--'); % mean
    std_f_star = sqrt(diag(cov_f_star));
    upper_x = (f_bar_star + sigma_confidence * std_f_star)';
    lower_x = (f_bar_star - sigma_confidence * std_f_star)';
    fill_Ax = [x_space, fliplr(x_space)];
    fill_Bx = [upper_x, fliplr(lower_x)];
    fill(fill_Ax, fill_Bx, 'k', 'facealpha', 0.2, 'edgealpha', 0);
    
    upper_x = (f_bar_star + upper_sig * std_f_star)';
    lower_x = (f_bar_star - lower_sig * std_f_star)';
    fill_Ax = [x_space, fliplr(x_space)];
    fill_Bx = [upper_x, fliplr(lower_x)];
    fill(fill_Ax, fill_Bx, 'b', 'facealpha', 0.2, 'edgealpha', 0);
    title(["Wind Disturbance in " + state + " direction"])
    xlabel("Altitude z")
    ylabel("magnitude")
    legend('Actual', 'observations', 'Estimated Mean', 'Confidence Bound', 'Optimism Bound');
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