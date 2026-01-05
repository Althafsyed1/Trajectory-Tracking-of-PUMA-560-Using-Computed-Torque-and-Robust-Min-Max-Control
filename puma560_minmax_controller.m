function puma560_robust_minmax_sim()
    % PUMA 560 Robust Min-Max Control: Compare Nominal vs 2kg Payload
    clear; clc; close all;

    %% 1. Load Robot Model
    mdl_puma560;
    nominal_robot = p560.nofriction();
    actual_robot = p560.nofriction();

    % Add 2 kg payload to link 6 of actual robot
    payload_mass = 2.0;
    actual_robot.links(6).m = actual_robot.links(6).m + payload_mass;

    %% 2. Simulation Parameters
    dt = 0.02;
    T = 5;
    t = 0:dt:T;
    N = length(t);

    Kp = diag([300, 300, 150, 50, 50, 30]);
    Kd = diag([60, 60, 30, 10, 10, 5]);

    %% 3. Define Trajectory
    q0 = [0, 0, 0, 0, 0, 0];
    qf = [pi/4, pi/3, pi/2, 0, pi/4, 0];
    [q_des, qd_des, qdd_des] = quintic_traj(q0, qf, t);

    %% === Run Simulation WITHOUT Payload (Nominal Robot) ===
    [q_nom, err_nom] = run_simulation(nominal_robot, nominal_robot, q_des, qd_des, qdd_des, Kp, Kd, t, dt, 'Nominal_Robot');

    %% === Run Simulation WITH 2kg Payload (Unknown to Controller) ===
    [q_payload, err_payload] = run_simulation(nominal_robot, actual_robot, q_des, qd_des, qdd_des, Kp, Kd, t, dt, 'Robust_MinMax_Controller');

    %% 4. Plot Joint Tracking Comparison
    figure('Name', 'Joint Tracking Comparison');
    for j = 1:6
        subplot(3,2,j);
        plot(t, q_des(:,j), 'k--', t, q_nom(:,j), 'b-', t, q_payload(:,j), 'r-');
        xlabel('Time [s]');
        ylabel(['q_', num2str(j), ' [rad]']);
        legend('Desired','Nominal','Payload');
        title(['Joint ', num2str(j)]);
        grid on;
    end

    %% 5. Plot Tracking Error Comparison
    figure('Name', 'Tracking Error Comparison');
    for j = 1:6
        subplot(3,2,j);
        plot(t, err_nom(:,j), 'b-', t, err_payload(:,j), 'r-');
        xlabel('Time [s]');
        ylabel(['Error q_', num2str(j), ' [rad]']);
        legend('Nominal','Payload');
        title(['Joint ', num2str(j), ' Error']);
        grid on;
    end

    %% 6. Numerical Error Comparison
    joint_names = {'q1','q2','q3','q4','q5','q6'};
    rmse_nom = sqrt(mean(err_nom.^2));
    rmse_payload = sqrt(mean(err_payload.^2));
    maxerr_nom = max(abs(err_nom));
    maxerr_payload = max(abs(err_payload));

    fprintf('\n%-6s | %-10s | %-10s | %-10s | %-10s\n', 'Joint', 'RMSE_nom', 'RMSE_payload', 'Max_nom', 'Max_payload');
    fprintf('---------------------------------------------------------------\n');
    for j = 1:6
        fprintf('%-6s | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            joint_names{j}, rmse_nom(j), rmse_payload(j), maxerr_nom(j), maxerr_payload(j));
    end
end

%% === Robust Min-Max Simulation Runner ===
function [q_log, error_log] = run_simulation(nominal_robot, actual_robot, q_des, qd_des, qdd_des, Kp, Kd, t, dt, label)
    N = length(t);
    q = q_des(1,:);
    qd = zeros(1,6);
    q_log = zeros(N,6);
    error_log = zeros(N,6);

    % Robust control parameter
    robust_gain = diag([10, 10, 10, 5, 5, 2]);

    % Setup video writer - Changed to MP4
    v = VideoWriter([label '_sim.mp4'], 'MPEG-4');
    v.Quality = 100; % Optional: Set quality (0-100)
    v.FrameRate = 30; % Optional: Set frame rate
    open(v);

    figure('Name', ['Simulation - ', strrep(label, '_', ' ')]);
    for i = 1:N
        q_ref = q_des(i,:);
        qd_ref = qd_des(i,:);
        qdd_ref = qdd_des(i,:);
        e = q_ref - q;
        ed = qd_ref - qd;

        M_nom = nominal_robot.inertia(q);
        C_nom = nominal_robot.coriolis(q, qd);
        G_nom = nominal_robot.gravload(q)';

        % Robust min-max control law
        tau_pd = M_nom * (qdd_ref' + Kd*ed' + Kp*e');
        robust_term = robust_gain * sign(ed') .* abs(ed').^0.5;
        tau = tau_pd + C_nom*qd' + G_nom + robust_term;

        M_act = actual_robot.inertia(q);
        C_act = actual_robot.coriolis(q, qd);
        G_act = actual_robot.gravload(q)';

        qdd = (M_act \ (tau - C_act*qd' - G_act))';
        qd = qd + qdd*dt;
        q = q + qd*dt;

        q_log(i,:) = q;
        error_log(i,:) = e;

        actual_robot.plot(q, 'delay', 0, 'noshadow', 'nowrist');
        title(sprintf('%s - Time: %.2f s', strrep(label, '_', ' '), t(i)));
        drawnow limitrate;

        % Capture frame for video
        frame = getframe(gcf);
        writeVideo(v, frame);
    end

    close(v);
end

%% === Quintic Trajectory Generator ===
function [q, qd, qdd] = quintic_traj(q0, qf, t)
    n_joints = length(q0);
    n_steps = length(t);
    q = zeros(n_steps, n_joints);
    qd = zeros(n_steps, n_joints);
    qdd = zeros(n_steps, n_joints);
    t0 = t(1);
    tf = t(end);

    A = [1, t0, t0^2,   t0^3,    t0^4,     t0^5;
         0, 1,  2*t0,  3*t0^2,  4*t0^3,   5*t0^4;
         0, 0,  2,     6*t0,    12*t0^2,  20*t0^3;
         1, tf, tf^2,  tf^3,    tf^4,     tf^5;
         0, 1,  2*tf,  3*tf^2,  4*tf^3,   5*tf^4;
         0, 0,  2,     6*tf,    12*tf^2,  20*tf^3];

    for j = 1:n_joints
        b = [q0(j); 0; 0; qf(j); 0; 0];
        coeffs = A \ b;
        for i = 1:n_steps
            ti = t(i);
            q(i,j)   = [1, ti, ti^2, ti^3, ti^4, ti^5] * coeffs;
            qd(i,j)  = [0, 1, 2*ti, 3*ti^2, 4*ti^3, 5*ti^4] * coeffs;
            qdd(i,j) = [0, 0, 2, 6*ti, 12*ti^2, 20*ti^3] * coeffs;
        end
    end
end