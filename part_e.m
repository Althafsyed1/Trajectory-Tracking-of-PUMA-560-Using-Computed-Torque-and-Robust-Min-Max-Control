function puma560_minmax_vs_nominal()
    clear; clc; close all;

    %% 1. Load Robot Model
    mdl_puma560;
    nominal_robot = p560.nofriction();
    actual_robot = p560.nofriction();

    % Add 2 kg payload to link 6 of actual robot
    payload_mass = 2.0;
    actual_robot.links(6).m = actual_robot.links(6).m + payload_mass;

    %% 2. Simulation Parameters
    dt = 0.01;
    T = 5;
    t = 0:dt:T;

    % Adjusted gains for better stability
    Kp = diag([150, 150, 100, 30, 30, 15]);
    Kd = diag([30, 30, 20, 5, 5, 2]);

    %% 3. Trajectory (Quintic)
    q0 = [0, 0, 0, 0, 0, 0];
    qf = [pi/4, pi/3, pi/2, 0, pi/4, 0];
    [q_des, qd_des, qdd_des] = quintic_traj(q0, qf, t);

    %% 4. Run Nominal Controller (No payload info) with torque limits
    tau_max = [97; 186; 89; 24; 20; 21]; % PUMA 560 torque limits from documentation
    [q_nom, err_nom_joint] = run_sim(nominal_robot, actual_robot, q_des, qd_des, qdd_des, Kp, Kd, t, dt, tau_max);

    %% 5. Run Robust Min-Max Controller
    [q_robust, err_robust_joint] = run_sim_robust(nominal_robot, actual_robot, q_des, qd_des, qdd_des, Kp, Kd, t, dt, tau_max);

    %% 6. Task-Space Error Comparison
    % Compute end-effector positions
    ee_nom = zeros(length(t),3);
    ee_robust = zeros(length(t),3);
    ee_des = zeros(length(t),3);
    
    for i = 1:length(t)
        ee_nom(i,:) = transl(nominal_robot.fkine(q_nom(i,:)))';
        ee_robust(i,:) = transl(nominal_robot.fkine(q_robust(i,:)))';
        ee_des(i,:) = transl(nominal_robot.fkine(q_des(i,:)))';
    end

    err_task_nom = ee_des - ee_nom;
    err_task_robust = ee_des - ee_robust;

    figure('Name','Task Space Tracking Error', 'Position', [100 100 800 600]);
    labels = {'X','Y','Z'};
    for i = 1:3
        subplot(3,1,i);
        plot(t, err_task_nom(:,i), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, err_task_robust(:,i), 'r-', 'LineWidth', 1.5);
        ylabel([labels{i} ' Error [m]']); grid on;
        legend('Nominal','Robust');
        title([labels{i} ' Task-Space Tracking Error']);
    end
    xlabel('Time [s]');

    %% 7. Joint-Space Tracking Error Comparison
    joint_labels = {'q_1','q_2','q_3','q_4','q_5','q_6'};
    figure('Name', 'Joint-Space Tracking Error', 'Position', [100 100 1200 800]);
    for j = 1:6
        subplot(3,2,j);
        plot(t, err_nom_joint(:,j), 'b-', 'LineWidth', 1.5); hold on;
        plot(t, err_robust_joint(:,j), 'r-', 'LineWidth', 1.5);
        xlabel('Time [s]');
        ylabel(['Error [rad]']);
        legend('Nominal', 'Robust');
        title(['Joint ', joint_labels{j}, ' Error']);
        grid on;
    end
end

%% --- Nominal Computed Torque Simulation (now with torque limits) ---
function [q_log, error_log] = run_sim(nominal, actual, q_des, qd_des, qdd_des, Kp, Kd, t, dt, tau_max)
    N = length(t); 
    q = q_des(1,:); 
    qd = zeros(1,6);
    q_log = zeros(N,6); 
    error_log = zeros(N,6);
    
    for i = 1:N
        e = q_des(i,:) - q;
        ed = qd_des(i,:) - qd;
        
        % Compute nominal control law
        M = nominal.inertia(q);
        C = nominal.coriolis(q, qd);
        G = nominal.gravload(q)';
        tau = M * (qdd_des(i,:)' + Kd*ed' + Kp*e') + C*qd' + G;
        
        % Apply torque limits to nominal controller too
        tau = max(min(tau, tau_max), -tau_max);
        
        % Compute actual dynamics
        M_act = actual.inertia(q); 
        C_act = actual.coriolis(q, qd); 
        G_act = actual.gravload(q)';
        
        % Regularized inverse for numerical stability
        reg = 1e-6*eye(6);
        qdd = ((M_act + reg) \ (tau - C_act*qd' - G_act))';
        
        % Semi-implicit Euler integration
        qd = qd + qdd*dt; 
        q = q + qd*dt;
        
        % Store results
        q_log(i,:) = q; 
        error_log(i,:) = e;
    end
end

%% --- Robust Min-Max Controller Simulation ---
function [q_log, error_log] = run_sim_robust(nominal, actual, q_des, qd_des, qdd_des, Kp, Kd, t, dt, tau_max)
    N = length(t); 
    q = q_des(1,:); 
    qd = zeros(1,6);
    q_log = zeros(N,6); 
    error_log = zeros(N,6);
    delta = 0.1;  % Boundary layer thickness
    rho = 0.5;    % Robustness gain
    
    for i = 1:N
        e = q_des(i,:) - q;
        ed = qd_des(i,:) - qd;
        
        % Nominal control law
        M = nominal.inertia(q);
        C = nominal.coriolis(q, qd);
        G = nominal.gravload(q)';
        v = qdd_des(i,:)' + Kd*ed' + Kp*e';
        tau_nominal = M*v + C*qd' + G;
        
        % Robust term
        norm_ed = norm(ed);
        if norm_ed > delta
            tau_robust = rho * ed'/norm_ed;
        else
            tau_robust = rho * ed'/delta;
        end
        
        % Combined control with saturation
        tau = tau_nominal + tau_robust;
        tau = max(min(tau, tau_max), -tau_max);
        
        % Compute actual dynamics
        M_act = actual.inertia(q); 
        C_act = actual.coriolis(q, qd); 
        G_act = actual.gravload(q)';
        
        % Regularized inverse
        reg = 1e-6*eye(6);
        qdd = ((M_act + reg) \ (tau - C_act*qd' - G_act))';
        
        % Integration
        qd = qd + qdd*dt; 
        q = q + qd*dt;
        
        % Store results
        q_log(i,:) = q; 
        error_log(i,:) = e;
    end
end

%% --- Quintic Trajectory Generator ---
function [q, qd, qdd] = quintic_traj(q0, qf, t)
    n = length(t); 
    nq = length(q0);
    q = zeros(n,nq); 
    qd = zeros(n,nq); 
    qdd = zeros(n,nq);
    t0 = t(1); 
    tf = t(end);
    
    A = [1 t0 t0^2 t0^3 t0^4 t0^5;
         0 1 2*t0 3*t0^2 4*t0^3 5*t0^4;
         0 0 2 6*t0 12*t0^2 20*t0^3;
         1 tf tf^2 tf^3 tf^4 tf^5;
         0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
         0 0 2 6*tf 12*tf^2 20*tf^3];

    for j = 1:nq
        b = [q0(j);0;0;qf(j);0;0];
        coeff = A\b;
        for i = 1:n
            ti = t(i);
            q(i,j) = [1 ti ti^2 ti^3 ti^4 ti^5]*coeff;
            qd(i,j) = [0 1 2*ti 3*ti^2 4*ti^3 5*ti^4]*coeff;
            qdd(i,j) = [0 0 2 6*ti 12*ti^2 20*ti^3]*coeff;
        end
    end
end