function main_puma_pd_6dof
    % PD Control Simulation for PUMA 560 (All 6 joints)
    % Based on Armstrong et al. (1986) full dynamic model
    
    %% Simulation Parameters
    t_span = [0 10];      % Simulation time span [s]
    t_step = 0.001;       % Time step [s]
    t = t_span(1):t_step:t_span(2);
    
    % Initial conditions [q1 q2 q3 q4 q5 q6 qd1 qd2 qd3 qd4 qd5 qd6]
    q0 = zeros(12,1);     % Column vector for proper dimensions
    
    % Desired joint positions [rad]
    q_desired = [pi/2; pi/2; pi/2; 0; pi/4; 0]; % Column vector
    
    %% PD Controller Gains
    % Tuned for all 6 joints (larger gains for first 3 heavy joints)
    Kp = diag([500, 180, 50, 30, 30, 20]);    % Proportional gains
    Kd = diag([350, 50, 20, 10, 10, 5]);      % Derivative gains
    
    %% Solve the ODE
    [t, q] = ode45(@(t,q) puma_6dof_dynamics(t, q, q_desired, Kp, Kd), t, q0);
    
    %% Plot Results
    figure('Position', [100 100 1200 800]);
    
    % Joint Positions (q1-q3)
    subplot(2,2,1);
    plot(t, q(:,1:3), 'LineWidth', 1.5);
    hold on;
    plot(t, q_desired(1)*ones(size(t)), '--k', 'LineWidth', 1.5);
    plot(t, q_desired(2)*ones(size(t)), '--k', 'LineWidth', 1.5);
    plot(t, q_desired(3)*ones(size(t)), '--k', 'LineWidth', 1.5);
    hold off;
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('q1', 'q2', 'q3', 'Desired');
    title('PD Control - First 3 Joints (q1-q3)');
    grid on;
    
    % Joint Positions (q4-q6)
    subplot(2,2,2);
    plot(t, q(:,4:6), 'LineWidth', 1.5);
    hold on;
    plot(t, q_desired(4)*ones(size(t)), '--k', 'LineWidth', 1.5);
    plot(t, q_desired(5)*ones(size(t)), '--k', 'LineWidth', 1.5);
    plot(t, q_desired(6)*ones(size(t)), '--k', 'LineWidth', 1.5);
    hold off;
    xlabel('Time [s]');
    ylabel('Joint Angle [rad]');
    legend('q4', 'q5', 'q6', 'Desired');
    title('PD Control - Wrist Joints (q4-q6)');
    grid on;
    
    % Joint Velocities (qd1-qd3)
    subplot(2,2,3);
    plot(t, q(:,7:9), 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Joint Velocity [rad/s]');
    legend('qd1', 'qd2', 'qd3');
    title('Joint Velocities (q1-q3)');
    grid on;
    
    % Joint Velocities (qd4-qd6)
    subplot(2,2,4);
    plot(t, q(:,10:12), 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel('Joint Velocity [rad/s]');
    legend('qd4', 'qd5', 'qd6');
    title('Joint Velocities (q4-q6)');
    grid on;
end

function dqdt = puma_6dof_dynamics(t, q, q_desired, Kp, Kd)
    % PUMA 560 6-DOF dynamics with PD control
    
    % Current state (ensure column vectors)
    q_curr = q(1:6);      % Current joint positions (6x1)
    qd_curr = q(7:12);    % Current joint velocities (6x1)
    
    % PD control law (proper matrix dimensions)
    tau = -Kp*(q_curr - q_desired) - Kd*qd_curr;
    
    % Get dynamic matrices (M, C, G) - using Robotics Toolbox
    robot = puma560_armstrong();  % Initialize robot with Armstrong parameters
    M = robot.inertia(q_curr');   % 6x6 mass matrix (need row vector input)
    C = robot.coriolis(q_curr', qd_curr'); % 6x6 Coriolis matrix
    G = robot.gravload(q_curr')'; % 6x1 gravity vector
    
    % Compute accelerations
    qdd = M \ (tau - C*qd_curr - G);
    
    % Return state derivatives (column vector)
    dqdt = [qd_curr; qdd];
end

function robot = puma560_armstrong()
    % Initialize PUMA 560 with Armstrong et al. parameters
    
    mdl_puma560; % Load standard model
    L = p560.links; % Get links structure

    % Masses [kg] (Table 4)
    m = [0, 17.40, 4.80, 0.82, 0.34, 0.09]; % Link 1 mass not measured (set to 0)

    % Centers of Gravity [m] (Table 5)
    r = {
        [0, 0, 0],                     % Link 1
        [0.068, 0.006, -0.016],        % Link 2
        [0, -0.070, 0.014],            % Link 3
        [0, 0, -0.019],                % Link 4
        [0, 0, 0],                     % Link 5
        [0, 0, 0.032]                  % Link 6
    };

    % Inertias [kg·m²] (Table 6)
    I = {
        diag([0, 0, 0.25]),            % Link 1
        diag([0.130, 0.524, 0.539]),   % Link 2
        diag([0.086, 0.025, 0.086]),   % Link 3
        diag([1.8e-3, 1.8e-3, 1.9e-3]), % Link 4
        diag([0.3e-3, 0.3e-3, 0.4e-3]), % Link 5
        diag([0.15e-3, 0.15e-3, 0.04e-3]) % Link 6
    };

    % Motor inertias (Jm) (Table 6)
    Jm = [1.14, 4.71, 0.98, 0.200, 0.179, 0.193];

    % Gear ratios (Table 7)
    G = [62.61, 107.36, 53.69, 76.01, 71.91, 76.73];

    % Apply all parameter overrides
    for i = 1:6
        L(i).m = m(i);
        L(i).r = r{i};
        L(i).I = I{i};
        L(i).Jm = Jm(i);
        L(i).G = G(i);
    end

    % Create robot with corrected parameters
    robot = SerialLink(L, 'name', 'PUMA560 (Armstrong 6-DOF)');
end