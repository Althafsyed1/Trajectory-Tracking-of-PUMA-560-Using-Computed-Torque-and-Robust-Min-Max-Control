%% PUMA 560 Dynamics Model - Armstrong Parameters + Validation
% Reference: Armstrong, Khatib, Burdick (Stanford, 1986)
clear; clc; close all;

fprintf('=== Loading PUMA 560 with exact parameters from paper ===\n');

% Load and remove friction/motor dynamics
mdl_puma560;
p560 = p560.nofriction();  % Remove friction, Jm, and G effects
L = p560.links;

% === Overwrite parameters from Armstrong et al. ===

% Masses (kg) - Table 4
m = [0, 17.40, 4.80, 0.82, 0.34, 0.09];

% Centers of Gravity (m) - Table 5
r = {
    [0, 0, 0];                     % Link 1
    [0.068, 0.006, -0.016];        % Link 2
    [0, -0.070, 0.014];            % Link 3
    [0, 0, -0.019];                % Link 4
    [0, 0, 0];                     % Link 5
    [0, 0, 0.032];                 % Link 6
};

% Inertia Tensors (kg·m²) - Table 6 (diagonal)
I = {
    diag([0, 0, 0.25]);             % Link 1
    diag([0.130, 0.524, 0.539]);    % Link 2
    diag([0.086, 0.025, 0.086]);    % Link 3
    diag([1.8e-3, 1.8e-3, 1.9e-3]); % Link 4
    diag([0.3e-3, 0.3e-3, 0.4e-3]); % Link 5
    diag([0.15e-3, 0.15e-3, 0.04e-3]) % Link 6
};

% Overwrite dynamic properties (no motor inertia or gearing)
for i = 1:6
    L(i).m = m(i);
    L(i).r = r{i};
    L(i).I = I{i};
    L(i).Jm = 0;       % Remove motor inertia
    L(i).G = 1;        % Remove gear ratio
end

% Rebuild robot with updated parameters
p560_paper = SerialLink(L, 'name', 'PUMA560 (Armstrong Params)');
fprintf('Robot model created with exact parameters from paper.\n\n');

%% 2. Compute Dynamics Matrices at Test Configuration
fprintf('=== Computing dynamics matrices at test configuration ===\n');
q_test = [0, pi/4, pi/2, 0, 0, 0];
qd_test = [0.1, 0.2, 0.3, 0, 0, 0];
qdd_test = zeros(1,6);

% A(q): Inertia matrix
A = p560_paper.inertia(q_test);
fprintf('Inertia matrix (A) computed:\n'); disp(A);

% B(q, q̇): Coriolis matrix
B = p560_paper.coriolis(q_test, qd_test);
fprintf('\nCoriolis matrix (B) computed:\n'); disp(B);

% G(q): Gravity vector
G = p560_paper.gravload(q_test)';
fprintf('\nGravity vector (G) computed:\n'); disp(G);

% Compute torque from model: τ = A*qdd + B*qd + G
tau_model = A*qdd_test' + B*qd_test' + G;
fprintf('\nTorque from dynamics model (A*qdd + B*qd + G):\n'); disp(tau_model');

%% 3. Validate Model Against RNE
fprintf('\n=== Validating model against RNE method ===\n');
tau_rne = p560_paper.rne(q_test, qd_test, qdd_test);
fprintf('Torque from RNE method:\n'); disp(tau_rne);

% Compute difference
diff = tau_model' - tau_rne;
fprintf('\nDifference between model and RNE:\n'); disp(diff);
fprintf('Maximum absolute difference: %g Nm\n', max(abs(diff)));

if max(abs(diff)) < 1e-3
    fprintf(' Model matches RNE within tolerance\n');
else
    fprintf(' Model does NOT match RNE\n');
end

%% 4. Matrix Property Checks
fprintf('\n=== Verifying matrix properties ===\n');
sym_err = norm(A - A', 'fro');
fprintf('Symmetric error (||A - Aᵀ||_F): %g\n', sym_err);
if sym_err < 1e-10
    fprintf(' Inertia matrix A is symmetric (as expected)\n');
else
    fprintf(' Inertia matrix A is NOT symmetric (check again)\n');
end

[~, p] = chol(A);
if p == 0
    fprintf('Inertia matrix A is positive definite (correct)\n');
else
    fprintf('Inertia matrix A is NOT positive definite\n');
end

fprintf('Condition number of A: %g\n', cond(A));

%% 5. Display Parameters for Verification
fprintf('\n=== Displaying all parameters for verification ===\n');
fprintf('\nLink masses (kg):\n'); disp(m);

fprintf('\nLink centers of gravity (m):\n');
for i = 1:6
    fprintf('Link %d: [%g, %g, %g]\n', i, r{i});
end

fprintf('\nLink inertia tensors (kg·m²):\n');
for i = 1:6
    fprintf('Link %d:\n'); disp(I{i});
end