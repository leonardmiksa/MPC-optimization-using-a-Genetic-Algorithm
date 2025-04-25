clc; clear all; close all;

%Initialization of the MPC parameters:
Q11 = 1500;
p = 5;

%% Genetic Algorithm for Q and p optimization:
tracking_error_weight = 1;     %adjust as needed
rise_time_weight = 0.5;        %adjust as needed
%Both of these parameters could be improved using neural networks


%% Fitness function design:
data = readtable('mpc_fitness_table.csv');  %Loading data from MPC response parameters

%Normalization of tracking error and rise time:
norm_error = (data.Tracking_Error - min(data.Tracking_Error)) ./ ...
             (max(data.Tracking_Error) - min(data.Tracking_Error));
norm_rise  = (data.Rise_Time - min(data.Rise_Time)) ./ ...
             (max(data.Rise_Time) - min(data.Rise_Time));

%Combined base fitness:
fitness = tracking_error_weight * norm_error + rise_time_weight * norm_rise;

%Building the interpolant:
F_interp = scatteredInterpolant(data.p, data.Q11, fitness, 'linear', 'linear');
[p_fine, q_fine] = meshgrid(5:1:65, 500:100:10000);              %Fine grid
fitness_base = F_interp(p_fine, q_fine);                         %Base fitness evaluation

%Penalty on prediction horizon (saturating after p ≈ 55):
penalty_p = 1 ./ (1 + exp(-0.5 * (p_fine - 55)));

%Penalty on Q11 > 8500:
penalty_q = 0.6 * max(0, q_fine - 8500) / (10000 - 8500);

%Final fitness function surface (normalized):
FITNESS_FINAL = min(1.0, fitness_base + penalty_p + penalty_q);

%Fitness function plot:
figure;
surf(p_fine, q_fine, FITNESS_FINAL)
xlabel('p')
ylabel('Q_{11}')
zlabel('f(p, Q_{11})')
title('3D display of fitness function')
colormap('jet')  % or 'parula', 'viridis', etc.
shading interp

fitnessFunction = @(p, Q11) interp2(p_fine, q_fine, FITNESS_FINAL, p, Q11);

                    
%% GA parameters:                              
popSize = 20;            
numGenerations = 20;    
mutationRate = 0.5;      
crossoverRate = 0.95;    
pRange = [5, 65];        %Prediction horizon range
QRange = [500, 10000];   %Q11 range

%Initial population:
populationP = pRange(1) + 10 * rand(popSize, 1);
populationQ = QRange(1) + 800 * rand(popSize, 1);
% populationX = xRange(1) + (xRange(2) - xRange(1)) * rand(popSize, 1);
% populationY = yRange(1) + (yRange(2) - yRange(1)) * rand(popSize, 1);
%%% INITAL POPULATION GENERATED MANUALLY INSTEAD OVER WHOLE SPACE FOR SIMULATION PURPOSES %%%

%Tracking best fitness:
bestFitnessHistory = zeros(numGenerations, 1);
bestpQHistory = zeros(numGenerations, 2);

noImprovementLimit = 10; %Number of generations with no improvement before stopping
lastBestFitness = -Inf;
stagnationCount = 0;

ga_fig = figure('Name', 'Genetic Algorithm visualization', 'NumberTitle', 'off');

%% Genetic Algorithm loop
for gen = 1:numGenerations

    %% MPC reference signal:
    Ts = 0.02;                                %Sampling time 
    t = 0:Ts:5;
    ref = zeros(size(t));
    ref(t >= 1) = 1;

    %% State-space model of the DC motor  
    %Parameters of the system:                
    m = 0.03;                                             %Muscle mass 
    b = 0.5;                                              %Friction coefficient 
    k = 0.04;                                             %Spring constant                           
    K = 0.6;                                              %Gain of the pneumatic valve 
    tau = 0.05;                                           %Time constant of the pneumatic valve 
    gama = 2000;                                          %Proportionality constant
    A = 0.000314;                                         %Cross-section area of the muscle

    %State-space matrices:
    A = [0      1       0;
         -k/m   -b/m    gama*A/m;
        0      0       -1/tau];
 
    B = [0;
         0;
         K/tau];
 
    C = [1    0    0];

    D = 0;

    %Simulation setup:
    steps = 150; 
    sim_time = 5;                  %Total simulation time (seconds)
    Ts = sim_time / steps;         %Sampling time

    [Ad, Bd] = c2d(A, B, Ts);      %Discretization

    %% MPC parameters
    prediction_horizon = p;       %Prediction horizon (p)
    control_horizon = 6;           %Control horizon (m)

    %Regulation matrices:
    Q = [Q11    0    0;           
         0    10    0;               
         0    0    100]          
    R = 1e-12;        

    n_x = length(Q);                %Number of states
    p = prediction_horizon
    m = control_horizon;

    %% Initial conditions
    x0 = [0; 0; 0];                 %Initial state of the system (can be replaced with x0 = zeros(:, 1))
    x = x0;                         %Current state
    u_prev = 0;                     %Previous input

    %% Simulation setup
    x_trajectory = zeros(n_x, steps);           %Store states for plotting
    u_trajectory = zeros(1, steps);             %Store inputs for plotting

    %% Optimization setup
    %Quadratic cost matrices:
    Q_bar = kron(eye(p), Q);                    
    R_bar = kron(eye(m), R);                    
    H = blkdiag(Q_bar, R_bar);                  %Quadratic cost matrix

    %Prediction matrices:
    Phi = zeros(n_x*p, n_x);                        %State prediction matrix
    Gamma = zeros(n_x*p, m);                      %Control influence matrix

    %Filling the prediction matrices
    for i = 1:p
       Phi(n_x*i-2 : n_x*i, :) = Ad^i;                      
       for j = 1:min(i, m)
           Gamma(n_x*i - 2:n_x*i, j) = Ad^(i - j) * Bd;
       end
    end


    %Constraints:
    U_max = 6000;                                        %Maximum input
    U_min = -6000;                                       %Minimum input
    delta_U_max = 200;                                   %Maximum change in input
    x_max = [100; 100; 100];                             %Maximum state values
    x_min = [-100; -100; -100];                          %Minimum state values

    predicted_states = zeros(n_x, p, steps);             %3D array to store predicted states
    control_inputs = zeros(m, steps);                    %Store control inputs

    u_opt = zeros(m, 1);                                 %Optimal control vector initialization

    %% MPC Loop
    for k = 1:steps    
        current_time = k * Ts;
    
        %% IMPORTANT!
    %Following reference variables are named randomly
    %Additional letters don't have any specific meaning
    reft = zeros(n_x * p, 1);
    
    %Sequence of step reference changes:
    if current_time >= 1
        reft = 1 * ones(n_x * p, 1);  
    end
    

        %% MPC setup for quadprog
    x_pred = Phi * x + Gamma * u_opt;                       %(n_x*p) X (n_x) + (n_x*p) X (m*n_u)
    predicted_states(:, :, k) = reshape(x_pred, [n_x, p]);

    %Defining quadratic cost matrix H:
    H = Gamma' * Q_bar * Gamma + R_bar;                     %m X m

    %Defining linear cost vector f:
    f = (Gamma' * Q_bar * (Phi * x - reft))';               %HERE REPLACE THE reft VARIABLE WITH ANOTHER REFERENCE IF NEEDED!
                                                            %refkk FOR IMPORTED, reft FOR CUSTOMLY DEFINED
                                                            
    %Adjust bounds for states:
    b_x_upper = repmat(x_max, p, 1) - Phi * x;              %Adjusted upper state bounds (for state constraints)
    b_x_lower = repmat(x_min, p, 1) - Phi * x;              %Adjusted lower state bounds (for state constraints)

    %Combining constraints:
    F_total = [Gamma; -Gamma; eye(m); -eye(m)];             %Input constraints
    F_x = eye(n_x * p);                                     
    F_total = [F_x * Gamma; -F_x * Gamma; eye(m); -eye(m)]; %State constraints

    b_total = [b_x_upper; -b_x_lower; U_max * ones(m, 1); -U_min * ones(m, 1)];
    

        %% Solving the quadratic programming problem:
    %options = optimset('Display', 'off');
    options = optimoptions('quadprog', ...
    'Algorithm', 'interior-point-convex', ...
    'ConstraintTolerance', 1e6, ...
    'Display', 'iter');
    %u_opt = quadprog(H, f, F_total, b_total, [], [], [], [], []);
    u_opt = quadprog(H, f, [], [], [], [], [], [], []);

    control_inputs(:, k) = u_opt(1:m);                      %Storing control inputs for the horizon
    u = u_opt(1);                                           %Extracting the first control input
    x = Ad * x + Bd * u;                                    %Applying the control input and update the state

    %Saving trajectories:
    x_trajectory(:, k) = x;
    u_trajectory(k) = u;

    %Updating previous input:
    u_prev = u;
    
    disp('State Prediction Error:');
    disp(norm(Phi * x - ref));
    disp('Current step:');
    disp(k);
    end

    %% Plotting results
    time = (0:steps-1) * Ts + 2*Ts;
    %xlqr can be replaced with reference signal; xlqr left for comparison
    figure;
    subplot(3, 1, 1);
    plot(time, x_trajectory(1, :), 'b', 'LineWidth', 2, 'DisplayName', 'Linear displacement');      %1st state variable
    hold on;
    t = t';
    plot(t, ref, 'r--', 'LineWidth', 1, 'DisplayName', 'Reference signal'); 
    title('Linear displacement response');
    ylabel('x_1 [m]');
    xlim([0, 5]);
    grid on;
    legend;

    subplot(3, 1, 2);
    plot(time, x_trajectory(2, :), 'g', 'LineWidth', 2, 'DisplayName', 'Linear speed');             %2nd state variable 
    title('Linear speed response');
    ylabel('x_2 [m/s]');
    xlim([0, 5]);
    grid on;
    legend;

    subplot(3, 1, 3);
    plot(time, x_trajectory(3, :), 'm', 'LineWidth', 2, 'DisplayName', 'Air flow');                 %3rd state variable
    title('Air flow response');
    xlabel('Time [s]');
    ylabel('x_3 [m^3/s]');
    xlim([0, 5]);
    grid on;
    legend;

    %% Extracting response parameters for Genetic Algorithm:
    displacement = x_trajectory(1, :);    %Extracting the first row of response matrix (displacement)

    steady_state_error = abs(displacement(end) - ref(end));  %Calculating tracking error
    overshoot = 100*( (max(displacement) - displacement(end)) / displacement(end) ); %Calculating overshoot [%]
    %Calculating rising time:
    %Rising edge detection
    step_idx = find(diff(ref) ~= 0, 1, 'first') + 1;
    step_time = t(step_idx);
    %Step value:
    step_size = ref(end) - ref(step_idx - 1);           %Final value - initial value
    %Thresholds:
    lower_thresh = ref(step_idx - 1) + 0.1 * step_size;
    upper_thresh = ref(step_idx - 1) + 0.9 * step_size;

    %Finding where displacement crosses 10% and 90% AFTER step:
    idx_start = find(displacement(step_idx:end) >= lower_thresh, 1, 'first') + step_idx - 1;
    idx_end   = find(displacement(step_idx:end) >= upper_thresh, 1, 'first') + step_idx - 1;

    %Computing rise time:
    if ~isempty(idx_start) && ~isempty(idx_end)
        rise_time = time(idx_end) - time(idx_start);
    else
        rise_time = NaN;  % Couldn't compute
    end
    



    
    %% Evaluate fitness (negated function)
    fitnessValues = fitnessFunction(populationP, populationQ);
    
    %Saving the best individual:
    [bestFitness, bestIdx] = min(fitnessValues);
    bestFitnessHistory(gen) = bestFitness; 
    bestpQHistory(gen, :) = [populationP(bestIdx), populationQ(bestIdx)];
    
    %Checking for improvement:
    if abs(bestFitnessHistory(gen) - lastBestFitness) < 1e-4
        stagnationCount = stagnationCount + 1;
    else
        stagnationCount = 0;            %Resetting if improvement occurs
    end
    lastBestFitness = bestFitnessHistory(gen);

    %Stopping if no improvement for last 10 generations:
    if stagnationCount >= noImprovementLimit
        disp('Stopping early due to no improvement.');
        break;
    end

    %Stopping if max generations reached:
    if gen >= numGenerations
        break;
    end
    
    
    %Tournament selection:
    newPopulationp = zeros(popSize, 1);
    newPopulationQ = zeros(popSize, 1);
    for i = 1:popSize
        candidates = randi([1 popSize], 2, 1);        %Selecting two individuals
        [~, bestIdx] = min(fitnessValues(candidates));
        newPopulationp(i) = populationP(candidates(bestIdx));
        newPopulationQ(i) = populationQ(candidates(bestIdx));
    end
    
    %Arithmetic Crossover:
    for i = 1:2:popSize-1
        if rand < crossoverRate
            alphaCrossover = rand;                    %Blend factor
            %Blend parents:
            newPopulationp(i) = alphaCrossover * newPopulationp(i) + (1 - alphaCrossover) * newPopulationp(i+1);
            newPopulationp(i+1) = (1 - alphaCrossover) * newPopulationp(i) + alphaCrossover * newPopulationp(i+1);
            newPopulationQ(i) = alphaCrossover * newPopulationQ(i) + (1 - alphaCrossover) * newPopulationQ(i+1);
            newPopulationQ(i+1) = (1 - alphaCrossover) * newPopulationQ(i) + alphaCrossover * newPopulationQ(i+1);
        end
    end
    
    %Gaussian mutation:
    if lastBestFitness < 0.035
        mutationMask = rand(popSize, 1) < mutationRate;
        mutationAmountp = 1 * randn(popSize, 1);      %Small random change
        mutationAmountQ = 10 * randn(popSize, 1);     %Small random change
        newPopulationp = newPopulationp + mutationMask .* mutationAmountp;
        newPopulationQ = newPopulationQ + mutationMask .* mutationAmountQ;
    else
        mutationMask = rand(popSize, 1) < mutationRate;
        mutationAmountp = 10 * randn(popSize, 1);     %Small random change
        mutationAmountQ = 1500 * randn(popSize, 1);   %Small random change
        newPopulationp = newPopulationp + mutationMask .* mutationAmountp;
        newPopulationQ = newPopulationQ + mutationMask .* mutationAmountQ;
    end
    
    %Keeping values in range:
    newPopulationp = max(pRange(1), min(pRange(2), newPopulationp));
    newPopulationQ = max(QRange(1), min(QRange(2), newPopulationQ));
    
    %Updating population:
    populationP = newPopulationp;
    populationQ = newPopulationQ;
    
    %% GA plot:
    %Visualization:
    figure(ga_fig);   
    clf;

    [X, Y] = meshgrid(linspace(pRange(1), pRange(2), 50), linspace(QRange(1), QRange(2), 50));
    Z = interp2(p_fine, q_fine, FITNESS_FINAL, X, Y);     %Real fitness surface
    subplot(2,1,1);
    surf(X, Y, Z, 'EdgeColor', 'none'); hold on;
    colormap(jet);
    shading interp;
    colorbar;
            
    %Evaluating interpolated fitness at best point:
    bestFitness = interp2(p_fine, q_fine, FITNESS_FINAL, ...
                          bestpQHistory(gen,1), bestpQHistory(gen,2));

    scatter3(bestpQHistory(gen, 1), bestpQHistory(gen, 2), bestFitness, 100, 'm', 'filled');

    %Evaluating fitness for population using interp2:
    populationFitness = interp2(p_fine, q_fine, FITNESS_FINAL, populationP, populationQ);
    scatter3(populationP, populationQ, populationFitness, 100, 'k', 'filled');

    title(['Generation ' num2str(gen)]);
    xlabel('p'); ylabel('Q_{11}'); zlabel('f(p, Q_{11})');
    xlim([5 65]);
    ylim([500 10000]);
    legend('Fitness function', 'Optimal solution', 'Population');
    view(0, 90);

    %Fitness history:
    subplot(2,1,2);
    plot(1:gen, bestFitnessHistory(1:gen), 'r', 'LineWidth', 2);
    title('Optimal solutions');
    xlabel('Generation');
    ylabel('min f(p, Q_{11})');
    xlim([1 numGenerations]);
    %Removing non-integer x-ticks:
    xt = get(gca, 'XTick');          %Getting current x-ticks
    xt_int = xt(mod(xt,1) == 0);     %Keeping only integer ticks
    set(gca, 'XTick', xt_int);       %Setting new x-ticks
    ylim([min(Z(:)), max(Z(:))]);
    
    
    %Updating Q11 and p:
    p = ceil(bestpQHistory(gen, 1));
    Q11 = ceil(bestpQHistory(gen, 2));
    
    pause(1);
end

