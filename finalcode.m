clc;
clear;
close all;

num_vehicles1 = 5; % Number of vehicles
% tasks = 10; % Number of tasks

Niter = 20;
Nreal = 1;

for j = 1:length(num_vehicles1)
    num_vehicles=num_vehicles1(j);

    %Calculate the transmission rate (Rk,m)
    Bmax = 0.18*(10^6); % Max bandwidth
    Pmax = 1000;  % Max transmit power
    pk = (Pmax/num_vehicles)*ones(num_vehicles,1); % Transmit power of vehicle k
    wk = ones(num_vehicles, 1); % Beamforming vector of vehicle k
    N0 = 2*(10^(-12)); % Gaussian noise variance 

    x = 100;
    y = 10;

%     % IRS Parameters
%     N = 4; % Number of elements in the IRS
%     beta_e = rand(N, 1); % Reflective coefficient of each element
%     theta_e = 2*pi*rand(N, 1); % Phase shift of each element
%     x_ris = [49,50,51,52];  %x-coordinates of RIS
%     y_ris = 10*ones(N,1);  %y-coordinates of RIS
        
    % Parameters         
    d0 = 100;      % Max communication distance
    rho = 0.8;    % Distortion factor
    lambda = -4;  % Path fading factor
    Zeta = 3;   % Rician factor
        
    % MEC Server Location
    xs = 0; % x-coordinate of the MEC server
    ys = 0; % y-coordinate of the MEC server
    
    for r = 1:Nreal
        xt_k = x*rand(num_vehicles, 1); % x-coordinates of the vehicles at time slot t
        yt_k = y*rand(num_vehicles, 1); % y-coordinates of the vehicles at time slot t
        
        % Initializing channel vectors
        hk_m = zeros(num_vehicles, 1);
%         G = zeros(num_vehicles, N);
        hk = zeros(num_vehicles, 1);
%         hr_m = zeros(N,1);
            
        % Calculation of R_k_m and other parameters
        S = sum(pk .* abs(wk .* hk).^2);
        R_k_m = zeros(num_vehicles, 1);
       
        % Parameters
        O = 1000; % Calculation density
        tau_k_m = 0.01; % The energy factors of vehicle k to the MEC server
        f_s = 10*(10^12); % Frequency of MEC server
        f_l = 10^12; % Local computing frequency
        k_mec = 10^(-30); % CPU factor in the MEC server
        k_loc = 5*10^(-28); % CPU factor in local vehicles

        ucr_max = optimproblem('ObjectiveSense','maximize');
        eta_l = optimvar('eta_l',[num_vehicles1(j),1],'Type','continuous','LowerBound',0,'UpperBound',1);
        eta_e = optimvar('eta_e',[num_vehicles1(j),1],'Type','continuous','LowerBound',0,'UpperBound',1);
    
        options = optimoptions('fmincon','Algorithm','sqp','ConstraintTolerance',1e-6,'MaxIterations',1500,'MaxFunctionEvaluations',2000);
%         x0.B_k_m = (Bmax/num_vehicles)*ones(num_vehicles,1);%*rand(num_vehicles,1); % Bandwidth between vehicle k and the MEC server
    
        ucr1_max = optimproblem('ObjectiveSense','maximize');
        alpha_k_i = optimvar('alpha_k_i',[num_vehicles1(j),1],'Type','continuous','LowerBound',0,'UpperBound',1);

        ucr2_max = optimproblem('ObjectiveSense','maximize');
        phi_k_m = optimvar('phi_k_m',[num_vehicles1(j),1],'Type','continuous','LowerBound',0,'UpperBound',1);
    
        for iter=1:Niter
    
            if iter == 1
                x0.eta_e = rand(num_vehicles,1);
                x0.eta_l = rand(num_vehicles,1);
                x0.alpha_k_i = rand(num_vehicles,1);
                x0.phi_k_m = rand(num_vehicles,1);
            else
                x0.eta_e = eta_e_values(iter-1)*ones(num_vehicles,1);
                x0.eta_l = eta_l_values(iter-1)*ones(num_vehicles,1);
                x0.alpha_k_i = alpha_values(iter-1)*ones(num_vehicles,1);
                x0.phi_k_m  = phi_k_m_values(iter-1)*ones(num_vehicles,1);
            end
                L = 0;
                E = 0;
                % Channel vector from each vehicle to the MEC server
                for k = 1:num_vehicles
                    distance = sqrt((xt_k(k) - xs)^2 + (yt_k(k) - ys)^2); % Distance between vehicle k and MEC server
                    
                    if distance <= d0
                        h_LoS_k_m = exp(-1j*(2*pi/d0)*distance); % LoS component
                        h_NLoS_k_m = (randn(1) + 1j*randn(1)) / sqrt(2); % NLoS component
                        
                        hk_m(k) = sqrt(rho * (distance^lambda)) * (Zeta / (1 + Zeta) * h_LoS_k_m + (1 / (1 + Zeta)) * h_NLoS_k_m);
                        
%                         for i = 1:4
%                             % Channel gain from IRS to the MEC server (hr,m)
%                             distance_r_m(i) = sqrt((xs - x_ris(i))^2 + (ys - y_ris(i))^2); % Distance between IRS and MEC server
%                             hr_m(i) = (distance_r_m(i)^lambda);
%                         
%                             % Channel gain between IRS and vehicles (G)
%                             distance_k_r(i) = sqrt((xt_k(k) - x_ris(i))^2 + (yt_k(k) - y_ris(i))^2); % Distance between vehicle k and RIS
%                             G(i) = (distance_k_r(i)^lambda);
%                         end
                        
%                         % Calculate the IRS parameter matrix 
%                         phi = diag(beta_e .* exp(1j * theta_e));
                
                        % Total channel vector (hk)
                        hk = hk_m ; % + G * phi * hr_m
    
                        R_k_m(k) =  x0.phi_k_m(k)*Bmax *(log2(1 + (pk(k) * abs(wk(k) * hk(k))^2) / ( N0))); %Transmition rate %%%S-(pk(k) * abs(wk(k) * hk(k))^2)
                        kappa = 1; % Utility parameter
                        l = 1; % Utility parameter
                        U_n(k) = kappa .* log(1 + l .* R_k_m(k)); % Utility
                    
                        X_k_i(k) = randi([100000,500000],1,1)*8; % Tasks data size (100-500 KB)
                        L_k_m_i(k) = X_k_i(k)/R_k_m(k); % Transmition latency
                        
                        L_k_i_mec(k) = (X_k_i(k)*O/f_s) + L_k_m_i(k); % Total execution time
                        %Ignored Lback as it is very small
                        E_k_i_mec(k) = tau_k_m*L_k_m_i(k) + k_mec*(f_s^2)*O*X_k_i(k); % Energy consumption
                        
                        L_k_i_loc(k) = (X_k_i(k)*O/f_l); % Local execution time
                        E_k_i_loc(k) = k_loc*(f_l^2)*O*X_k_i(k); % Energy consumption
                    
                        % total execution latency and total energy consumption
                        L = L + x0.alpha_k_i(k) * L_k_i_loc(k) + (1 - x0.alpha_k_i(k)) * L_k_i_mec(k);
                        E = E + x0.alpha_k_i(k) * E_k_i_loc(k) + (1 - x0.alpha_k_i(k)) * E_k_i_mec(k);
                    else
                        hk_m(k) = 0; % Vehicle is out of communication range
                    end
                end
                cost(j) = eta_l(j)*L + eta_e(j)*E;
       
                ucr_max.Objective = (U_n(j)/cost(j));
                
                ucr_max.Constraints.cost = cost(j) >= 0;
                ucr_max.Constraints.energyfactor = eta_e(j) <= 1;
                ucr_max.Constraints.latencyfactor = eta_l(j) <= 1;
                ucr_max.Constraints.addition = eta_l(j)+eta_e(j) == 1;
            
                [sol, ~]= solve(ucr_max,x0,'options',options);
            
                x0.eta_l = sol.eta_l;
                x0.eta_e = sol.eta_e;
                eta_l_values(iter) = sum(x0.eta_l)/num_vehicles;
                eta_e_values(iter) = sum(x0.eta_e)/num_vehicles;
            
                for k = 1:num_vehicles
                    % total execution latency and total energy consumption
                    L = 0;
                    E = 0;
                    L = L + alpha_k_i(k) * L_k_i_loc(k) + (1 - alpha_k_i(k)) * L_k_i_mec(k);
                    E = E + alpha_k_i(k) * E_k_i_loc(k) + (1 - alpha_k_i(k)) * E_k_i_mec(k);
                end
                cost(j) = x0.eta_l(j)*L + x0.eta_e(j)*E;
               
                ucr1_max.Objective = (U_n(j)/cost(j));
                
                ucr1_max.Constraints.cost = cost(j) >= 0;
                ucr1_max.Constraints.alpha_k_i = alpha_k_i(j) <= 1;
                 
                [sol1, ~]= solve(ucr1_max,x0,'options',options);
            
                x0.alpha_k_i = sol1.alpha_k_i;
                alpha_values(iter) = sum(x0.alpha_k_i)/num_vehicles;
    
                for k = 1:num_vehicles
                     R_k_m(k) =  x0.phi_k_m(k)*Bmax*(log2(1 + (pk(k) * abs(wk(k) * hk(k))^2) / ( N0))); %Transmition rate %%%S-(pk(k) * abs(wk(k) * hk(k))^2)
                        kappa = 1; % Utility parameter
                        l = 1; % Utility parameter
                        U_n(k) = kappa .* log(1 + l .* R_k_m(k)); % Utility
                    
                        X_k_i(k) = randi([100000,500000],1,1)*8; % Tasks data size (100-500 KB)
                        L_k_m_i(k) = X_k_i(k)/R_k_m(k); % Transmition latency
                        
                        L_k_i_mec(k) = (X_k_i(k)*O/f_s) + L_k_m_i(k); % Total execution time
                        %Ignored Lback as it is very small
                        E_k_i_mec(k) = tau_k_m*L_k_m_i(k) + k_mec*(f_s^2)*O*X_k_i(k); % Energy consumption
                        
                        L_k_i_loc(k) = (X_k_i(k)*O/f_l); % Local execution time
                        E_k_i_loc(k) = k_loc*(f_l^2)*O*X_k_i(k); % Energy consumption
                    
                        % total execution latency and total energy consumption
                        L = L + x0.alpha_k_i(k) * L_k_i_loc(k) + (1 - x0.alpha_k_i(k)) * L_k_i_mec(k);
                        E = E + x0.alpha_k_i(k) * E_k_i_loc(k) + (1 - x0.alpha_k_i(k)) * E_k_i_mec(k);
                end
        
                cost(j) = x0.eta_l(j)*L + x0.eta_e(j)*E;
                    
                ucr2_max.Objective = (U_n(j)/cost(j));
                
                ucr2_max.Constraints.cost = cost(j) >= 0;
                ucr2_max.Constraints.phi_k_m = (phi_k_m(j)) <= 1;

                [sol2, ~]= solve(ucr2_max,x0,'options',options);
            
                x0.phi_k_m = sol2.phi_k_m;
                phi_k_m_values(iter) = sum(x0.phi_k_m)/num_vehicles;
               
                for k = 1:num_vehicles
                    % total execution latency and total energy consumption
                    latency(k) = x0.alpha_k_i(k) * L_k_i_loc(k) + (1 - x0.alpha_k_i(k)) * L_k_i_mec(k);
                    energy(k) = x0.alpha_k_i(k) * E_k_i_loc(k) + (1 - x0.alpha_k_i(k)) * E_k_i_mec(k);
                end
                latency_values(iter) = mean(latency);
                energy_values(iter) = mean(energy);
                cost_values(iter) = eta_e_values(iter)*energy_values(iter)+eta_l_values(iter)*latency_values(iter);
                utility_values(iter) = mean(U_n);
                ucr_values(iter) = utility_values(iter)/cost_values(iter);
                bandwidth_values(iter) = Bmax*phi_k_m_values(iter);
        end
        avg_eta_l(r)=mean(eta_l_values);
        avg_eta_e(r)=mean(eta_e_values);
        avg_alpha(r)=mean(alpha_values);
        avg_phi(r)=mean(phi_k_m_values);
        avg_latency(r)=mean(latency_values);
        avg_energy(r)=mean(energy_values);
        avg_cost(r)=mean(cost_values);
        avg_utility(r)=mean(utility_values);
    end
    final_eta_e(j) = mean(avg_eta_e);
    final_eta_l(j) = mean(avg_eta_l);
    final_alpha(j) = mean(avg_alpha);
    final_phi(j) = mean(avg_phi);
    final_bandwidth(j) = final_phi(j)*Bmax;
    final_latency(j) = mean(avg_latency);
    final_energy(j) = mean(avg_energy);
    final_cost(j) = mean(avg_cost);
    final_utility(j) = mean(avg_utility);
    final_ucr(j) = final_utility(j)/final_cost(j);
end
iteration_values = 1:Niter;