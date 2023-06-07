function mean_computation_time = computationTime(runs, tspan, filterFunc, mu0, cov0, y, J, dt, Q, R)
    
    run_time = zeros(1,length(tspan)-1); 
    % for j = 1:runs
        mu = zeros(length(mu0),length(tspan)-1);
        cov = zeros(length(mu0),length(mu0),length(tspan)-1);
        mu(:,1) = mu0;
        cov(:,:,1) = cov0;
        
        for i = 2:length(tspan)-1
            t = tspan(i);
            if t < 50
                M = [0.0001;0;0];
            elseif t > t(end)-50
                M = [-0.0001;0;0];
            else
                M = [0;0;0];
            end
            tic;
            [mu(:,i),cov(:,:,i)] = filterFunc(mu(:,i-1), cov(:,:,i-1), y(:,i), M, J, dt, Q, R);
            run_time(i) = toc; 
        end
        
    % end 
    mean_computation_time = mean(run_time);
end