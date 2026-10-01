function [X,U,K,costHistory] = ilqr(x0,U0,model,Ts,Q,R,Qf,Fmax)

x0 = x0(:);
U0 = min(Fmax,max(-Fmax,U0(:)'));
N = numel(U0);
if N < 1
    error('U0 must contain at least one control sample.');
end

% Initial trajectory
[X,U] = forward_pass(x0,zeros(4,N+1),U0,zeros(1,N),zeros(1,4,N), 0,model,Ts,Fmax);

maxIterations = 60;
alphas = [1 0.5 0.25 0.1 0.05 0.02 0.01 0.005 0.001];
relativeCostTolerance = 1e-5;
controlStepTolerance = 1e-5;

cost = trajectory_cost(X,U,Q,R,Qf);
costHistory = zeros(maxIterations+1,1);
costHistory(1) = cost;
acceptedIterations = 0;

for iteration = 1:maxIterations
    [A,B] = linearize_trajectory(X,U,model,Ts);
    [d,K] = backward_lqr(X,U,A,B,Q,R,Qf);

    accepted = false;
    for alpha = alphas
        [Xcandidate,Ucandidate] = forward_pass( ...
            x0,X,U,d,K,alpha,model,Ts,Fmax);
        candidateCost = trajectory_cost(Xcandidate,Ucandidate,Q,R,Qf);

        if ~isnan(candidateCost) && ~isinf(candidateCost) && ...
                candidateCost < cost - 1e-12*max(1,abs(cost))
            previousCost = cost;
            previousU = U;
            X = Xcandidate;
            U = Ucandidate;
            cost = candidateCost;
            acceptedIterations = acceptedIterations + 1;
            costHistory(acceptedIterations+1) = cost;
            accepted = true;
            break;
        end
    end

    % Stop when the line search can no longer improve the trajectory.
    if ~accepted
        break;
    end

    costImprovement = previousCost - cost;
    controlChange = max(abs(U-previousU));
    if costImprovement <= relativeCostTolerance*max(1,abs(previousCost)) || ...
            controlChange <= controlStepTolerance
        break;
    end
end

costHistory = costHistory(1:acceptedIterations+1);

% Return gains recomputed about the accepted nominal trajectory.
[A,B] = linearize_trajectory(X,U,model,Ts);
[~,K] = backward_lqr(X,U,A,B,Q,R,Qf);

end
