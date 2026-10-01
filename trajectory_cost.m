function J = trajectory_cost(X,U,Q,R,Qf)
% J is the total cost corresponding to the control trajectory U from
% initial state x0 
% Use the objective function given in the project manual


N = numel(U);

J = 0.5*(X(:,N+1)'*Qf*X(:,N+1));
for k = 1:N
    xk = X(:,k);
    uk = U(k);
    J = J + 0.5*(xk'*Q*xk + uk'*R*uk);
end


end
