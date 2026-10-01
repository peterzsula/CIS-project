function [Xnew,Unew] = forward_pass(x0,X,U,d,K,alpha,model,Ts,Fmax)
% X, U: nominal trajectory
% d: feedforward corrections (1,N)--> this corresponds to k in the class
% notes. Here denoted as d to avoid conflict with time index. 
% K: feedback gains of size (1,4,N)
% alpha: line-search step size  unew= unom +\alpha*k - K (xnew-xnom)

N = numel(U);

% Initialization
Xnew = zeros(4,N+1);
Unew = zeros(1,N);
Xnew(:,1) = x0;


for k = 1:N
    % Feedback acts on the deviation from the nominal state.  Wrap the
    % angular deviation so equivalent angles do not create a 2*pi jump.
    dx = Xnew(:,k) - X(:,k);
    dx(3) = atan2(sin(dx(3)),cos(dx(3)));

    uk = U(k) + alpha*d(k) - K(:,:,k)*dx;
    Unew(k) = min(Fmax,max(-Fmax,uk));
    Xnew(:,k+1) = discrete_step(Xnew(:,k),Unew(k),model,Ts);
end


end
