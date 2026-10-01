function [d,K] = backward_lqr(X,U,A,B,Q,R,Qf)
% A is (nx,nx,N), B is (nx,nu,N), and X is (nx,N+1).
% d is the feedforward term and K is the feedback gain in
%     u = U + alpha*d - K*(x-X).

N = numel(U);
nx = size(X,1);
nu = size(B,2);

% Quadratic and linear coefficients of the local cost-to-go.
S = zeros(nx,nx,N+1);
s = zeros(nx,N+1);
S(:,:,N+1) = Qf;
s(:,N+1) = Qf*X(:,N+1);

d = zeros(nu,N);
K = zeros(nu,nx,N);
for k = N:-1:1
    Ak = A(:,:,k);
    Bk = B(:,:,k);
    Skp1 = S(:,:,k+1);
    skp1 = s(:,k+1);

    G = R + Bk'*Skp1*Bk;
    G = 0.5*(G+G');

    K(:,:,k) = G \ (Bk'*Skp1*Ak);
    % For the stated positive quadratic running cost, stationarity gives
    % R*U + B'*s.  The minus sign printed before B'*s in the handout would
    % make the iLQR feedforward step non-descent for this cost convention.
    d(:,k) = -(G \ (R*U(k) + Bk'*skp1));

    S(:,:,k) = Q + Ak'*Skp1*(Ak-Bk*K(:,:,k));
    S(:,:,k) = 0.5*(S(:,:,k)+S(:,:,k)');
    s(:,k) = Q*X(:,k) + Ak'*skp1 + Ak'*Skp1*Bk*d(:,k);
end

% The project model has one force input; use the row-vector convention.
if nu == 1
    d = reshape(d,1,N);
end
end
