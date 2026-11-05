function u_Tk = u_min_sequence(A, B, T, xk, xf)
%u_min_sequence Calculate a minimum energy sequence of states
%   
%   Use the formula for the controllability grammian in discrete time to
%   obtain a T-many sequence of states that transfers the system from an
%   initial state xk to a final state xf.
%
%   A = nxn matrix, system dynamics in discrete time
%   B = nx1 matrix (vector!), for our non-autonomous single input system
%   T = trajectory length (number of timesteps)
%   xk = nx1 vector, system states at initial condition
%   xf = nx1 vector, system states at final condition
%
%   u_Tk = u_{k+T-1:k}, = [u_{k+T-1}; ...; u_k]

% First, calculate the controllability matrix, Q_T = [B AB ... A^{T-1}B]
% Hint: you could use a for loop here. There should be T-many columns.
n = size(A,1);
Q = zeros(n, T);

% One possibility is the following. Or calculate however you would like.
% for j=1:T
%     v = FILL_THIS_IN;
%     Q(:,j) = v;
% end

% The grammian is then:
W = FILL_THIS_IN;

% and plugging in our full formula:
u_Tk = FILL_THIS_IN;

end