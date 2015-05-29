function [t,w]  =   LegendreNodesAndWeights(N)
ab              =   RecurrenceCoefficients(N+1);
ab(N+1,1)       =   0;
ab(N+1,2)       =   4*(N)^3/((2*N-1)*(2*N)^2);
% Call routine to calculate eigenvalues and eigenvectors
xw              =   gauss(N+1,ab);    
t               =   xw(:,1);
t(1)            =   -1;
t(N+1)          =   1;
w               =   xw(:,2);

end


% Recurrence coefficients for monic Jacobi polynomials.
function ab     =   RecurrenceCoefficients(N)

nu              =   0;
mu              =   2;

if (N == 1)
    ab          =   [nu mu];
    return;
end

N               =   N-1;
n               =   1:N;
nab             =   2*n;
A               =   [nu (0^2-0^2)*ones(1,N)./(nab.*(nab+2))];
n               =   2:N; 
nab             =   nab(n);
B1              =   1/3;
B               =   4*(n).^4./((nab.^2).*(nab+1).*(nab-1));
ab              =   [A' [mu; B1; B']];

end

% Gauss quadrature rule
function xw     =   gauss(N,ab)

N0              =   size(ab,1);

if (N0 < N)
    error('input array ab too short');
end

J               =   zeros(N);

for n = 1:N
    J(n,n)      =   ab(n,1);
end

for n = 2:N
  J(n,n-1)      =   sqrt(ab(n,2));
  J(n-1,n)      =   J(n,n-1);
end

[V,D]           =   eig(J);
[D,I]           =   sort(diag(D));
V               =   V(:,I);
xw              =   [D ab(1,2)*V(1,:)'.^2];

end