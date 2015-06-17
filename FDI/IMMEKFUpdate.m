function [X_Filter,P_Filter, innovation, S, X_IMM, P_IMM, modeProbs]= IMMEKFUpdate(X_IMM,P_IMM,u,FilterUpdateRate,measurement,Q,R_Noise,transMatrix,modeProbs)
                                                                      
%Determine number of models
m = length(X_IMM);

%Number of states
n = size(length(X_IMM{1}),1);

%*****Interacting/Mixing*****
c_j = zeros(1,m);
for j = 1:m
    for i = 1:m
        c_j(j) = c_j(j) + transMatrix(i,j)*modeProbs(i);
    end
end

modeProbs_ij = zeros(m,m);
for i = 1:m
    for j = 1:m
        modeProbs_ij(i,j) = 1/c_j(j)*transMatrix(i,j)*modeProbs(i);
    end
end

%Calculate mixed state and covariance
X0j = cell(1,m);
for j = 1:m
    X0j{j} = zeros(n,1);
    for i = 1:m
        X0j{j} = X0j{j} + X_IMM{i}*modeProbs_ij(i,j); 
    end
end

P0j = cell(1,m);
for j = 1:m
    P0j{j} = zeros(n,n);
    for i = 1:m
        P0j{j} = P0j{j} + modeProbs_ij(i,j)*(P_IMM{i} + (X_IMM{i}-X0j{j})*(X_IMM{i}-X0j{j})'); 
    end
end

%Perform Kalman filter updates on each filter and calculate likelihoods
for i = 1:m
    [X_IMM{i},P_IMM{i}, measurementPred, S{i}, innovation{i}]  = IMM_EKFPredictAndUpdate(X0j{i},P0j{i},i,u,FilterUpdateRate,measurement,Q{i},R_Noise);
    lamda(i)                               = gaussPDF(measurement, measurementPred, S{i}); 
end

%Mode probability update
c = 0;
for j = 1:m
    c = c + lamda(j) * c_j(j);
end

for j = 1:m
    modeProbs(j) = 1/c * lamda(j) * c_j(j); 
end

%Estimate and Covariance Combination
X_Filter = zeros(n,1);
for j = 1:m
    X_Filter = X_Filter + X_IMM{j} * modeProbs(j);
end

P_Filter = zeros(n,n);
for j = 1:m
    P_Filter = P_Filter +  modeProbs(j) * (P_IMM{j} + (X_IMM{j}-X_Filter)*(X_IMM{j}-X_Filter)');
end