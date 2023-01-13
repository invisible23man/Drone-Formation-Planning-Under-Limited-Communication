function consensus_i = consensus(eta_i,eta_Nis, lambda_i)
%consensus_i To ensure that the robots find the same
%   solution, a a consensus is applied where lambda is
%   a strictly positive multiplier determining how fast
%   the consensus is reached.

consensus_i = 0;
for j=1:size(eta_Nis, 1)
    consensus_i = consensus_i + (eta_i-eta_Nis(j,:));
end
consensus_i = (lambda_i*consensus_i)';
end
