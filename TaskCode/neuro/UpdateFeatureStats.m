function Neuro = UpdateFeatureStats(Neuro)
% function Neuro = UpdateFeatureStats(Neuro)
% update estimate of mean and variance for each channel using Welford Alg
% Neuro 
% 	.NeuralFeatures - [ features x 1 ]
%   .FeatureStats - structure, which is updated

X = Neuro.NeuralFeatures';

% updates
w                           = 1;
Neuro.FeatureStats.wSum1    = Neuro.FeatureStats.wSum1 + w;
Neuro.FeatureStats.wSum2    = Neuro.FeatureStats.wSum2 + w*w;
meanOld                     = Neuro.FeatureStats.mean;
Neuro.FeatureStats.mean     = meanOld + (w / Neuro.FeatureStats.wSum1) * (X - meanOld);
Neuro.FeatureStats.S        = Neuro.FeatureStats.S + w*(X - meanOld).*(X - Neuro.FeatureStats.mean);
Neuro.FeatureStats.var      = Neuro.FeatureStats.S / (Neuro.FeatureStats.wSum1 - 1);

% ignore phase features
NumPhase = sum([Neuro.FilterBank.phase_flag]);
Neuro.FeatureStats.mean(1:NumPhase*128) = 0;
Neuro.FeatureStats.var(1:NumPhase*128) = 0;

end % UpdateNeuralStats