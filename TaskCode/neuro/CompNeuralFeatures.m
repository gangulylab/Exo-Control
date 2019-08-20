function Neuro = CompNeuralFeatures(Neuro),
% Neuro = CompNeuralFeatures(Neuro)
% computes neural features
% phase in delta band + pwr in all available bands
% sets features on bad channels to 0
%
% Neuro
%   .DeltaBuf - buffer of delta band filtered neural data [ samps x chans ]
%   .FilteredData - filtered data from last bin [ samps x chans x frqs ]
%   .NeuralFeatures - vector of features for decoding [ features*chans x 1 ]

% allocate memory
samps = Neuro.NumSamps;
neural_features = zeros(Neuro.NumFeatures,Neuro.NumChannels);

% first compute hilbert for low freq bands
H = hilbert(Neuro.FilterDataBuf);

% compute phase features
idx = [Neuro.FilterBank.phase_flag];
idx = idx(1:Neuro.NumBuffer);
ang = angle(H(:,:,idx)); % instantaneous angle
for i=1:Neuro.NumPhase,
    neural_features(i,:) = angle(sum(exp(1i*squeeze(ang(end-samps+1:end,:,i)))));
end

% compute pwr in low freq bands based on hilbert (only keep last bin)
hilb_pwr = abs(H); % [samples x channels x freqs]
pwr1 = squeeze(log10(mean(hilb_pwr(end-samps+1:end,:,:),1)))'; % avg in last bin
% [freqs x channels]

% compute average pwr for all remaining freq bands in last bin
pwr2 = squeeze(log10(mean(Neuro.FilteredData(:,:,Neuro.NumBuffer+1:end).^2, 1)))';
% [freqs x channels]

% combine feature vectors and remove singleton dimension
pwr = cat(1,pwr1,pwr2);
feature_idx = [Neuro.FilterBank.feature];
for i=(Neuro.NumPhase+1):Neuro.NumFeatures,
    idx = feature_idx == i;
    neural_features(i,:) = mean(pwr(idx,:),1);
end

% set bad channels to 0
neural_features(:,Neuro.BadChannels) = 0;

% put features in Neuro
Neuro.NeuralFeatures = reshape(neural_features',[],1);

end % CompNeuralFeatures

