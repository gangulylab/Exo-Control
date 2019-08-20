function F = FitDimRed(DataDir, DimRed)
% F = FitDimRed(DataDir, Neuro)
% Use dimensionality reduction teckniques to find low-dim or latent space
% for neural features.
% 
% DataDir - directory of neural data to fit dim red params
% DimRed - structure
%   .Method - 1-pca, 2-fa
%   .AvgTrialsFlag - 0-cat imagined mvmts, 1-avg imagined mvmts
%   .NumDims - number of dimensions to reduce to (default=[], lets user
%       define NumDims through interactive plot)
% F - returned mapping from full space to lowdim space (ie, X' = F*X),
%   where X is the full neural feature space [ features x samples ]

% ouput to screen
fprintf('\n\nFitting Dimensionality Reduction Parameters:\n')
switch DimRed.Method,
    case 1,
        fprintf('  Principal Component Analysis\n')
    case 2,
        fprintf('  Factor Analysis\n')
end
fprintf('  Data in %s\n', DataDir)
switch DimRed.AvgTrialsFlag,
    case false,
        fprintf('  Concatenating Trials\n\n')
    case true,
        fprintf('  Averaging Trials\n\n')
end

% user select data (override)
if DimRed.InitMode==2,
    [filenames,DataDir] = uigetfile('*.mat','Select the INPUT DATA FILE(s)','MultiSelect','on');
    for i=1:length(filenames),
        datafiles(i) = dir(fullfile(DataDir,filenames{i}));
    end
else,
    % load all data & organize according to DimRed.
    datafiles = dir(fullfile(DataDir,'Data*.mat'));
end

X = [];
for i=1:length(datafiles),
    load(fullfile(DataDir,datafiles(i).name)) %#ok<LOAD>
    switch DimRed.AvgTrialsFlag,
        case false, % concatenate trials
            X = cat(2,X,TrialData.NeuralFeatures{:});
            econstr = 'on';
        case true, % going to avg trials, cat in 3rd dim for now
            Xtrial = cat(2,TrialData.NeuralFeatures{:});
            if size(Xtrial,2)==80, % ignore trials w/ weird sizes
                X = cat(3,X,Xtrial);
                econstr = 'off';
            end
    end
end
if DimRed.AvgTrialsFlag,
    X = mean(X,3);
end

% use interactive PCA plot if num dims is not given
if isempty(DimRed.NumDims),
    % PCA
    [C,~,~,~,per_var_exp,mu] = pca(X','Economy',econstr);
    
    % get user input about # PCS
    fig = figure; hold on
    title('press key to exit')
    plot(cumsum(per_var_exp))
    plot([0;size(X,1)],[80 90 95;80 90 95],'k--')
    keydwn = waitforbuttonpress;
    while keydwn==0,
        keydwn = waitforbuttonpress;
    end
    close(fig)
    % user input
    NumDims = [];
    while isempty(NumDims),
        switch DimRed.Method,
            case 1, % PCA
                NumDims = input('# of PCs to use: ');
            case 2, % FA
                NumDims = input('# of Factors to use: ');
        end
    end
else,
    NumDims = DimRed.NumDims;
end

% do dimensionality reduction
switch DimRed.Method,
    case 1, % PCA
        if ~isempty(DimRed.NumDims),
            [C,~,~,~,~,mu] = pca(X','Economy',econstr);
        end
        C = C(:,1:NumDims);
        % return function handle to do PCA on new data
        F = @(X) ((X' - mu)*C)';
        
    case 2, % FA
        [estParams, ~] = myfastfa(X, NumDims);
        F = @(X) estParams.L\(X-estParams.d);
end

end % FitDimRed