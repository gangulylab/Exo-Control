function PlotERPs(datadir)
% function PlotERPs(datadir)
% loads all trials in datadir and plots each feature as a heatmap in a
% separate figure. saves a .png
% 
% datadir - directory of data (also saves fig there)

fprintf('\n\nMaking ERP plots...')

if isempty(datadir),
    datadir = uigetdir();
end

% grab data trial data
datafiles = dir(fullfile(datadir,'Data*.mat'));
Y = [];
TargetID = [];
for i=1:length(datafiles),
    % load data, grab neural features
    load(fullfile(datadir,datafiles(i).name)) %#ok<LOAD>
    Ytrial = cat(2,TrialData.NeuralFeatures{:});
    %Ytrial = Ytrial(:,1:2:end); % tmp
    %Ytrial = Ytrial(:,11:end); % tmp
    Y = cat(3,Y,Ytrial);
    TargetID = cat(1,TargetID,TrialData.TargetID);
end

% channel layout
load('ECOG_Grid_8596-002135.mat');
[R,C] = size(ecog_grid);
Nch = 128; % channels
Nft = size(Y,1)/Nch; % neural features
Ntm = size(Y,2); % time pts per trial
limch = ecog_grid(R,1);
legch = ecog_grid(R,round(C/2));

% all targets
TargetIDList = unique(TargetID);
leg = cell(1,length(TargetIDList));

% go through each feature and plot erps
feature_list = 1:Nft;
feature_list_str = {'delta phase','delta pwr','beta pwr','high gamma pwr'};
for i=feature_list,
    fig = figure('units','normalized','position',[.1,.1,.8,.8],...
        'name',feature_list_str{i},'numbertitle','off');
    ax = tight_subplot(R,C,[.01,.01],[.05,.01],[.03,.01]);
    set(ax,'NextPlot','add')
    
    % each reach target
    for Tidx=1:length(TargetIDList),
        % avg over trials going to same target
        T = TargetIDList(Tidx);
        leg{Tidx} = sprintf('T: %i',T);
        trial_idx = (T==TargetID);
        Ytarg = squeeze(mean(Y(:,:,trial_idx),3));


        for ch=1:Nch,
            [r,c] = find(ecog_grid == ch);
            idx = C*(r-1) + c;

            % plot
            erp = squeeze(Ytarg((ch-1)*Nft+i,:));
            plot(ax(idx),erp,'linewidth',1)

        end

    end % reach target
    
    % clean up
    XX = [1,Ntm];
    YY = cell2mat(get(ax,'YLim'));
    YY = [min(YY(:,1)),max(YY(:,2))];
    set(ax,'XLim',XX,'YLim',YY,'XTick',[],'YTick',[]);
    
    % add channel nums
    for ch=1:Nch,
        [r,c] = find(ecog_grid == ch);
        idx = C*(r-1) + c;
        text(ax(idx),XX(1),YY(1),sprintf('ch%03i',ch),...
            'VerticalAlignment','Bottom')
    end
    
    % add limits to limch
    [r,c] = find(ecog_grid == limch);
    idx = C*(r-1) + c;
    set(ax(idx),'XTick',XX,'XTickLabel',XX,'YTick',YY,'YTickLabel',YY)
    
    % add legend to legch
    [r,c] = find(ecog_grid == legch);
    idx = C*(r-1) + c;
    lgd = legend(ax(idx),leg,...
        'orientation','horizontal',...
        'position',[0.35,0.01,0.25,0.025]);
    
    % linkaxes (for manual adjustment of axes before saving)
    linkaxes(ax,'xy')
    waitforbuttonpress;
    
    % save plot
%     saveas(fig,fullfile(datadir,sprintf('ERPs_Feature%i',i)),'png')
end

fprintf('Done.\n\n')

end % PlotERPs
