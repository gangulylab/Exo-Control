function varargout = NeuroPipeline(Neuro,Data),
% function Neuro = NeuroPipeline(Neuro)
% function [Neuro,Data] = NeuroPipeline(Neuro,Data)
% Neuro processing pipeline. To change processing, edit this function.

% process neural data
Neuro = ReadBR(Neuro);
Neuro = RefNeuralData(Neuro);
if Neuro.UpdateChStatsFlag,
    Neuro = UpdateChStats(Neuro);
end
if Neuro.ZscoreRawFlag,
    Neuro = ZscoreChannels(Neuro);
end
Neuro = ApplyFilterBank(Neuro);
Neuro = UpdateNeuroBuf(Neuro);
Neuro = CompNeuralFeatures(Neuro);
if Neuro.UpdateFeatureStatsFlag,
    Neuro = UpdateFeatureStats(Neuro);
end
if Neuro.ZscoreFeaturesFlag,
    Neuro = ZscoreFeatures(Neuro);
end
if Neuro.DimRed.Flag,
    Neuro.NeuralFactors = Neuro.DimRed.F(Neuro.NeuralFeatures);
end
varargout{1} = Neuro;

% if Data exists and is not empty, fill structure
if exist('Data','var') && ~isempty(Data),
    Data.NeuralTimeBR(1,end+1) = Neuro.TimeStamp;
    Data.NeuralSamps(1,end+1) = Neuro.NumSamps;
    Data.NeuralFeatures{end+1} = Neuro.NeuralFeatures;
    if Neuro.SaveRaw,
        Data.BroadbandData{end+1} = Neuro.BroadbandData;
    end
    if Neuro.DimRed.Flag,
        Data.NeuralFactors{end+1} = Neuro.NeuralFactors;
    end
    if Neuro.SaveProcessed,
        Data.ProcessedData{end+1} = Neuro.FilteredData;
    end
    varargout{2} = Data;
end

end % NeuroPipeline