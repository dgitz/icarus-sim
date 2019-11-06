function [small_signal_list] = subset_signals (signals,compare_string)
  small_signal_list = [];
  for s = 1:length(signals)
    if(isempty(strfind(signals{s}(1).name,compare_string)) == 0)
      small_signal_list{length(small_signal_list)+1} = signals{s};
    end
  end
end