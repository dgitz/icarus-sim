function [result] = check_model(model_name)
max_blockcount = 900;
block_definition{1}.name = 'TimeCompensate';
block_definition{1}.max_count = 50;
block_definition{2}.name = 'SensorPostProcessing';
block_definition{2}.max_count = 50;
block_definition{3}.name = 'SignalLinker';
block_definition{3}.max_count = 100;
result = 0;
if((strcmp(model_name,'Pose_AutoCode')) || (strcmp(model_name,'PoseExecutor')))
else
    disp(['[ERROR]: Model: ' model_name ' Is Not Supported.']);
    return;
end
if(bdIsLoaded(model_name) == 0)
    open_system(model_name)
end
if(bdIsLoaded(model_name) == 0)
    disp(['[ERROR]: Unable to open Model: ' model_name ]);
    return;
end
system_listing = find_system(model_name,'LookUnderMasks', 'on', 'FollowLinks', 'on','Virtual','off');
top_level_blocks = [];
for i = 1:length(system_listing)
    item = char(system_listing(i));    
    if((count(item,'/') == 2)) % Top Level
        top_level_blocks{length(top_level_blocks)+1} = item;
    end
end
all_checks_ok = 1;
for i = 1:length(top_level_blocks)
    found = 0;
    for j = 1:length(block_definition)

         match_str = [model_name  '/PoseModel/'  block_definition{j}.name];
        
        if(strcmp(top_level_blocks{i},match_str) == 1)
            found = 1;
            block_count = length(find_system(top_level_blocks{i}, 'LookUnderMasks', 'on', 'FollowLinks', 'on','Virtual','off'));
            if(block_count > block_definition{j}.max_count)
                all_checks_ok = 0;
                disp(['[ERROR]: Model: ' model_name ' Block: ' block_definition{j}.name ... 
                    ' Exceeds Block Count Threshold: ' num2str(block_count) ' > ' num2str(block_definition{1}.max_count) '.']);
            else
                disp(['[INFO]: Model: ' model_name ' Block: ' block_definition{j}.name ...
                    ' Usage Level: ' num2str(100.0 * block_count/block_definition{j}.max_count) ' %' ...
                    ' (' num2str(block_count) ' used of ' num2str(block_definition{j}.max_count) ' available).']);
            end
        end
        
    end
end
model_blockcount = length(find_system(model_name, 'LookUnderMasks', 'on', 'FollowLinks', 'on','Virtual','off'));
if(model_blockcount > max_blockcount)
    disp(['[ERROR]: Model: ' model_name ' Block Count: ' num2str(model_blockcount) ' > ' num2str(max_blockcount) '.']);
    all_checks_ok = 0;
else
    disp(['[INFO]: Model: ' model_name ' Block Count Usage Level: ' num2str(100.0*model_blockcount/max_blockcount) ' %' ...
        ' (' num2str(model_blockcount) ' used of ' num2str(max_blockcount) ' available).']);
end
if(all_checks_ok == 1)
    result = 1;
end
if(result == 0)
    disp(['[ERROR]: Model: ' model_name ' violates build constraints.']);
end
if(all_checks_ok == 1)
    disp(['[INFO]: Model: ' model_name ' appears Valid!']);
end

end

