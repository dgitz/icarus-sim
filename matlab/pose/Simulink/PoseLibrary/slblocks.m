function blkStruct = slblocks
		% This function specifies that the library should appear
		% in the Library Browser
		% and be cached in the browser repository
        blkStruct.OpenFcn = {'helperlibrary','posemodel'};
        Browser(1).Library = 'helperlibrary'; 
        Browser(1).Name    = 'Pose Helper Library'; 
        Browser(1).IsFlat  = 1;% Is this library "flat" (i.e. no subsystems)?
        Browser(2).Library = 'posemodel'; 
        Browser(2).Name    = 'Pose Model Library'; 
        Browser(2).IsFlat  = 1;% Is this library "flat" (i.e. no subsystems)?
        Browser(3).Library = 'timecompensate'; 
        Browser(3).Name    = 'Pose Time Compensate Library'; 
        Browser(3).IsFlat  = 1;% Is this library "flat" (i.e. no subsystems)?
		% 'My Library' is the library name that appears 
             % in the Library Browser
		blkStruct.Browser = Browser; 
