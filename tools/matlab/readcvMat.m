%infile = 'testcvMat.txt';
infile = 'test_cvMat_Data.txt';
fid = fopen(infile,'r');

num_expr = '([\d|\.]+)'; %matches decimal numbers
startcvMat = '^\[';
endcvMat = '\]$';

Tall = {};
k = 0;
while(~feof(fid))
    line = fgetl(fid);
    
    res = regexp(line, startcvMat);  %check if this is the first line of a cv::Mat
    if(~isempty(res))
        k = k+1;
        T = [];
        i = 0;
        while true
            i = i+1;
            tokens = regexp(line, num_expr,'tokens');  %split entries 
            
            j = 0;
            for tok = tokens
               j = j+1;
               T(i,j) =  str2double(tok{1});  %append entries to matrix
            end
            
            res2 = regexp(line, endcvMat);  %check if this was the last line of the cv::Mat
            if(~isempty(res2))
                Tall{k} = T;
                break;  %done with this cv::Mat, break and look for next
            end
            line = fgetl(fid);  % get next line of cv::Mat
        end
    end
end

    