%initialize data matrix
%numberOfValues*measurements*sampleParameters
magData = zeros(400,4,32);
for i = 1:32
     for j = 1:4
    if j ==1
        filename = sprintf('hxl%d.txt', i-1);
        % read hxl file in layer u
        fileID = fopen(filename,'r');
    end
    if j ==2
        filename = sprintf('hxh%d.txt', i-1); 
        fileID = fopen(filename,'r');
    end
    if j ==3
        filename = sprintf('hyl%d.txt', i-1);
        fileID = fopen(filename,'r');
    end
    if j ==4
        filename = sprintf('hyh%d.txt', i-1);
        fileID = fopen(filename,'r');
    end
        magData(:,j,i) = fscanf(fileID,'%d');   % save read data in a 3-D matrix
     end
end


