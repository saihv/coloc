colocPoses = fopen('D:/test/colocTests/3trajNew2/poses.txt');
gtPoses = fopen('D:/test/colocTests/3trajNew2/airsim_rec2018-01-29-13-57-31.txt');

figure(1);
hold on;
groundTruth = [];
coloc = [];

i = 1;
while ~feof(gtPoses)
    currentLine = fgets(gtPoses);
    data = textscan(currentLine, '%d8 %s %f %f %f %f %f %f %f');
    
    x = data{3};
    y = data{4};
    z = -1 * data{5};
    groundTruth(i,:) = [x y z];
    i = i + 1;
    plot3(x, y, z, 'go');
end
hold on

i = 1;
while ~feof(colocPoses)
    currentLine = fgets(colocPoses);
    data = textscan(currentLine, '%d8, %d8, %f, %f, %f, %f, %f, %f');
    sourceID = data{1};
    destID = data{2};
    
    x = data{3};
    y = -1*data{5};
    z = data{4} + 5;
    

    if sourceID == destID
        fprintf('Intra pose estimation for drone %d \n', sourceID);
        %plot3(x, y, z, 'rx');
        coloc(sourceID,:,:) = [x y z];
    else
        fprintf('Inter pose estimation between drone %d and drone %d \n', sourceID, destID);
    end
end

immse(coloc, groundTruth)