clearvars
close all
colocPoses = fopen('C:/Users/saihv/Desktop/3traj/poses.txt');
gtPoses = fopen('C:/Users/saihv/Desktop/3traj/airsim_rec2018-01-22-12-55-23.txt');

figure(1);
hold on;

filterData = kalmanFilter();

initialize(filterData, [0 0 0]', 0.06, 1e-4, 1e-4, 1e-4);

i = 1;
while ~feof(gtPoses)
    currentLine = fgets(gtPoses);
    data = textscan(currentLine, '%d8 %s %f %f %f %f %f %f %f');
    
    if data{1} == 0
        x = data{3};
        y = data{4};
        z = data{5};
        %groundTruth(i,:) = [x y z];
        plot3(x, z, y, 'go');
    
        groundTruth(i,:) = [x z y];
        i = i + 1;
    end
end
hold on

i = 1;
while ~feof(colocPoses)
    currentLine = fgets(colocPoses);
    data = textscan(currentLine, '%d,%d8,%d8,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    
    imageIdx = data{1};
    sourceID = data{2};
    destID = data{3};    
    
    x = data{4};
    y = data{5};
    z = data{6} * -1;    
    
    cov = zeros(3,3);
    cov(1,1) = data{7};
    cov(1,2) = data{8};
    cov(1,3) = data{9};
    cov(2,1) = data{10};
    cov(2,2) = data{11};
    cov(2,3) = data{12};
    cov(3,1) = data{13};
    cov(3,2) = data{14};
    cov(3,3) = data{15};


    if destID == 0
        colocRaw(i,:) = [x y z];
        y = y-5;
        plot3(x, y, z, 'rx')

        %fprintf('Intra pose estimation for drone %d \n', sourceID);
        %plot3(x, y, z, 'rx');
        %coloc(sourceID,:,:) = [x y z];
        
        predict(filterData);
        cov
        updateSelf(filterData, [x y z]', 0.5*cov);
        
        colocFiltered(i,:) =  [filterData.xPosterior(1), filterData.xPosterior(2), filterData.xPosterior(3)];
        plot3(filterData.xPosterior(1), filterData.xPosterior(2), filterData.xPosterior(3), 'bx')
        i = i + 1;
    end
end


