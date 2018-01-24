clearvars
close all
colocPoses = fopen('C:/Users/saihv/Desktop/3traj/poses.txt');
gtPoses = fopen('C:/Users/saihv/Desktop/3traj/airsim_rec2018-01-22-12-55-23.txt');

figure(1);
hold on;

filterData(1) = kalmanFilter();
filterData(2) = kalmanFilter();
filterData(3) = kalmanFilter();

initialize(filterData(1), [0 0 0]', 0.06, 1e-4, 1e-4, 1e-4);
initialize(filterData(2), [-5 0 0]', 0.06, 1e-4, 1e-4, 1e-4);
initialize(filterData(3), [5 0 0]', 0.06, 1e-4, 1e-4, 1e-4);

colocRaw = {};

i = 1;
while ~feof(gtPoses)
    currentLine = fgets(gtPoses);
    data = textscan(currentLine, '%d8 %s %f %f %f %f %f %f %f');
    
    x = data{3};
    y = data{5};
    z = data{4};
    %groundTruth(i,:) = [x y z];
    plot3(x, y, z, 'go');
    
    groundTruth{data{1}+1}(i,:) = [x y z];
    if data{1} == 2
        i = i+1;
    end
end
hold on

i = 1;
while ~feof(colocPoses)
    currentLine = fgets(colocPoses);
    data = textscan(currentLine, '%d,%d8,%d8,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    
    
    if data{1} > i
        colocFiltered{1}(i,:) = [filterData(1).xPosterior(1), filterData(1).xPosterior(2), filterData(1).xPosterior(3)];
        colocFiltered{2}(i,:) = [filterData(2).xPosterior(1), filterData(2).xPosterior(2), filterData(2).xPosterior(3)];
        colocFiltered{3}(i,:) = [filterData(3).xPosterior(1), filterData(3).xPosterior(2), filterData(3).xPosterior(3)];
        i
        i = i+1;
        data{1}
    end 
    
    imageIdx = data{1};
    sourceID = data{2};
    destID = data{3};    
    
    x = data{4};
    y = data{5};
    z = data{6};    
    
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

    if sourceID == destID
        y = y - 5;
        z = -1*z;
        colocRaw{sourceID+1}(i,:) = [x y z];
        %fprintf('Intra pose estimation for drone %d \n', sourceID);
        %plot3(x, y, z, 'rx');
        %coloc(sourceID,:,:) = [x y z];
        
        predict(filterData(sourceID+1));
        updateSelf(filterData(sourceID+1), [x y z]', 0.5*cov);
    else
        
        %fprintf('Inter pose estimation between drone %d and drone %d \n', sourceID, destID);
        %[xrel, Prel] = propagateRelativeCovariance(filterData(sourceID+1), [x y z]', 0.01*cov);
        %fuseRelative(filterData(destID+1), xrel, Prel); 
        
    end
    
    plot3(filterData(1).xPosterior(1), filterData(1).xPosterior(2), filterData(1).xPosterior(3), 'rx')
    plot3(filterData(2).xPosterior(1), filterData(2).xPosterior(2), filterData(2).xPosterior(3), 'rx')
    plot3(filterData(3).xPosterior(1), filterData(3).xPosterior(2), filterData(3).xPosterior(3), 'rx')
end

colocFiltered{1}(i,:) = [filterData(1).xPosterior(1), filterData(1).xPosterior(2), filterData(1).xPosterior(3)];
colocFiltered{2}(i,:) = [filterData(2).xPosterior(1), filterData(2).xPosterior(2), filterData(2).xPosterior(3)];
colocFiltered{3}(i,:) = [filterData(3).xPosterior(1), filterData(3).xPosterior(2), filterData(3).xPosterior(3)];


