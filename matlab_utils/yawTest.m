clearvars
close all
colocPoses = fopen('C:/Users/saihv/Desktop/yawTest/poses.txt');
i = 1;
x = [];
yaw = [];
while ~feof(colocPoses)
    currentLine = fgets(colocPoses);
    data = textscan(currentLine, '%d,%d8,%d8,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    if abs(data{17}) > 1
    x(i) = data{4};
    data{17}
    if abs(data{17}) > 120
        if data{17} < 0
            yaw(i) = (-1*(data{17}) - 180) * pi/180;
        elseif data{17} > 0
            yaw (i) = (180 - abs(data{17})) * pi/180;
        end
    else
        yaw(i) = data{17} * pi/180;
    end
        i = i+1;
    end

end

plot(yaw, x, 'rx')


    