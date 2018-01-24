clearvars
close all
colocPoses = fopen('C:/Users/saihv/Desktop/angleTest/poses.txt');
i = 1;
z = [];
pitch = [];
while ~feof(colocPoses)
    currentLine = fgets(colocPoses);
    data = textscan(currentLine, '%d,%d8,%d8,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    if abs(data{18}) > 1
    z(i) = (data{5} - 6)/5.0;
    data{18}
    if abs(data{18}) > 120
        if data{18} < 0
            pitch(i) = (180 + data{18})*pi/180;
        elseif data{18} > 0
            pitch (i) = (data{18} - 180)*pi/180;
        end
    else
        pitch(i) = data{18}*pi/180;
    end
        i = i+1;
    end

end

plot(pitch, z, 'rx')


    