function res = irscan()
% function res = irscan(data, Calibrate)

clf;
hold on;
lower_angle = 80;
upper_angle = 150;
difference_angle = upper_angle-lower_angle;

%% Plotting Line of Best Fit

Calibrate = [[570.4310, 13.5];[500.8350, 17.5];[467.7893, 24];[429.6631, 27.5];[220.2796, 61]]; % Calibrate(:,1) == IR Reading, Calibrate(:,2) == distance (cm)

coeffs = polyfit(Calibrate(:,2), Calibrate(:,1), 1);
% Get fitted values
fittedX = linspace(min(Calibrate(:,2)), max(Calibrate(:,2)), 200);
fittedY = polyval(coeffs, fittedX);
% Plot the fitted line
% hold on;
plot(fittedX, fittedY, 'r-', 'LineWidth', 3);

plot(Calibrate(:,2),Calibrate(:,1));
xlabel('Distance (cm)');
ylabel('IR Sensor Reading');

%% Prepping Data for Averaging Function

data = [[93,95];[94,101];[95,364];[96,472];[97,492];[98,496];[99,508];[100,479];[101,490];[102,497];[103,493];[104,512];[105,493];[106,494];[107,494];[108,493];[109,491];[110,489];[111,490];[112,490];[113,490];[114,495];[115,486];[116,485];[117,487];[118,482];[119,487];[120,482];[121,512];[122,479];[123,479];[124,479];[125,479];[126,483];[127,471];[128,474];[129,419];[130,423];[131,435];[132,450];[133,474];[134,442];[135,383];[136,159];[137,3];[138,14];[139,3];[140,3];[141,35];[142,59];[143,47];[144,51];[145,59];[146,64];[147,60];[148,51];[149,66];[150,55];[149,52];[148,55];[147,82];[146,56];[145,59];[144,59];[143,59];[142,56];[141,60];[140,51];[139,39];[138,64];[137,55];[136,3];[135,27];[134,3];[133,24];[132,380];[131,393];[130,418];[129,446];[128,446];[127,439];[126,420];[125,439];[124,435];[123,468];[122,476];[121,479];[120,479];[119,479];[118,482];[117,483];[116,486];[115,483];[114,482];[113,620];[112,486];[111,603];[110,490];[109,489];[108,494];[107,490];[106,504];[105,493];[104,494];[103,494];[102,494];[101,522];[100,494];[99,493];[98,493];[97,479];[96,628];[95,521];[94,633];[93,515];[92,503];[91,505];[90,501];[89,500];[88,500];[87,500];[86,500];[85,530];[84,497];[83,497];[82,495];[81,480];[80,470];[79,558];[78,461];[77,493];[76,472];[75,479];[74,524];[73,528];[72,532];[71,510];[70,503];[71,511];[72,515];[73,511];[74,533];[75,533];[76,529];[77,523];[78,508];[79,475];[80,475];[81,468];[82,448];[83,461];[84,475];[85,520];[86,493];[87,497];[88,502];[89,500];[90,500];[91,505];[92,504];[93,531];[94,504];[95,505];[96,511];[97,510];[98,518];[99,599];[100,475];[101,494];[102,493];[103,493];[104,494];[105,493];[106,501];[107,494];[108,493];[109,489];[110,490];[111,492];[112,490];[113,519];[114,491];[115,490];[116,598];[117,484];[118,491];[119,482];[120,483];[121,482];[122,478];[123,484];[124,478];[125,506];[126,475];[127,465];[128,450];[129,435];[130,420];[131,523];[132,440];[133,547];[134,416];[135,384];[136,266];[137,11];[138,3];[139,2];[140,27];[141,55];[142,73];[143,56];[144,59];[145,42];[146,52];[147,56];[148,68];[149,60];[150,69];[149,56];[148,55];[147,48];[146,59];[145,47];[144,77];[143,59];[142,51];[141,51];[140,47];[139,44];[138,66];[137,52];[136,27];[135,3];[134,3];[133,103];[132,324];[131,505];[130,404];[129,439];[128,443];[127,426];[126,451];[125,443];[124,464];[123,472];[122,475];[121,509];[120,479];[119,477];[118,479];[117,482];[116,602];[115,483];[114,504];[113,486];[112,487];[111,490];[110,489];[109,490];[108,489];[107,489];[106,490];[105,493];[104,497];[103,494];[102,514];[101,493];[100,494];[99,494];[98,489];[97,508];[96,483];[95,522];[94,548];[93,508];[92,504];[91,506];[90,503];[89,504];[88,504];[87,504];[86,502];[85,501];[84,496];[83,494];[82,490];[81,479];[80,472];[79,449];[78,452];[77,471];[76,482];[75,515];[74,503];[73,533];[72,529];[71,511];[70,509];[71,519];[72,518];[73,519];[74,511];[75,528];[76,536];[77,518];[78,497];[79,480];[80,472];[81,455];[82,450];[83,480];[84,515];[85,490];[86,497];[87,500];[88,501];[89,500];[90,504];[91,504];[92,527];[93,505];[94,504];[95,509];[96,539];[97,515];[98,526];[99,483];[100,478];[101,493];[102,494];[103,495];[104,493];[105,522];[106,493];[107,493];[108,493];[109,491];[110,493];[111,602];[112,491];[113,489];[114,490];[115,491];[116,486];[117,515];[118,482];[119,482];[120,487];[121,482];[122,485];[123,478];[124,496];[125,479];[126,475];[127,476];[128,560];[129,450];[130,423];[131,436];[132,450];[133,445];[134,442];[135,386];[136,206];[137,3];[138,3];[139,23];[140,4];[141,57];[142,63];[143,60];[144,48];[145,65];[146,60];[147,119];[148,55];[149,59];[150,57];[149,47];[148,48];[147,65];[146,60];[145,87];[144,65];[143,59];[142,71];[141,65];[140,39];[139,52];[138,48];[137,71];[136,30];[135,2];[134,9];[133,197];[132,347];[131,386];[130,436];[129,475];[128,441];[127,439];[126,423];[125,423];[124,458];[123,472];[122,501];[121,558];[120,480];[119,595];[118,482];[117,483];[116,482];[115,481];[114,487];[113,486];[112,488];[111,486];[110,513];[109,489];[108,490];[107,491];[106,490];[105,497];[104,494];[103,493];[102,492];[101,494]];
% data(:,1) == Angles (degrees), data(:,2) == IR Reading
% scan_degrees = [50:170];
scan_degrees = [lower_angle:upper_angle];
sum = 0;
average_scan = zeros(difference_angle,2);


%% Averaging Function

for i = 1:difference_angle % For a degree in the range 50-170 degrees...
    counter = 1; % Reset counter
    sum = 0; % Reset sum of IR Readings
    for j = 1:length(data) % Checking for repeated angles in data matrix X
        if (data(j,1) == scan_degrees(i)) % If the angle is identical to current angle x
            sum = sum + data(j,2); % Add it's IR reading to the total sum of IR readings.
            counter = counter + 1; % Count the number of times the angle was identical
        end
    end
    average_scan(i,1) = scan_degrees(i); % The degrees, from 80-150 degrees
    average_scan(i,2) = sum / counter; % Divide the sum of all the distance numbers for an angle by the # of times that angle appears
end

figure
% plot(data(:,1), data(:,2))
plot(average_scan(:,1),average_scan(:,2));
xlabel('Angle (degrees)');
ylabel('IR Sensor Reading');

%% Converting IR Reading into Distance
IRtoCM = [];

for num = 1:length(fittedX)
    IRtoCM(num,1) = fittedX(num); % The distance in cm
    IRtoCM(num,2) = round(fittedY(num)); % The distance in IR readings
end

% for num = 2:length(fittedX)
%     if (IRtoCM(num,2) == IRtoCM(num-1,2))
%         print 'Two distances have the same IR reading. Please inspect rounded calibration data.'
%     end
% end

distance = [];

for i = 1:length(average_scan(:,2)) % For each IR reading in the data
    for j = 1:(length(IRtoCM(:,2))-1) % Iterate through the calibrated line of best fit.
        
        %Event!
        if (IRtoCM(j+1,2) >= average_scan(i,2)) %If the next IR data point in the calibration line is greater than the current IR reading...
            diffupper = IRtoCM(j+1,2) - average_scan(i,2);
            difflower = average_scan(i,2) - IRtoCM(j,2);
            if (diffupper <= difflower)  %Check whether the current data point is closer to the next calibration data point, rather than the previous calibration data point.
                distance(i) = IRtoCM(j+1,1); %Set the distance for point average_scan(i,2)'s IR reading to the distance in cm that the calibration curve predicts for the next point.
            else
                distance(i) = IRtoCM(j,1); %Set the distance for point average_scan(i,2)'s IR reading to the distance in cm that the calibration curve predicts for the previous point.
            end
        end
        
    end
%     distance(i)
end

distance = transpose(distance);

figure
plot(average_scan(:,1),distance);
xlabel('Angle (degrees)');
ylabel('Distance (cm)');

%% Converting distance into X and Y coordinates

height = sind((average_scan(:,1)-100)) .* distance;

abs_distance = height ./ tand((average_scan(:,1)-100));
for check = 1:length(abs_distance)
    if (abs_distance(check) == NaN)
        abs_distance(check) = 0;
    end
end

smoothed_distance = three_point_moving_average(abs_distance);
smoothed_distance1 = three_point_moving_average(smoothed_distance);
smoothed_distance2 = three_point_moving_average(smoothed_distance1);

figure
plot(smoothed_distance2, height);
xlabel('Height (cm)');
ylabel('Absolute Distance (cm)');

%% 3PT AVERAGE FUNCTION

function y = three_point_moving_average(x)

% make a vector y to store the result of the moving average
y = zeros(size(x)); 

% compute the first sample, assuming the previous sample is zero
y(1) = 1/3*(x(1) + x(2));

% compute the 3 pt. moving average of the rest of the samples 
for n = 2:length(x)-1
    y(n) = 1/3*(x(n-1)+x(n)+x(n+1));
end

% Compute the last sample outside the for loop to because there wont be an
% x(n+1), lets assume it is zero here

y(length(x)) = 1/3*(x(length(x)-1) + x(length(x)));
end

end