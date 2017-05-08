function res = temp()

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
%% Stuff
x = csvread('three_d_scan3.csv');
l = x(:,2)
b = coeffs(1);
c = coeffs(2);
length(l);

for i = 1:length(l)
g(i,1) = c + b * l(i);
end
w = g

x(:,2) = g(:,1)
e = length(x)
q = length(g)

d_c = 24.85046596; %cm from IR sensor to center of rotation
phi = [0:2*pi/58.:2*pi];

h = x;

for j = 1:(length(x)/7-1)
    h(j,2)= mean(x((7*(j-1)+1):(7*j+7), 2));
    h(j,1)= x(7*(j-1)+1,1); 
end

x = h;


point = zeros(length(x),2);



for i = 1:length(x)

    point(i,1) = x(i,2)*cos(((x(i,1))/180.)*pi);
    point(i,2) = x(i,2)*sin(((x(i,1))/180.)*pi);
end

d_phi = 2*pi/length(phi);



hold on
for k = 1:length(phi)
    d_phi = k/length(phi) * 2*pi;
    R = [[cos(d_phi), -sin(d_phi), 0]; [sin(d_phi), cos(d_phi), 0];[0,0,1]];

    X = point(:,1);
    Z = point(:,2);
    Y = zeros(length(point),1);
    
    three_d = [X,Y,Z]*R;
    plot3(three_d(:,1), three_d(:,2), three_d(:,3));

end


end
