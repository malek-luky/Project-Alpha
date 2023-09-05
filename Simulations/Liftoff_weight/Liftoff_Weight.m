clear all



% Define the filename of the .mat file
mat_filename = 'measured_data.mat';

% Load the data from the .mat file
loaded_data = load(mat_filename);
thrust_N_data = loaded_data.measured_data;

time = thrust_N_data.Time;
thrust_weight = thrust_N_data.Data;
thrust_N = thrust_weight * 9.81 / 1000; % transfer from liftoff grams to N

% Define the filename for the CSV file
csv_filename = 'my_data.csv';
data_to_save = [time, thrust_N];

% Save the data to a CSV file
writematrix(data_to_save,csv_filename);

% Intergation
weight=1:1:1000; %1g -> 1kg
integral = zeros(size(weight));
for i = 1:length(weight)
    index_liftoff_temp = find(thrust_weight>weight(i), 1, 'first' );
    acceleration = thrust_N - weight(i)/1000*9.81;
    acceleration(1:index_liftoff_temp) = 0;
    speed = cumtrapz(time, acceleration);
    altitude = cumtrapz(time, speed);
    integral(i) = altitude(end);      
end

%find best weight
best_weight = find(abs(integral)==min(abs(integral)),1,'last')
best_weight = 0.337 %real weight
index_liftoff = find(thrust_weight>best_weight, 1, "first");
acceleration = thrust_N - best_weight/1000*9.81;
acceleration(1:index_liftoff) = 0;
speed = cumtrapz(time, acceleration);
altitude = cumtrapz(time, speed);


% Plot it - AVERAGE
plot(time,altitude,"LineWidth",6);
set(gca,'FontSize',20)
xlabel("Time [s]");
ylabel("Altitude [m]");