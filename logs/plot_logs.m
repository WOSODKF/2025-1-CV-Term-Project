clear; close all; clc;

inf_time_0 = readtable('yolo_inference_time_0.csv');
inf_time_1 = readtable('yolo_inference_time_1.csv');
inf_time_2 = readtable('yolo_inference_time_2.csv');
inf_time_3 = readtable('yolo_inference_time_3.csv');
comp_time = readtable('mesh_timing_log.csv');

figure;
plot(inf_time_0.timestamp(2:end), inf_time_0.inference_time(2:end));

figure;
plot(inf_time_1.timestamp(2:end), inf_time_1.inference_time(2:end));

figure;
plot(inf_time_2.timestamp(2:end), inf_time_2.inference_time(2:end));

figure;
plot(inf_time_3.timestamp(2:end), inf_time_3.inference_time(2:end));

figure;
corr_timestamp = [];
corr_time = [];
for i=1:size(comp_time.timestamp)
    if comp_time.correction_time_ms(i) ~= 0
        corr_timestamp = [corr_timestamp comp_time.timestamp(i)];
        corr_time = [corr_time comp_time.correction_time_ms(i)];
    end
end
plot(corr_timestamp, corr_time);

figure;
plot(comp_time.timestamp, comp_time.prediction_time_ms);