clear; close all; clc;

inf_time_0 = readtable('yolo_inference_time_0.csv');
inf_time_1 = readtable('yolo_inference_time_1.csv');
inf_time_2 = readtable('yolo_inference_time_2.csv');
inf_time_3 = readtable('yolo_inference_time_3.csv');
comp_time = readtable('mesh_timing_log.csv');
RMSE = readtable('mesh_estimation_error.csv');

figure;
subplot(1,4,1);
plot(inf_time_0.timestamp(2:end), inf_time_0.inference_time(2:end),'Color','r','LineWidth',1.2); hold on;
avg_0 = mean(inf_time_0.inference_time(2:end));
max_0 = max(inf_time_0.inference_time(2:end));
plot(inf_time_0.timestamp(2:end), avg_0*ones(size(inf_time_0.timestamp(2:end))), "Color",'b','LineStyle','--','LineWidth',1.0);
plot(inf_time_0.timestamp(2:end), max_0*ones(size(inf_time_0.timestamp(2:end))), "Color",'k','LineStyle','--','LineWidth',1.0);
grid on;
xlabel('Timestamp[s]')
ylabel('Inference time[ms]')
xlim([0, 40]);
legend('','avg','max');
title('view 1')

subplot(1,4,2);
plot(inf_time_1.timestamp(2:end), inf_time_1.inference_time(2:end),'Color','r','LineWidth',1.0); hold on;
avg_1 = mean(inf_time_1.inference_time(2:end));
max_1 = max(inf_time_1.inference_time(2:end));
plot(inf_time_0.timestamp(2:end), avg_1*ones(size(inf_time_0.timestamp(2:end))), "Color",'b','LineStyle','--','LineWidth',1.0);
plot(inf_time_0.timestamp(2:end), max_1*ones(size(inf_time_0.timestamp(2:end))), "Color",'k','LineStyle','--','LineWidth',1.0);
grid on;
xlabel('Timestamp[s]')
ylabel('Inference time[ms]')
xlim([0, 40]);
title('view 2')

subplot(1,4,3);
plot(inf_time_2.timestamp(2:end), inf_time_2.inference_time(2:end),'Color','r','LineWidth',1.0); hold on;
avg_2 = mean(inf_time_2.inference_time(2:end));
max_2 = max(inf_time_2.inference_time(2:end));
plot(inf_time_0.timestamp(2:end), avg_2*ones(size(inf_time_0.timestamp(2:end))), "Color",'b','LineStyle','--','LineWidth',1.0);
plot(inf_time_0.timestamp(2:end), max_2*ones(size(inf_time_0.timestamp(2:end))), "Color",'k','LineStyle','--','LineWidth',1.0);
grid on;
xlabel('Timestamp[s]')
ylabel('Inference time[ms]')
xlim([0, 40]);
title('view 3')

subplot(1,4,4);
plot(inf_time_3.timestamp(2:end), inf_time_3.inference_time(2:end),'Color','r','LineWidth',1.0); hold on;
avg_3 = mean(inf_time_3.inference_time(2:end));
max_3 = max(inf_time_3.inference_time(2:end));
plot(inf_time_0.timestamp(2:end), avg_3*ones(size(inf_time_0.timestamp(2:end))), "Color",'b','LineStyle','--','LineWidth',1.0);
plot(inf_time_0.timestamp(2:end), max_3*ones(size(inf_time_0.timestamp(2:end))), "Color",'k','LineStyle','--','LineWidth',1.0);
grid on;
xlabel('Timestamp[s]')
ylabel('Onference time[ms]')
xlim([0, 40]);
title('view 4')

avg = mean([avg_0, avg_1, avg_2, avg_3]);
sgstr=strcat('YOLO Inference Time (avg: ', num2str(avg),'ms)');
sgtitle(sgstr);


% figure;
% plot(inf_time_0.timestamp(2:end), inf_time_0.inference_time(2:end));
% 
% figure;
% plot(inf_time_1.timestamp(2:end), inf_time_1.inference_time(2:end));
% 
% figure;
% plot(inf_time_2.timestamp(2:end), inf_time_2.inference_time(2:end));
% 
% figure;
% plot(inf_time_3.timestamp(2:end), inf_time_3.inference_time(2:end));

figure;
corr_timestamp = [];
corr_time = [];
for i=1:size(comp_time.timestamp)
    if comp_time.correction_time_ms(i) ~= 0
        corr_timestamp = [corr_timestamp comp_time.timestamp(i)];
        corr_time = [corr_time comp_time.correction_time_ms(i)];
    end
end
plot(corr_timestamp, corr_time,'Color','b','LineWidth',1.2);
grid on;
xlabel('Timestamp[s]');
ylabel('Correction time[ms]');
xlim([0, 40]);
avg_corr_time = mean(corr_time);
titlestr = strcat('Correction Time (avg: ', num2str(avg_corr_time), 'ms)');
title(titlestr);


figure;
plot(comp_time.timestamp, comp_time.prediction_time_ms, 'Color','b','LineWidth',1.2);
grid on;
xlabel('Timestamp[s]');
ylabel('Prediction time[ms]');
xlim([0, 40]);
avg_pred_time = mean(comp_time.prediction_time_ms);
titlestr = strcat('Prediction Time (avg: ', num2str(avg_pred_time), 'ms)');
title(titlestr);

figure;
plot(RMSE.timestamp, RMSE.RMSE,'Color','g','LineWidth',1.2);
grid on;
xlabel('Timestamp[s]');
ylabel('RMSE[m]');
xlim([0, 40]);
titlestr = strcat('Tracking Error (RMSE)');
title(titlestr);