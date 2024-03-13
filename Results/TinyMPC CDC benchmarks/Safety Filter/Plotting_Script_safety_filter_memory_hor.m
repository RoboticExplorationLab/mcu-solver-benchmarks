OSQP_S_P_1 = readtable('Memory Footprint Safety Filter OSQP.xlsx', "Sheet", "Problem_1_State");
OSQP_H_P_1 = readtable('Memory Footprint Safety Filter OSQP.xlsx', "Sheet", "Problem_1_Hor");
OSQP_S_P_2 = readtable('Memory Footprint Safety Filter OSQP.xlsx', "Sheet", "Problem_2_State");
OSQP_H_P_2 = readtable('Memory Footprint Safety Filter OSQP.xlsx', "Sheet", "Problem_2_Hor");
OSQP_S_P_3 = readtable('Memory Footprint Safety Filter OSQP.xlsx', "Sheet", "Problem_3_State");
OSQP_H_P_3 = readtable('Memory Footprint Safety Filter OSQP.xlsx', "Sheet", "Problem_3_Hor");
TinyMPC_S_P_1 = readtable('Memory Footprint Safety Filter TinyMPC.xlsx', "Sheet", "Problem_1_State");
TinyMPC_H_P_1 = readtable('Memory Footprint Safety Filter TinyMPC.xlsx', "Sheet", "Problem_1_Horizon");
TinyMPC_S_P_2 = readtable('Memory Footprint Safety Filter TinyMPC.xlsx', "Sheet", "Problem_2_State");
TinyMPC_H_P_2 = readtable('Memory Footprint Safety Filter TinyMPC.xlsx', "Sheet", "Problem_2_Horizon");
TinyMPC_S_P_3 = readtable('Memory Footprint Safety Filter TinyMPC.xlsx', "Sheet", "Problem_3_State");
TinyMPC_H_P_3 = readtable('Memory Footprint Safety Filter TinyMPC.xlsx', "Sheet", "Problem_3_Horizon");

osqp_mem_P_1 = table2array(OSQP_H_P_1);
osqp_mem_P_2 = table2array(OSQP_H_P_2);
osqp_mem_P_3 = table2array(OSQP_H_P_3);


size_P_1 = osqp_mem_P_1(3, :);
size_P_2 = osqp_mem_P_2(3, :);
size_P_3 = osqp_mem_P_3(3, :);

size_problems_all = [size_P_1; size_P_2; size_P_3];

avg_mem_OSQP = mean(size_problems_all)/1024

tinympc_mem_P_1 = table2array(TinyMPC_H_P_1);
tinympc_mem_P_2 = table2array(TinyMPC_H_P_2);
tinympc_mem_P_3 = table2array(TinyMPC_H_P_3);


size_P_1 = tinympc_mem_P_1(3, :);
size_P_2 = tinympc_mem_P_2(3, :);
size_P_3 = tinympc_mem_P_3(3, :);

size_problems_all = [size_P_1; size_P_2; size_P_3];

avg_mem_tinympc = mean(size_problems_all)/1024
%{
second_row_array1 = osqp_mem_P_1(4, :);
second_row_array2 = osqp_mem_P_2(4, :);
second_row_array3 = osqp_mem_P_3(4, :);

second_rows_matrix = [second_row_array1; second_row_array2; second_row_array3];

average_across_columns = mean(second_rows_matrix)
%}
figure('Position', [100, 100, 800, 600]);
xs_tinympc = [4,10,16,22,30,40,50];
xs_tinympc_index = 1:numel(xs_tinympc);
xs_osqp = [4,10,16,22,30];
xs_osqp_index = 1:numel(xs_osqp);
line1 = plot(xs_tinympc_index,avg_mem_tinympc,'Marker','.','MarkerSize',25,'color', [0, 0, 1], 'LineWidth', 2)
hold on 
for i = 1:numel(xs_tinympc_index)
    plot([xs_tinympc_index(i)-0.2, xs_tinympc_index(i)+0.2], [1, 1], 'w', 'LineWidth', 1.5);
    hold on 
end
xticks(xs_tinympc_index);
yticks([0, 40, 80, 120, 160]); % Specify custom y-axis tick values
xticklabels({'4', '10', '16', '22', '30', '40', '50'});
xlabel('Horizon (N)', 'FontSize', 24);
ylabel('Memory Usage (kB)', 'FontSize', 24);
hold on

line2 = plot(xs_osqp_index,avg_mem_osqp,'Marker','.','MarkerSize',25,'color', [1, 0, 0], 'LineWidth', 2)
hold on
yline(128, '--', 'Label', 'RAM Threshold [128 kB]','FontSize', 18, 'LineWidth', 2);
% Increase x-axis tick label font size
set(gca, 'FontSize', 20);

% Increase y-axis tick label font size
set(gca, 'FontSize', 20);
legend([line1,line2],'TinyMPC', 'OSQP');
grid on;
matlab2tikz('Safety_filter_mem_hor.tikz');
