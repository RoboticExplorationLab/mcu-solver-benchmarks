OSQP_S_P_1 = readtable('Safety Filter STM32 OSQP.xlsx', "Sheet", 1);
OSQP_H_P_1 = readtable('Safety Filter STM32 OSQP.xlsx', "Sheet", 2);
OSQP_S_P_2 = readtable('Safety Filter STM32 OSQP Problem 2.xlsx', "Sheet", 1);
OSQP_H_P_2 = readtable('Safety Filter STM32 OSQP Problem 2.xlsx', "Sheet", 2);
OSQP_S_P_3 = readtable('Safety Filter STM32 OSQP Problem 3.xlsx', "Sheet", 1);
OSQP_H_P_3 = readtable('Safety Filter STM32 OSQP Problem 3.xlsx', "Sheet", 2);
TinyMPC_S_P_1 = readtable('Safety Filter STM32 TinyMPC.xlsx', "Sheet", 1);
TinyMPC_H_P_1 = readtable('Safety Filter STM32 TinyMPC.xlsx', "Sheet", 2);
TinyMPC_S_P_2 = readtable('Safety Filter STM32 TinyMPC Problem 2.xlsx', "Sheet", 1);
TinyMPC_H_P_2 = readtable('Safety Filter STM32 TinyMPC Problem 2.xlsx', "Sheet", 2);
TinyMPC_S_P_3 = readtable('Safety Filter STM32 TinyMPC Problem 3.xlsx', "Sheet", 1);
TinyMPC_H_P_3 = readtable('Safety Filter STM32 TinyMPC Problem 3.xlsx', "Sheet", 2);

osqp_iterations_selectedIndices = [1, 4, 7, 13];
osqp_solvetimes_selectedIndices = [2, 5, 8, 14];

selectedColumns_osqp_iterations = OSQP_S_P_1(:, osqp_iterations_selectedIndices);
osqp_iterations_P_1 = table2array(selectedColumns_osqp_iterations);
selectedColumns_osqp_iterations = OSQP_S_P_2(:, osqp_iterations_selectedIndices);
osqp_iterations_P_2 = table2array(selectedColumns_osqp_iterations);
selectedColumns_osqp_iterations = OSQP_S_P_3(:, osqp_iterations_selectedIndices);
osqp_iterations_P_3 = table2array(selectedColumns_osqp_iterations);
selectedColumns_osqp_solvetimes = OSQP_S_P_1(:, osqp_solvetimes_selectedIndices);
osqp_solvetimes_P_1 = table2array(selectedColumns_osqp_solvetimes);
selectedColumns_osqp_solvetimes = OSQP_S_P_2(:, osqp_solvetimes_selectedIndices);
osqp_solvetimes_P_2 = table2array(selectedColumns_osqp_solvetimes);
selectedColumns_osqp_solvetimes = OSQP_S_P_3(:, osqp_solvetimes_selectedIndices);
osqp_solvetimes_P_3 = table2array(selectedColumns_osqp_solvetimes);

[numRows, numCols] = size(osqp_solvetimes_P_1); % Assuming both tables have the same size

resultTable = table(); % Initialize an empty table to store the results

for i = 1:numCols
    % Extract columns from table1 and table2
    col_table1_p1 = osqp_solvetimes_P_1(:, i);
    col_table2_p1 = osqp_iterations_P_1(:, i);
    
    % Perform element-wise division
    osqp_s_result_p1(:,i) = col_table1_p1 ./ col_table2_p1;
    col_table1_p2 = osqp_solvetimes_P_2(:, i);
    col_table2_p2 = osqp_iterations_P_2(:, i);
    
    % Perform element-wise division
    osqp_s_result_p2(:,i) = col_table1_p2 ./ col_table2_p2;
    col_table1_p3 = osqp_solvetimes_P_3(:, i);
    col_table2_p3 = osqp_iterations_P_3(:, i);
    
    % Perform element-wise division
    osqp_s_result_p3(:,i) = col_table1_p3 ./ col_table2_p3;

end

tinympc_iterations_selectedIndices = [1, 4, 7, 13, 25];
tinympc_solvetimes_selectedIndices = [2, 5, 8, 14, 26];

selectedColumns_tinympc_iterations = TinyMPC_S_P_1(:, tinympc_iterations_selectedIndices);
tinympc_iterations_P_1 = table2array(selectedColumns_tinympc_iterations);
selectedColumns_tinympc_iterations = TinyMPC_S_P_2(:, tinympc_iterations_selectedIndices);
tinympc_iterations_P_2 = table2array(selectedColumns_tinympc_iterations);
selectedColumns_tinympc_iterations = TinyMPC_S_P_3(:, tinympc_iterations_selectedIndices);
tinympc_iterations_P_3 = table2array(selectedColumns_tinympc_iterations);
selectedColumns_tinympc_solvetimes = TinyMPC_S_P_1(:, tinympc_solvetimes_selectedIndices);
tinympc_solvetimes_P_1 = table2array(selectedColumns_tinympc_solvetimes);
selectedColumns_tinympc_solvetimes = TinyMPC_S_P_2(:, tinympc_solvetimes_selectedIndices);
tinympc_solvetimes_P_2 = table2array(selectedColumns_tinympc_solvetimes);
selectedColumns_tinympc_solvetimes = TinyMPC_S_P_3(:, tinympc_solvetimes_selectedIndices);
tinympc_solvetimes_P_3 = table2array(selectedColumns_tinympc_solvetimes);

[numRows, numCols] = size(tinympc_solvetimes_P_1); % Assuming both tables have the same size


for i = 1:numCols
    % Extract columns from table1 and table2
    col_table1_p1 = tinympc_solvetimes_P_1(:, i);
    col_table2_p1 = tinympc_iterations_P_1(:, i);
    
    % Perform element-wise division
    tinympc_s_result_p1(:,i) = col_table1_p1 ./ col_table2_p1;
    col_table1_p2 = tinympc_solvetimes_P_2(:, i);
    col_table2_p2 = tinympc_iterations_P_2(:, i);
    
    % Perform element-wise division
    tinympc_s_result_p2(:,i) = col_table1_p2 ./ col_table2_p2;
    col_table1_p3 = tinympc_solvetimes_P_3(:, i);
    col_table2_p3 = tinympc_iterations_P_3(:, i);
    
    % Perform element-wise division
    tinympc_s_result_p3(:,i) = col_table1_p3 ./ col_table2_p3;

end
% Combine the three column arrays into a single matrix
tinympc_data = [tinympc_s_result_p1(:,:), tinympc_s_result_p2(:,:), tinympc_s_result_p3(:,:)];
% Calculate the row-wise average
tinympc_s_result = mean(tinympc_data, 2);
combined_matrix = cat(3, tinympc_s_result_p1, tinympc_s_result_p2, tinympc_s_result_p3);

% Calculate the row-wise average across the three matrices
tinympc_s_result = mean(combined_matrix, 3);
% Combine the three column arrays into a single matrix

combined_matrix = cat(3, osqp_s_result_p1, osqp_s_result_p2, osqp_s_result_p3);

% Calculate the row-wise average across the three matrices
osqp_s_result = mean(combined_matrix, 3);

time_div = 1000;
xs_tinympc = [2,4,8,16,32];
xs_tinympc_index = 1:numel(xs_tinympc);
xs_osqp = [2,4,8,16];
xs_osqp_index = 1:numel(xs_osqp);

tinympc_s_result = (round(tinympc_s_result));
osqp_s_result = (round(osqp_s_result));

figure('Position', [100, 100, 800, 600]);
min_y = min(tinympc_s_result)
max_y = max(tinympc_s_result)
for i = 1:numel(xs_tinympc_index)
    plot([xs_tinympc_index(i)-0.3, xs_tinympc_index(i)+0.3], [max_y(i), max_y(i)], '-','color', [0, 0, 0.6], 'LineWidth', 1.5);
    hold on 
    plot([xs_tinympc_index(i)-0.3, xs_tinympc_index(i)+0.3], [min_y(i), min_y(i)], '-','color', [0, 0, 0.6], 'LineWidth', 1.5);
    hold on
end

semilogy(xs_tinympc_index,mean(tinympc_s_result),'Marker','.','MarkerSize',25,'color', [0, 0, 0.6], 'LineWidth', 5)
mean(tinympc_s_result)
hold on 
for i = 1:numel(xs_tinympc_index)
    line([xs_tinympc_index(i) xs_tinympc_index(i)], [max_y(i) min_y(i)], 'Color', [0, 0, 0.6], 'LineStyle', '-','LineWidth', 1.5);
end

xticks(xs_tinympc_index);
%yticks([4.5, 5.5, 6.5, 7.5, 8.5, 9.5]); % Specify custom y-axis tick values
xticklabels({'2', '4', '8', '16', '32'});
xlabel('State dimension (N)', 'FontSize', 24);
ylabel('Time per iteration (us) - Log Scale', 'FontSize', 24);
hold on

min_y = min(osqp_s_result)
max_y = max(osqp_s_result)
for i = 1:numel(xs_osqp_index)
    plot([xs_osqp_index(i)-0.3, xs_osqp_index(i)+0.3], [max_y(i), max_y(i)], 'r-', 'LineWidth', 1.5);
    hold on 
    plot([xs_osqp_index(i)-0.3, xs_osqp_index(i)+0.3], [min_y(i), min_y(i)], 'r-', 'LineWidth', 1.5);
    hold on
end
%{
plot(xs_tinympc, max_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold on
plot(xs_tinympc, min_y, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
hold on
%}
semilogy(xs_osqp_index,mean(osqp_s_result),'LineStyle','-.','Marker','.','MarkerSize',25,'color', [1, 0, 0], 'LineWidth', 5)
mean(osqp_s_result)
hold on 
for i = 1:numel(xs_osqp_index)
    line([xs_osqp_index(i) xs_osqp_index(i)], [max_y(i) min_y(i)], 'Color', 'r', 'LineStyle', '-','LineWidth', 1.5);
end
set(gca,'yscale','log')
yticks([1e2, 1e3, 1e4]);
%grid on;
matlab2tikz('Safety_filter_state.tikz');
x = mean(tinympc_s_result(:,1:4))
y = mean(osqp_s_result)
y./x
mean(y./x)