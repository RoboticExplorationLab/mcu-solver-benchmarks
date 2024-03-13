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

tinympc_s_result = round(tinympc_s_result);
osqp_s_result = round(osqp_s_result);

figure('Position', [100, 100, 800, 600]);
min_y = min(tinympc_s_result)
max_y = max(tinympc_s_result)
for i = 1:numel(xs_tinympc_index)
    plot([xs_tinympc_index(i)-0.2, xs_tinympc_index(i)+0.2], [max_y(i), max_y(i)], 'b-', 'LineWidth', 1.5);
    hold on 
    plot([xs_tinympc_index(i)-0.2, xs_tinympc_index(i)+0.2], [min_y(i), min_y(i)], 'b-', 'LineWidth', 1.5);
    hold on
end
%{
plot(xs_tinympc, max_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold on
plot(xs_tinympc, min_y, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
hold on
%}
plot(xs_tinympc_index,mean(tinympc_s_result),'Marker','.','MarkerSize',25,'color', [0, 0, 1], 'LineWidth', 2)
mean(tinympc_s_result)
hold on 
for i = 1:numel(xs_tinympc_index)
    line([xs_tinympc_index(i) xs_tinympc_index(i)], [max_y(i) min_y(i)], 'Color', 'k', 'LineStyle', '-','LineWidth', 1.5);
end

xticks(xs_tinympc_index);
yticks([0, 800, 1600, 2400, 3200, 4000]); % Specify custom y-axis tick values
xticklabels({'2', '4', '8', '16', '32'});
xlabel('State dimension (N)', 'FontSize', 24);
ylabel('Time per iteration (us)', 'FontSize', 24);
hold on

min_y = min(osqp_s_result)
max_y = max(osqp_s_result)
for i = 1:numel(xs_osqp_index)
    plot([xs_osqp_index(i)-0.2, xs_osqp_index(i)+0.2], [max_y(i), max_y(i)], 'r-', 'LineWidth', 1.5);
    hold on 
    plot([xs_osqp_index(i)-0.2, xs_osqp_index(i)+0.2], [min_y(i), min_y(i)], 'r-', 'LineWidth', 1.5);
    hold on
end
%{
plot(xs_tinympc, max_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold on
plot(xs_tinympc, min_y, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
hold on
%}
plot(xs_osqp_index,mean(osqp_s_result),'Marker','.','MarkerSize',25,'color', [1, 0, 0], 'LineWidth', 2)
mean(osqp_s_result)
hold on 
for i = 1:numel(xs_osqp_index)
    line([xs_osqp_index(i) xs_osqp_index(i)], [max_y(i) min_y(i)], 'Color', 'r', 'LineStyle', '-','LineWidth', 1.5);
end

%setBoxParams(boxplot(osqp_s_result,xs_osqp,'Whisker',0,'OutlierSize', 0), osqp_color, box_lw, osqp_box_color);
%{
Adjust whiskers for each group
for i = 1:size(tinympc_s_result, 2)
    h = findobj(gca, 'Tag', ['Box', num2str(i)]);
    
    % Update whiskers for the i-th box plot
    set(h, 'Whisker', [lower_whisker_lengths(i), upper_whisker_lengths(i)]);
end
%}
%{
if plot_tinympc
    if box
        setBoxParams(boxplot(tinympc_s_result/time_div,xs_tinympc,'Whisker', [lower_whisker_length; upper_whisker_length]), tiny_color, box_lw, tiny_box_color);
       % boxplot(tinympc_s_result/time_div,xs_tinympc);
        %setBoxParams(boxchart(xs_tinympc, data.tinympc.nx/time_div), tiny_color, box_lw, tiny_box_color);
    else
        setScatterParams(scatter(xs_tinympc, tinympc_s_result/time_div), tiny_color)
    end
end
%}
% Increase x-axis tick label font size
set(gca, 'FontSize', 20);

% Increase y-axis tick label font size
set(gca, 'FontSize', 20);
grid on;
matlab2tikz('Safety_filter_state.tikz');
x = mean(tinympc_s_result(:,1:4))
y = mean(osqp_s_result)
y./x
mean(y./x)
function setBoxParams(f, color, lineWidth, boxColor)
    set(f,{'Color'},{color})
    set(f,{'linew'},{lineWidth})
    set(f,{'LineStyle'},{'-'});
    %set(f,{'BoxFaceColor'},{boxColor})
end