SCS_H_2 = readtable('Rocket Landing STM32 SCS.xlsx', "Sheet", 'horizon 2');
SCS_H_4 = readtable('Rocket Landing STM32 SCS.xlsx', "Sheet", 'horizon 4');
SCS_H_8 = readtable('Rocket Landing STM32 SCS.xlsx', "Sheet", 'horizon 8');
SCS_H_16 = readtable('Rocket Landing STM32 SCS.xlsx', "Sheet", 'horizon 16');
SCS_H_32 = readtable('Rocket Landing STM32 SCS.xlsx', "Sheet", 'horizon 32');
ECOS_H_2 = readtable('Rocket Landing STM32 ECOS.xlsx', "Sheet", 'horizon 2');
ECOS_H_4 = readtable('Rocket Landing STM32 ECOS.xlsx', "Sheet", 'horizon 4');
ECOS_H_8 = readtable('Rocket Landing STM32 ECOS.xlsx', "Sheet", 'horizon 8');
ECOS_H_16 = readtable('Rocket Landing STM32 ECOS.xlsx', "Sheet", 'horizon 16');
ECOS_H_32 = readtable('Rocket Landing STM32 ECOS.xlsx', "Sheet", 'horizon 32');
TinyMPC_H_2 = readtable('Rocket Landing STM32 TinyMPC.xlsx', "Sheet", 'horizon 2');
TinyMPC_H_4 = readtable('Rocket Landing STM32 TinyMPC.xlsx', "Sheet", 'horizon 4');
TinyMPC_H_8 = readtable('Rocket Landing STM32 TinyMPC.xlsx', "Sheet", 'horizon 8');
TinyMPC_H_16 = readtable('Rocket Landing STM32 TinyMPC.xlsx', "Sheet", 'horizon 16');
TinyMPC_H_32 = readtable('Rocket Landing STM32 TinyMPC.xlsx', "Sheet", 'horizon 32');
TinyMPC_H_64 = readtable('Rocket Landing STM32 TinyMPC.xlsx', "Sheet", 'horizon 64');

SCS_H_2_iters = table2array(SCS_H_2(:,1));
SCS_H_4_iters = table2array(SCS_H_4(:,1));
SCS_H_8_iters = table2array(SCS_H_8(:,1));
SCS_H_16_iters = table2array(SCS_H_16(:,1));
SCS_H_32_iters = table2array(SCS_H_32(:,1));
ECOS_H_2_iters = table2array(ECOS_H_2(:,1));
ECOS_H_4_iters = table2array(ECOS_H_4(:,1));
ECOS_H_8_iters = table2array(ECOS_H_8(:,1));
ECOS_H_16_iters = table2array(ECOS_H_16(:,1));
ECOS_H_32_iters = table2array(ECOS_H_32(:,1));
TinyMPC_H_2_iters = table2array(TinyMPC_H_2(:,1));
TinyMPC_H_4_iters = table2array(TinyMPC_H_4(:,1));
TinyMPC_H_8_iters = table2array(TinyMPC_H_8(:,1));
TinyMPC_H_16_iters = table2array(TinyMPC_H_16(:,1));
TinyMPC_H_32_iters = table2array(TinyMPC_H_32(:,1));
TinyMPC_H_64_iters = table2array(TinyMPC_H_64(:,1));

SCS_H_2_solve = table2array(SCS_H_2(:,2));
SCS_H_4_solve = table2array(SCS_H_4(:,2));
SCS_H_8_solve = table2array(SCS_H_8(:,2));
SCS_H_16_solve = table2array(SCS_H_16(:,2));
SCS_H_32_solve = table2array(SCS_H_32(:,2));
ECOS_H_2_solve = table2array(ECOS_H_2(:,2));
ECOS_H_4_solve = table2array(ECOS_H_4(:,2));
ECOS_H_8_solve = table2array(ECOS_H_8(:,2));
ECOS_H_16_solve = table2array(ECOS_H_16(:,2));
ECOS_H_32_solve = table2array(ECOS_H_32(:,2));
TinyMPC_H_2_solve = table2array(TinyMPC_H_2(:,2));
TinyMPC_H_4_solve = table2array(TinyMPC_H_4(:,2));
TinyMPC_H_8_solve = table2array(TinyMPC_H_8(:,2));
TinyMPC_H_16_solve = table2array(TinyMPC_H_16(:,2));
TinyMPC_H_32_solve = table2array(TinyMPC_H_32(:,2));
TinyMPC_H_64_solve = table2array(TinyMPC_H_64(:,2));

SCS_hor_solve = horzcat(SCS_H_2_solve, SCS_H_4_solve, SCS_H_8_solve, SCS_H_16_solve, SCS_H_32_solve);
%ECOS_hor_solve = horzcat(ECOS_H_2_solve, ECOS_H_4_solve, ECOS_H_8_solve, ECOS_H_16_solve, ECOS_H_32_solve);
TinyMPC_hor_solve = horzcat(TinyMPC_H_2_solve, TinyMPC_H_4_solve, TinyMPC_H_8_solve, TinyMPC_H_16_solve, TinyMPC_H_32_solve, TinyMPC_H_64_solve);

SCS_hor_iters = horzcat(SCS_H_2_iters, SCS_H_4_iters, SCS_H_8_iters, SCS_H_16_iters, SCS_H_32_iters);
%ECOS_hor_iters = horzcat(ECOS_H_2_iters, ECOS_H_4_iters, ECOS_H_8_iters, ECOS_H_16_iters, ECOS_H_32_iters);
TinyMPC_hor_iters = horzcat(TinyMPC_H_2_iters, TinyMPC_H_4_iters, TinyMPC_H_8_iters, TinyMPC_H_16_iters, TinyMPC_H_32_iters, TinyMPC_H_64_iters);
% Find the maximum length among the arrays
max_length = max([length(ECOS_H_2_solve), length(ECOS_H_4_solve), length(ECOS_H_8_solve), length(ECOS_H_16_solve), length(ECOS_H_32_solve)]);

% Pad shorter arrays with NaN to match the maximum length
ECOS_H_2_solve = [ECOS_H_2_solve; NaN(max_length - length(ECOS_H_2_solve), 1)];
ECOS_H_4_solve = [ECOS_H_4_solve; NaN(max_length - length(ECOS_H_4_solve), 1)];
ECOS_H_8_solve = [ECOS_H_8_solve; NaN(max_length - length(ECOS_H_8_solve), 1)];
ECOS_H_16_solve = [ECOS_H_16_solve; NaN(max_length - length(ECOS_H_16_solve), 1)];
ECOS_H_32_solve = [ECOS_H_32_solve; NaN(max_length - length(ECOS_H_32_solve), 1)];

% Concatenate arrays horizontally to form multiple columns
ECOS_hor_solve = [ECOS_H_2_solve, ECOS_H_4_solve, ECOS_H_8_solve, ECOS_H_16_solve, ECOS_H_32_solve];
max_length = max([length(ECOS_H_2_iters), length(ECOS_H_4_iters), length(ECOS_H_8_iters), length(ECOS_H_16_iters), length(ECOS_H_32_iters)]);

% Pad shorter arrays with NaN to match the maximum length
ECOS_H_2_iters = [ECOS_H_2_iters; NaN(max_length - length(ECOS_H_2_iters), 1)];
ECOS_H_4_iters = [ECOS_H_4_iters; NaN(max_length - length(ECOS_H_4_iters), 1)];
ECOS_H_8_iters = [ECOS_H_8_iters; NaN(max_length - length(ECOS_H_8_iters), 1)];
ECOS_H_16_iters = [ECOS_H_16_iters; NaN(max_length - length(ECOS_H_16_iters), 1)];
ECOS_H_32_iters = [ECOS_H_32_iters; NaN(max_length - length(ECOS_H_32_iters), 1)];

% Concatenate arrays horizontally to form multiple columns
ECOS_hor_iters = [ECOS_H_2_iters, ECOS_H_4_iters, ECOS_H_8_iters, ECOS_H_16_iters, ECOS_H_32_iters];
[numRows, numCols] = size(TinyMPC_hor_solve); % Assuming both tables have the same size
%{
TinyMPC_hor_result(:,1) = TinyMPC_H_2_solve ./ TinyMPC_H_2_iters;
TinyMPC_hor_result(:,2) = TinyMPC_H_4_solve ./ TinyMPC_H_4_iters;
TinyMPC_hor_result(:,3) = TinyMPC_H_8_solve ./ TinyMPC_H_8_iters;
TinyMPC_hor_result(:,4) = TinyMPC_H_16_solve ./ TinyMPC_H_16_iters;
TinyMPC_hor_result(:,5) = TinyMPC_H_32_solve ./ TinyMPC_H_32_iters;
TinyMPC_hor_result(:,6) = TinyMPC_H_64_solve ./ TinyMPC_H_64_iters;
%}
for i = 1:numCols
    % Extract columns from table1 and table2
    col_table1_p1 = TinyMPC_hor_solve(:, i);
    col_table2_p1 = TinyMPC_hor_iters(:, i);
    
    % Perform element-wise division
    TinyMPC_hor_result(:,i) = col_table1_p1 ./ col_table2_p1;

end

[numRows, numCols] = size(SCS_hor_solve); % Assuming both tables have the same size


for i = 1:numCols
    % Extract columns from table1 and table2
    col_table1_p1 = SCS_hor_solve(:, i);
    col_table2_p1 = SCS_hor_iters(:, i);
    
    % Perform element-wise division
    SCS_hor_result(:,i) = col_table1_p1 ./ col_table2_p1;

end

[numRows, numCols] = size(ECOS_hor_solve); % Assuming both tables have the same size


for i = 1:numCols
    % Extract columns from table1 and table2
    col_table1_p1 = ECOS_hor_solve(:, i);
    col_table2_p1 = ECOS_hor_iters(:, i);
    
    % Perform element-wise division
    ECOS_hor_result(:,i) = col_table1_p1 ./ col_table2_p1;

end

time_div = 1000;
xs_tinympc = [2,4,8,16,32,64];
xs_tinympc_index = 1:numel(xs_tinympc);
xs_ecos = [2,4,8,16,32];
xs_ecos_index = 1:numel(xs_ecos);
xs_scs = [2,4,8,16,32];
xs_scs_index = 1:numel(xs_scs);
TinyMPC_hor_result = round(TinyMPC_hor_result);
ECOS_hor_result = round(ECOS_hor_result);
SCS_hor_result = round(SCS_hor_result);

figure('Position', [100, 100, 800, 600]);


hold on
min_y = min(TinyMPC_hor_result)
max_y = max(TinyMPC_hor_result)
for i = 1:numel(xs_tinympc_index)
    plot([xs_tinympc_index(i)-0.2, xs_tinympc_index(i)+0.2], [max_y(i), max_y(i)], 'b-', 'LineWidth', 1.5);
    hold on 
    plot([xs_tinympc_index(i)-0.2, xs_tinympc_index(i)+0.2], [min_y(i), min_y(i)], 'b-', 'LineWidth', 1.5);
    hold on
end

line1 = plot(xs_tinympc_index,mean(TinyMPC_hor_result),'Marker','.','MarkerSize',25,'color', [0, 0, 1], 'LineWidth', 2, 'DisplayName','TinyMPC')
hold on 
for i = 1:numel(xs_tinympc_index)
    line([xs_tinympc_index(i) xs_tinympc_index(i)], [max_y(i) min_y(i)], 'Color', 'k', 'LineStyle', '-','LineWidth', 1.5);
end

xticks(xs_tinympc_index);
yticks([0, 4000, 8000, 12000, 16000, 20000]); % Specify custom y-axis tick values
xticklabels({'2', '4', '8', '16', '32','64'});
xlabel('Time Horizon (N)', 'FontSize', 24);
ylabel('Time per iteration (us)', 'FontSize', 24);
hold on

min_y = nanmin(ECOS_hor_result)
max_y = nanmax(ECOS_hor_result)
for i = 1:numel(xs_ecos_index)
    plot([xs_ecos_index(i)-0.2, xs_ecos_index(i)+0.2], [max_y(i), max_y(i)], 'r-', 'LineWidth', 1.5);
    hold on 
    plot([xs_ecos_index(i)-0.2, xs_ecos_index(i)+0.2], [min_y(i), min_y(i)], 'r-', 'LineWidth', 1.5);
    hold on
end

line2 = plot(xs_ecos_index,nanmean(ECOS_hor_result),'Marker','.','MarkerSize',25,'color', [1, 0, 0], 'LineWidth', 2, 'DisplayName','ECOS')

hold on 
for i = 1:numel(xs_ecos_index)
    line([xs_ecos_index(i) xs_ecos_index(i)], [max_y(i) min_y(i)], 'Color', 'r', 'LineStyle', '-','LineWidth', 1.5);
end

min_y = min(SCS_hor_result)
max_y = max(SCS_hor_result)
for i = 1:numel(xs_scs_index)
    plot([xs_scs_index(i)-0.2, xs_scs_index(i)+0.2], [max_y(i), max_y(i)], 'g-', 'LineWidth', 1.5);
    hold on 
    plot([xs_scs_index(i)-0.2, xs_scs_index(i)+0.2], [min_y(i), min_y(i)], 'g-', 'LineWidth', 1.5);
    hold on
end

line3 = plot(xs_scs_index,mean(SCS_hor_result),'Marker','.','MarkerSize',25,'color', [0, 1, 0], 'LineWidth', 2, 'DisplayName','SCS')

hold on 
for i = 1:numel(xs_scs_index)
    line([xs_scs_index(i) xs_scs_index(i)], [max_y(i) min_y(i)], 'Color', 'g', 'LineStyle', '-','LineWidth', 1.5);
end

% Increase x-axis tick label font size
set(gca, 'FontSize', 20);

% Increase y-axis tick label font size
set(gca, 'FontSize', 20);
legend([line1, line2,line3], 'TinyMPC','ECOS','SCS');
box on;
grid on;

matlab2tikz('Rocket_landing_horizon.tikz');

a = nanmean(TinyMPC_hor_result(:,1:5))
b = nanmean(ECOS_hor_result)
c = mean(SCS_hor_result)
x = c./a
mean(x)
y = b./a
mean(y)