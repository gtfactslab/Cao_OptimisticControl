clear
files_op = ["results/results_0.txt", "results/results_1.txt"];
files_oi = ["results/results_2.txt", "results/results_3.txt"];
files_oj = ["results/results_4.txt", "results/results_5.txt"];

data = [];
costs = [];


ignore = [];
for i = 1:numel(files_op)
    fid = fopen(files_op(i), 'r');
    fid2 = fopen(files_oi(i), 'r');
    fid3 = fopen(files_oj(i), 'r');
    
    tline = fgetl(fid); % skip the first line, it's a header
    tline = fgetl(fid);
    
    tline2 = fgetl(fid2); % skip the first line, it's a header
    tline2 = fgetl(fid2);
    
    tline3 = fgetl(fid3); % skip the first line, it's a header
    tline3 = fgetl(fid3);
    
    while ischar(tline)
%         disp(tline);
        dline = str2num(tline);
        dcost = dline(3:end);
        dline2 = str2num(tline2);
        dcost2 = dline2(3:end);
        dline3 = str2num(tline3);
        dcost3 = dline3(3:end);
        
        tline = fgetl(fid);
        tline2 = fgetl(fid2);
        tline3 = fgetl(fid3);
        
        [~, I] = min(dcost);
        if any(dcost < 0.001) || I ~= 3 %|| any(dcost > 400)
            continue;
        end
        
        data = [data; [dline, dcost2, dcost3]];
        costs = [costs; [dcost, dcost2, dcost3]];
    end
    
    fclose(fid);
end

% pessimism first, then largest to smallest
diff = [costs(:, 2), costs(:, 6), costs(:, 1), costs(:, 5), costs(:, 4), costs(:, 7), costs(:, 8), costs(:, 9)];

edges = [0:50:500];
full_edges = [-500:50:500];

% figure(1)
% subplot(4, 2, 1)
% histogram(diff(:, 1), edges);
% title('Opt5')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% subplot(4, 2, 2)
% histogram(diff(:, 2), edges);
% title('Opt4')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% subplot(4, 2, 3)
% histogram(diff(:, 3), edges);
% title('Opt3')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% subplot(4, 2, 4)
% histogram(diff(:, 4), edges);
% title('Opt2')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% subplot(4, 2, 5)
% histogram(diff(:, 5), edges);
% title('Opt1')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% subplot(4, 2, 6)
% histogram(diff(:, 6), edges);
% title('Opt05')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% subplot(4, 2, 7)
% histogram(diff(:, 7), edges);
% title('Opt01')
% xlabel('Suboptimality')
% ylabel('Number of Runs')
% 

% CDF
figure(2)
lplt = cdfplot(diff(:, 1));
hold on
set(lplt, 'LineStyle', '-', 'Color', 'r');
lplt = cdfplot(diff(:, 2));
set(lplt, 'LineStyle', ':', 'Color', 'k');
lplt = cdfplot(diff(:, 3));
set(lplt, 'LineStyle', ':', 'Color', 'k');
lplt = cdfplot(diff(:, 4));
set(lplt, 'LineStyle', ':', 'Color', 'k');
lplt = cdfplot(diff(:, 5));
set(lplt, 'LineStyle', '-', 'Color', 'g');
lplt = cdfplot(diff(:, 6));
set(lplt, 'LineStyle', ':', 'Color', 'k');
lplt = cdfplot(diff(:, 7));
set(lplt, 'LineStyle', ':', 'Color', 'k');
lplt = cdfplot(diff(:, 8));
set(lplt, 'LineStyle', ':', 'Color', 'k');
axis([0, 600, 0, 1]);
legend('Pes', 'Opt5', 'Opt4', 'Opt3', 'Opt2', 'Opt1','Opt05','Opt01', 'Location', 'southeast')
xlabel('Cost Incurred')
ylabel('Empirical Probability')
hold off

pess = datastats(diff(:, 1))
opt5 = datastats(diff(:, 2))
opt4 = datastats(diff(:, 3))
opt3 = datastats(diff(:, 4))
opt2 = datastats(diff(:, 5))
opt1 = datastats(diff(:, 6))
opt05 = datastats(diff(:,7))
opt01 = datastats(diff(:,8))