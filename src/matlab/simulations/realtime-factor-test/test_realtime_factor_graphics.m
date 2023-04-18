clear
figure("Position",[0,0,1400,700]);
for i=1:4
    switch i
        case 1
            load ("rtf-100hz.mat");
        case 2
            load ("rtf-50hz.mat");
        case 3
            load ("rtf-25hz.mat")
        case 4
            load ("rtf-125hz.mat")
    end
    
    proc_data = zeros(46,3);
    for times = 1:46
        %Data1
        part_data = data(((times-1)*40)+1:((times-1)*40)+40,2);
        uavs = data(((times-1)*40)+1,1);
        mean_part_data = mean(part_data');
        std_part_data = std(part_data');
        proc_data(times,:) = [uavs mean_part_data std_part_data];
    end

     switch i
         case 1
             color = [0.00,0.45,0.74];
             legend_label = 'Navigation ex. at 100Hz';
         case 2
             color = [0.85,0.33,0.10];
             legend_label = 'Navigation ex. at 50Hz';
         case 3
             color = [0.49,0.18,0.56];
             legend_label = 'Navigation ex. at 25Hz';
         case 4
             color = [0.93,0.69,0.13];
             legend_label = 'Navigation ex. at 12.5Hz';
     end

    curve1 = proc_data(:,2)' + proc_data(:,3)';
    curve2 = proc_data(:,2)' - proc_data(:,3)';
    x2 = [proc_data(:,1)', fliplr(proc_data(:,1)')];
    inBetween = [curve1, fliplr(curve2)];
    fill(x2, inBetween, color, 'FaceAlpha', 0.2, 'EdgeColor','none');

    p = plot(proc_data(:,1)', proc_data(:,2)', 'LineWidth', 2, 'Color', color);
    p.DisplayName = legend_label;
    
    xlim([0 360])
    hold on;
    grid on;
    ylabel('Real time factor in the simulation', 'FontSize',16)
    xlabel('Number of UAVs in the simulation', 'FontSize',16)
    %legend('Navigation ex. at 100Hz','Navigation ex. at 500Hz','Navigation ex. at 25Hz','Navigation ex. at 12,5Hz')
    ax = gca;
    ax.FontSize = 16;
    
    clear data;
end

patch = findobj(ax, 'Type', 'patch');
for i=1:length(patch)
    patch(i).HandleVisibility = 'off';
end
legend();