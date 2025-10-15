function PlotThermalResults(t, ref, T_adrc, T_pid, heater_adrc, heater_pid, ...
                           disturbance_estimated, total_disturbance, ambient_variation)

    % Graph 1: Temperature Response Comparison
    figure('Position', [100, 100, 1200, 500]);
    
    subplot(1,2,1);
    plot(t, ref, 'k--', 'LineWidth', 3); hold on;
    plot(t, T_adrc, 'b-', 'LineWidth', 2);
    plot(t, T_pid, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Temperature (°C)');
    title('Temperature Control: ADRC vs PID');
    legend('Reference (80°C)', 'ADRC', 'PID', 'Location', 'southeast');
    grid on;
    ylim([70, 90]);
    
    % Mark disturbance events
    line([600, 600], [70, 90], 'Color', 'green', 'LineStyle', ':', 'LineWidth', 1);
    line([900, 900], [70, 90], 'Color', 'green', 'LineStyle', ':', 'LineWidth', 1);
    text(600, 91, 'Ambient↓', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'green');
    text(900, 91, 'Ambient↑', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'green');
    
    % Graph 2: Disturbance Estimation
    subplot(1,2,2);
    plot(t, total_disturbance, 'k-', 'LineWidth', 2); hold on;
    plot(t, disturbance_estimated, 'b-', 'LineWidth', 2);
    plot(t, ambient_variation, 'g--', 'LineWidth', 1, 'DisplayName', 'Ambient Only');
    xlabel('Time (s)');
    ylabel('Disturbance Magnitude (°C/s)');
    title('Thermal Disturbance Estimation - Luenberger Observer');
    legend('Total Disturbance', 'Estimated', 'Ambient Component', 'Location', 'southeast');
    grid on;
    
    sgtitle('Temperature Control System Performance', 'FontSize', 14, 'FontWeight', 'bold');
    saveas(gcf, 'temp_response_comparison.png');
    
    % Additional plots for control effort and detailed analysis
    figure('Position', [100, 100, 1000, 400]);
    
    subplot(1,2,1);
    plot(t, heater_adrc, 'b-', 'LineWidth', 2); hold on;
    plot(t, heater_pid, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Heater Power (W)');
    title('Control Effort Comparison');
    legend('ADRC', 'PID', 'Location', 'northeast');
    grid on;
    
    subplot(1,2,2);
    zoom_idx = t >= 550 & t <= 750;
    plot(t(zoom_idx), ref(zoom_idx), 'k--', 'LineWidth', 2); hold on;
    plot(t(zoom_idx), T_adrc(zoom_idx), 'b-', 'LineWidth', 2);
    plot(t(zoom_idx), T_pid(zoom_idx), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Temperature (°C)');
    title('Zoom: Disturbance Rejection at 600s');
    legend('Reference', 'ADRC', 'PID', 'Location', 'southeast');
    grid on;
    
    sgtitle('Temperature Control: Detailed Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    saveas(gcf, 'temp_detailed_analysis.png');
end
