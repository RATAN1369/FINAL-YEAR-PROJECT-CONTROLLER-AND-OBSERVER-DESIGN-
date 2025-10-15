function PlotResults(t, Vout_ref, Vout_adrc, Vout_pid, I_L_adrc, I_L_pid, ...
                    duty_adrc, duty_pid, disturbance_estimated, R_load, Vin_actual)
%% Plot all simulation results

    % Graph 1: Output Voltage Comparison
    figure('Position', [100, 100, 1200, 500]);
    
    subplot(1,2,1);
    plot(t*1000, Vout_ref, 'k--', 'LineWidth', 3); hold on;
    plot(t*1000, Vout_adrc, 'b-', 'LineWidth', 2);
    plot(t*1000, Vout_pid, 'r-', 'LineWidth', 1.5);
    xlabel('Time (ms)');
    ylabel('Output Voltage (V)');
    title('DC-DC Converter: Output Voltage Regulation');
    legend('Reference (12V)', 'ADRC', 'PID', 'Location', 'southeast');
    grid on;
    ylim([11.5, 12.5]);
    
    % Graph 2: Disturbance Estimation
    subplot(1,2,2);
    actual_disturbance = (Vin_actual - 24)./24 * 10 + (10 - R_load)./10 * 5;
    plot(t*1000, actual_disturbance, 'k-', 'LineWidth', 2); hold on;
    plot(t*1000, disturbance_estimated, 'b-', 'LineWidth', 2);
    xlabel('Time (ms)');
    ylabel('Disturbance Magnitude');
    title('Disturbance Estimation - Luenberger Observer');
    legend('Actual Disturbance', 'Estimated', 'Location', 'southeast');
    grid on;
    
    sgtitle('DC-DC Buck Converter: ADRC vs PID Performance', 'FontSize', 14, 'FontWeight', 'bold');
    saveas(gcf, 'converter_voltage_comparison.png');
    
    % Additional plots...
    % [Include the plotting code from previous versions for control effort and performance summary]
end
