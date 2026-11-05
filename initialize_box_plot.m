function box_plot_struct = initialize_box_plot()
box_plot_struct = struct();
box_plot_struct.plot =  patch('XData', 0, 'YData', 0, 'FaceColor', 'blue');
end