%updates box plotting object
% box_coords: struct containing box_coords.x and box_coords.y
% box_coords.x: x coords of all corners of the box
% box_coords.y: y coords of all corners of the box (must be in same order)
function update_box_plot(box_plot_struct, box_coords)
x_data = box_coords.x;
y_data = box_coords.y;
set(box_plot_struct.plot,...
'XData', x_data,...
'ydata', y_data);
end