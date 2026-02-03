import numpy as np
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg


class GeneralDashboard(QMainWindow):
    def __init__(self, plot_configs, buffer_size=250):
        """
        plot_configs: A list of dicts, e.g. [{"title": "Pos", "color": "g"}, ...]
        buffer_size: How many history points to keep
        """
        super().__init__()
        self.setWindowTitle("General MuJoCo Dashboard")
        self.resize(600, 200 * len(plot_configs)) # Scale height based on plot count
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        
        # Storage for our references
        self.curves = []      # The pyqtgraph Curve objects
        self.data_buffers = [] # The numpy arrays holding history
        self.time_buffer = np.zeros(buffer_size)  # Time values for x-axis
        self.buffer_size = buffer_size

        # --- Dynamic Creation Loop ---
        for config in plot_configs:
            # 1. Create the Plot Widget
            plot_widget = pg.PlotWidget(title=config.get("title", "Untitled"))
            plot_widget.showGrid(x=True, y=True)
            layout.addWidget(plot_widget)
            
            # 2. Create the Curve (Line)
            color = config.get("color", "w") # Default to white if no color provided
            curve = plot_widget.plot(pen=color)
            self.curves.append(curve)
            
            # 3. Create the Data Buffer
            # We add a new zero-filled array to our list of buffers
            self.data_buffers.append(np.zeros(buffer_size))

    def update_dashboard(self, new_values, current_time):
        """
        new_values: A list or tuple of numbers corresponding to the plots.
        current_time: Current simulation time for x-axis
        Example: [current_pos, current_vel, current_acc], 1.234
        """
        # Ensure we received the right amount of data
        if len(new_values) != len(self.curves):
            print(f"Warning: Expected {len(self.curves)} values, got {len(new_values)}")
            return

        # Shift time buffer
        self.time_buffer[:-1] = self.time_buffer[1:]
        self.time_buffer[-1] = current_time

        # Loop through every plot and update it
        for i, val in enumerate(new_values):
            # Shift buffer: discard old, add new
            self.data_buffers[i][:-1] = self.data_buffers[i][1:]
            self.data_buffers[i][-1] = val
            
            # Update the visual curve with time on x-axis
            self.curves[i].setData(self.time_buffer, self.data_buffers[i])
