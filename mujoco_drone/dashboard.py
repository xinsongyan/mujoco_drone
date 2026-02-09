import numpy as np
from PyQt5.QtWidgets import QApplication

from debugs.general_dashboard import GeneralDashboard


class DroneSimulationDashboard:
    """Dashboard for displaying drone simulation data."""
    
    def __init__(self):
        """Initialize the dashboard with default plots."""
        self.app = QApplication.instance()
        if self.app is None:
            self.app = QApplication([])
        
        # Configure position and control plots
        self.position_plots = [
            {"title": "Drone Position X (m)", "color": "r"},
            {"title": "Drone Position Y (m)", "color": "g"},
            {"title": "Drone Position Z (m)", "color": "b"},
            {"title": "Thrust Total (N)", "color": "m"},
            {"title": "Drone Roll (deg)", "color": "r"},
            {"title": "Drone Pitch (deg)", "color": "g"},
            {"title": "Drone Yaw (deg)", "color": "b"}
        ]
        
        self.dashboard = GeneralDashboard(self.position_plots)
        self.dashboard.show()
    
    def update(self, drone, time):
        """Update dashboard with drone state data.
        
        Args:
            drone: SimpleDrone object
            time: Simulation time
        """
        pos_x = drone.d.qpos[0]
        pos_y = drone.d.qpos[1]
        pos_z = drone.d.qpos[2]
        thrust = drone.thrust_total
        roll, pitch, yaw = drone.state_estimator.base_rpy

        self.dashboard.update_dashboard(
            [pos_x, pos_y, pos_z, thrust, np.degrees(roll), np.degrees(pitch), np.degrees(yaw)],
            time
        )
        self.app.processEvents()
