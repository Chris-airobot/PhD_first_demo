import open3d as o3d
import numpy as np

class PointPicker:
    def __init__(self, point_cloud):
        self.pcd = point_cloud
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(self.pcd)
        self.vis.register_animation_callback(self.animation_callback)
        self.points_picked = []

    def pick_points(self):
        self.vis.run()

    def animation_callback(self, vis):
        if len(self.points_picked) > 0:
            # If points are picked, stop animation
            vis.register_animation_callback(None)

    def pick_handler(self, vis, event):
        if event.type == o3d.visualization.GUIEvent.EventType.KeyDown:
            if event.key == ord("P"):
                print("Point picking mode activated. Click on points. Press 'P' again to finish.")
                vis.register_animation_callback(self.animation_callback)
                vis.poll_events()
                vis.update_renderer()
            elif event.key == ord("Q"):
                print("Quitting point picking mode.")
                vis.register_animation_callback(None)
                vis.poll_events()
                vis.update_renderer()

        elif event.type == o3d.visualization.GUIEvent.EventType.MouseButtonPress:
            if event.button == o3d.visualization.GUIEvent.MouseButton.Left:
                print("Clicked at", event.picked_points[0])
                self.points_picked.append(event.picked_points[0])

# Load your point cloud
pcd = o3d.io.read_point_cloud("/home/riot/kinova_gen3_lite/src/first_demo/pcds/1702357444278407.pcd")

# Create a PointPicker instance
picker = PointPicker(pcd)

# Register pick handler
picker.vis.register_gui_event(picker.pick_handler)

# Run the point picking mode
picker.pick_points()

# Access the color information for the picked points
for point in picker.points_picked:
    color = pcd.colors[pcd.compute_point_cloud_distance(np.array([point]))[1][0]]
    print(f"Color at {point}: {color}")
