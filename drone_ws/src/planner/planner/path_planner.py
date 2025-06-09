import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from typing import List, Tuple
from shapely.geometry import Polygon, Point, LineString
import networkx as nx
import matplotlib.pyplot as plt

class path_planner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.boundary_xy: List[Tuple[float, float]] | None = None
        self.waypoints_xy: List[Tuple[float, float]] | None = None
        self.publisher_ = self.create_publisher(Path, 'planning', 10)
        self.boundary_sub = self.create_subscription(
            Path,
            'mission_boundaries',
            self.boundaries_callback,
            10)
        self.waypoints_sub = self.create_subscription(
            Path, 'mission_waypoints', self.waypoints_callback, 10)

    def boundaries_callback(self, boundaries: Path)-> None:
        """Save boundary vertices"""
        self.boundary_xy = [(p.pose.position.x, p.pose.position.y) for p in boundaries.poses]
        self.get_logger().info(f'Received mission boundaries')
        self.plan()

    def waypoints_callback(self, waypoints: Path)-> None:
        """Save waypoints"""
        self.waypoints_xy = [(p.pose.position.x, p.pose.position.y) for p in waypoints.poses]
        self.get_logger().info(f'Received {len(self.waypoints_xy)} way‑points')
        self.plan()
    
    def plan(self)-> None:
        #if self.boundary_xy is None or self.waypoints_xy is None:
        #    return  # haven’t got both yet
        try:
            boundary_ = [
            (0, 0),
            (8, 1),
            (6, 4),
            (9, 8),
            (5, 7),
            (2, 10),
            (0, 7),
            (-2, 5),
            (-1, 2)
            ]
            waypoints = [
                (1, 1),
                (1, 6),
                (2, 8),
                (3, 2),
                (4, 5),
                (6, 6)
            ]
            self.boundary_xy = boundary_
            self.waypoints_xy = waypoints
            path_xy = self.build_graph(
                self.boundary_xy,
                self.waypoints_xy,
                start=(0.5, 0.5)  # drone’s take‑off point in mission_origin frame 
                #TODO: potentially change that
            )
        except Exception as e:
            self.get_logger().error(f'Planner failed: {e}')
            return

        self.publish_path(path_xy)

    def build_graph(self, 
        boundary: List[Tuple[float, float]],
        waypoints: List[Tuple[float, float]],
        start: Tuple[float, float]
        )-> List[Tuple[float, float]]:

        """ Build 2D Visibility Graph """
        #Create the mission bonudary polygon
        boundary_polygon = Polygon(boundary)# Define your polygon (example: square with a hole) 
        #visualise(boundary_polygon)
        
        #Ensure the wps are within the mission boundary
        for wp in waypoints:
            if boundary_polygon.contains(Point(wp)):
                print("Waypoint is inside the mission boundary.")
            else:
                self.get_logger().warn(f"Waypoint {wp} is outside mission boundary!")
        #Assemble the node set
        nodes = [start] + waypoints + boundary
        #Build the visibility edges
        edges = []
        for i, u in enumerate(nodes):
            for j, v in enumerate(nodes):
                if i >= j:
                    continue  # avoid duplicate pairs and self-pairs
                
                line = LineString([u, v])
                
                if boundary_polygon.covers(line):
                    # edge is valid
                    weight = line.length  # Euclidean distance
                    edges.append( (u, v, weight) )
        self.plot_visibility_graph(boundary, nodes, edges)

        #Build the graph
        G = nx.Graph()
        G.add_nodes_from(nodes)
        G.add_weighted_edges_from(edges)

        #Build the path -- expectation: wps are ordered
        full_path = [start]
        current = start
        for target in waypoints:
            segment = nx.shortest_path(G, source=current, target=target, weight="weight")
            full_path.extend(segment[1:])  # avoid duplicating the first node
            current = target
        #print(full_path)
        return full_path


    def publish_path(self, full_path: List[Tuple[float, float]]) -> None:
        """
        Convert a list of (x, y) tuples into nav_msgs/Path and publish it.
        • Z is fixed to 0.0
        • Orientation is the identity quaternion (w = 1)
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'   # <- change to your global frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in full_path:
            pose = PoseStamped()
            pose.header = path_msg.header          # same frame & stamp

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # Identity quaternion (no rotation): (x, y, z, w) = (0, 0, 0, 1)
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} poses')
    
    
    def visualise(polygon: Polygon):
        # Plotting
        print(polygon)
        x, y = polygon.exterior.xy
        plt.plot(x, y, color='blue', label='Exterior')

        for interior in polygon.interiors:
            ix, iy = interior.xy
            plt.plot(ix, iy, color='red', linestyle='--', label='Interior')

        plt.gca().set_aspect('equal')
        plt.legend()
        plt.title("Polygon Visualization")
        plt.grid(True)
        plt.show()

    def plot_visibility_graph(self, boundary, nodes, edges) -> None:
        boundary_polygon = Polygon(boundary)
        
        # Plot the polygon boundary
        x, y = boundary_polygon.exterior.xy
        plt.plot(x, y, color='blue', label='Boundary')

        # Plot nodes
        nx, ny = zip(*nodes)
        plt.scatter(nx, ny, color='purple', zorder=5, label='Nodes')

        # Plot edges
        for u, v, _ in edges:
            line = LineString([u, v])
            lx, ly = line.xy
            plt.plot(lx, ly, color='green', linewidth=1, zorder=2)
        
        plt.gca().set_aspect('equal')
        plt.legend()
        plt.title('Visibility Graph')
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    node = path_planner()
    try:
        #rclpy.spin(node)
        node.plan()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()