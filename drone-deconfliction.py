import numpy as np
from math import sqrt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.spatial import KDTree
import time

class Waypoint:
    def __init__(self, x, y, z=None, t=None):
        self.x = x
        self.y = y
        self.z = z if z is not None else 0
        self.t = t

class FlightPlan:
    def __init__(self, flight_id, waypoints):
        self.id = flight_id
        self.waypoints = waypoints
        self.segments = self._create_segments()
        
    def _create_segments(self):
        segments = []
        for i in range(len(self.waypoints)-1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i+1]
            segments.append((wp1, wp2))
        return segments

class DeconflictionEngine:
    def __init__(self, buffer=5.0):
        self.flights = {}
        self.buffer = buffer
        self.kd_tree = None
        self.flight_points = []

    def add_flight(self, flight_plan):
        self.flights[flight_plan.id] = flight_plan
        self._update_spatial_index()
    
    def _update_spatial_index(self):
        self.flight_points = []
        for fid, flight in self.flights.items():
            for wp in flight.waypoints:
                self.flight_points.append((wp.x, wp.y, wp.z, wp.t, fid))
        if self.flight_points:
            self.kd_tree = KDTree([(p[0], p[1], p[2], p[3]) for p in self.flight_points])
    
    def check_mission(self, primary_mission):
        primary_id = "primary"
        primary_segments = FlightPlan(primary_id, primary_mission).segments
        
        conflicts = []
        for wp1, wp2 in primary_segments:
            t_start, t_end = min(wp1.t, wp2.t), max(wp1.t, wp2.t)
            query_points = self._generate_query_points(wp1, wp2)
            
            for point in query_points:
                x, y, z, t = point
                if self.kd_tree:
                    dists, idxs = self.kd_tree.query([(x, y, z, t)], k=5, distance_upper_bound=self.buffer)
                    for dist, idx in zip(dists[0], idxs[0]):
                        if dist < float('inf'):
                            conflict_point = self.flight_points[idx]
                            conflict_time = conflict_point[3]
                            if abs(conflict_time - t) < 1.0:  # Temporal proximity check
                                conflicts.append({
                                    'location': (x, y, z),
                                    'time': t,
                                    'conflict_flight': conflict_point[4],
                                    'distance': dist
                                })
        return conflicts if conflicts else None
    
    def _generate_query_points(self, wp1, wp2, num_points=50):
        points = []
        for ratio in np.linspace(0, 1, num_points):
            t = wp1.t + ratio * (wp2.t - wp1.t)
            x = wp1.x + ratio * (wp2.x - wp1.x)
            y = wp1.y + ratio * (wp2.y - wp1.y)
            z = wp1.z + ratio * (wp2.z - wp1.z)
            points.append((x, y, z, t))
        return points

# Visualization Functions
def plot_3d_trajectories(primary, flights, conflicts=None):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot primary mission
    primary_x = [wp.x for wp in primary]
    primary_y = [wp.y for wp in primary]
    primary_z = [wp.z for wp in primary]
    ax.plot(primary_x, primary_y, primary_z, 'b-o', linewidth=2, markersize=8, label='Primary Mission')
    
    # Plot other flights
    colors = ['r', 'g', 'm', 'c']
    for i, (fid, flight) in enumerate(flights.items()):
        x = [wp.x for wp in flight.waypoints]
        y = [wp.y for wp in flight.waypoints]
        z = [wp.z for wp in flight.waypoints]
        ax.plot(x, y, z, f'{colors[i%len(colors)]}--^', linewidth=1.5, label=f'Flight {fid}')
    
    # Plot conflicts
    if conflicts:
        for conflict in conflicts:
            x, y, z = conflict['location']
            ax.scatter(x, y, z, s=200, c='red', marker='*', alpha=0.8)
            ax.text(x, y, z, f"t={conflict['time']:.1f}s", color='black')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('3D Flight Trajectories with Conflict Zones')
    ax.legend()
    plt.tight_layout()
    plt.savefig('flight_trajectories.png')
    plt.show()

def animate_conflicts(primary, flights, conflicts):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    time_text = ax.text(0.05, 0.95, 0.05, '', transform=ax.transAxes)
    
    def update(frame):
        ax.cla()
        time_text.set_text(f'Time: {frame:.1f}s')
        
        # Plot primary
        primary_x = [wp.x for wp in primary]
        primary_y = [wp.y for wp in primary]
        primary_z = [wp.z for wp in primary]
        ax.plot(primary_x, primary_y, primary_z, 'b-o', label='Primary')
        
        # Plot other flights at current time
        for fid, flight in flights.items():
            x, y, z = [], [], []
            for wp in flight.waypoints:
                if wp.t <= frame:
                    x.append(wp.x)
                    y.append(wp.y)
                    z.append(wp.z)
            if x:
                ax.plot(x, y, z, 'r--^', label=f'Flight {fid}')
        
        # Highlight conflicts
        for conflict in conflicts:
            if abs(conflict['time'] - frame) < 0.5:
                x, y, z = conflict['location']
                ax.scatter(x, y, z, s=200, c='red', marker='*')
                ax.text(x, y, z, f"Conflict with {conflict['conflict_flight']}", fontsize=8)
        
        ax.set_xlim(0, 100)
        ax.set_ylim(0, 100)
        ax.set_zlim(0, 50)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Altitude (m)')
        ax.set_title(f'UAV Traffic at t={frame:.1f}s')
        return ax, time_text

    ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 20, 0.5), blit=False)
    ani.save('conflict_animation.mp4', fps=2)
    return ani

# USER INPUT FOR PRIMARY DRONE MISSION
def get_primary_mission():
    primary_mission = []
    
    print("\n" + "="*50)
    print("Primary Mission Configuration")
    print("="*50)
    
    num_points = int(input("Enter number of mission waypoints: "))
    
    print("\nEnter coordinates for each waypoint:")
    for i in range(num_points):
        print(f"\nWaypoint {i+1}:")
        x = float(input("  X coordinate: "))
        y = float(input("  Y coordinate: "))
        z = float(input("  Z coordinate: "))
        primary_mission.append((x, y, z))
    
    # Get time window
    print("\nMission Time Window:")
    start_time = float(input("  Start time (seconds): "))
    end_time = float(input("  End time (seconds): "))
    
    # Convert to Waypoint objects with distributed times
    waypoint_objects = []
    for i, (x, y, z) in enumerate(primary_mission):
        # Distribute time evenly across waypoints
        t = start_time + (end_time - start_time) * (i / (num_points - 1)) if num_points > 1 else start_time
        waypoint_objects.append(Waypoint(x, y, z, t))
    
    return waypoint_objects

# Example Usage
if __name__ == "__main__":
    # Create deconfliction engine
    engine = DeconflictionEngine(buffer=8.0)
    
    # Get primary mission from user
    primary_mission = get_primary_mission()
    
    # Define simulated flights (could also be loaded from file)
    flight1 = FlightPlan("F1", [
        Waypoint(90, 10, 20, 0),
        Waypoint(50, 50, 30, 8),
        Waypoint(10, 90, 20, 16)
    ])
    
    flight2 = FlightPlan("F2", [
        Waypoint(20, 80, 25, 2),
        Waypoint(60, 60, 15, 10),
        Waypoint(80, 20, 25, 18)
    ])
    
    engine.add_flight(flight1)
    engine.add_flight(flight2)
    
    # Check for conflicts
    conflicts = engine.check_mission(primary_mission)
    
    # Visualization
    plot_3d_trajectories(primary_mission, 
                         {"F1": flight1, "F2": flight2}, 
                         conflicts)
    
    if conflicts:
        animate_conflicts(primary_mission, 
                          {"F1": flight1, "F2": flight2}, 
                          conflicts)
        print(f"\nConflict detected at:")
        for c in conflicts:
            loc = c['location']
            print(f"- Time {c['time']:.1f}s: Position {loc} with {c['conflict_flight']} (Distance: {c['distance']:.2f}m)")
    else:
        print("\nNo conflicts detected - Mission is safe!")

# 10,10,10
# 50,50,30
# 90,90,10
