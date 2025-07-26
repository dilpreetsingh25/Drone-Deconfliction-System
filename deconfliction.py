import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, FFMpegWriter
from datetime import datetime, timedelta
import math
import json
import os
import sys
from typing import List, Dict, Tuple, Optional

class VectorMath:
    """Utility class for vector operations"""
    @staticmethod
    def distance(a: np.ndarray, b: np.ndarray) -> float:
        """Calculate Euclidean distance between two points"""
        return np.linalg.norm(a - b)
    
    @staticmethod
    def interpolate(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
        """Linear interpolation between two points"""
        return start + fraction * (end - start)
    
    @staticmethod
    def midpoint(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        """Calculate midpoint between two points"""
        return (a + b) / 2

class Waypoint:
    """Represents a point in 4D space (x, y, z, time)"""
    def __init__(self, x: float, y: float, z: float, t: Optional[datetime] = None):
        self.position = np.array([x, y, z], dtype=float)
        self.t = t
        
    def __repr__(self) -> str:
        return f"WP({self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f}, {self.t})"

class FlightPath:
    """Represents a drone's flight path with timed waypoints"""
    def __init__(self, drone_id: str, waypoints: List[Waypoint]):
        self.drone_id = drone_id
        self.waypoints = waypoints
        self.segments = self._create_segments()
        self.min_time, self.max_time = self._calculate_time_range()
        
    def _create_segments(self) -> List[Tuple[Waypoint, Waypoint]]:
        """Create segments between consecutive waypoints"""
        segments = []
        for i in range(len(self.waypoints)-1):
            segments.append((self.waypoints[i], self.waypoints[i+1]))
        return segments
    
    def _calculate_time_range(self) -> Tuple[datetime, datetime]:
        """Calculate the start and end time of the flight"""
        if not self.waypoints or not self.waypoints[0].t:
            return datetime.min, datetime.max
        min_time = min(wp.t for wp in self.waypoints)
        max_time = max(wp.t for wp in self.waypoints)
        return min_time, max_time
    
    def position_at_time(self, t: datetime) -> Optional[np.ndarray]:
        """Get position at a specific time using linear interpolation"""
        for start, end in self.segments:
            if start.t <= t <= end.t:
                total_duration = (end.t - start.t).total_seconds()
                if total_duration <= 0:
                    return start.position.copy()
                elapsed = (t - start.t).total_seconds()
                fraction = elapsed / total_duration
                return VectorMath.interpolate(start.position, end.position, fraction)
        return None
    
    def get_all_positions(self) -> np.ndarray:
        """Get all positions as a numpy array"""
        return np.array([wp.position for wp in self.waypoints])
    
    def get_time_range_seconds(self) -> Tuple[float, float]:
        """Get time range in seconds since epoch"""
        return self.min_time.timestamp(), self.max_time.timestamp()
    
    def get_bounding_box(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get the spatial bounding box of the flight path"""
        positions = self.get_all_positions()
        return np.min(positions, axis=0), np.max(positions, axis=0)

class ConflictResult:
    """Stores the results of a conflict check"""
    def __init__(self, status: str, conflicts: Optional[List[Dict]] = None):
        self.status = status  # "clear" or "conflict"
        self.conflicts = conflicts or []
    
    def add_conflict(self, time: datetime, location: np.ndarray, 
                    drone_id: str, min_distance: float) -> None:
        """Add a conflict to the result"""
        self.conflicts.append({
            "time": time,
            "location": location.copy(),
            "drone_id": drone_id,
            "min_distance": min_distance
        })
        
    def __repr__(self) -> str:
        if self.status == "clear":
            return "Mission Clear: No conflicts detected"
        else:
            return f"Conflict Detected: {len(self.conflicts)} conflict(s) found"
    
    def detailed_report(self) -> str:
        """Generate a detailed conflict report"""
        if not self.conflicts:
            return "No conflicts detected"
        
        report = "Conflict Report:\n"
        report += "="*50 + "\n"
        for i, conflict in enumerate(self.conflicts, 1):
            report += f"Conflict {i}:\n"
            report += f"  Time: {conflict['time']}\n"
            loc = conflict['location']
            report += f"  Location: ({loc[0]:.2f}, {loc[1]:.2f}, {loc[2]:.2f})\n"
            report += f"  With Drone: {conflict['drone_id']}\n"
            report += f"  Minimum Distance: {conflict['min_distance']:.2f}m\n"
            report += "-"*50 + "\n"
        return report

class DeconflictionEngine:
    """Core deconfliction engine for detecting spatial and temporal conflicts"""
    def __init__(self, safety_buffer: float = 5.0):
        self.safety_buffer = safety_buffer
        self.safety_buffer_sq = safety_buffer ** 2
        self.simulated_flights = []
    
    def add_simulated_flight(self, flight_path: FlightPath) -> None:
        """Add a simulated flight to the system"""
        self.simulated_flights.append(flight_path)
        
    def load_flights_from_file(self, filename: str) -> None:
        """Load simulated flights from a JSON file"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
                for flight_data in data:
                    waypoints = []
                    for wp in flight_data['waypoints']:
                        dt = datetime.strptime(wp['t'], "%Y-%m-%d %H:%M:%S")
                        waypoints.append(Waypoint(wp['x'], wp['y'], wp['z'], dt))
                    self.add_simulated_flight(FlightPath(flight_data['id'], waypoints))
            print(f"Loaded {len(self.simulated_flights)} simulated flights from {filename}")
        except Exception as e:
            print(f"Error loading flights: {str(e)}")
    
    def check_mission(self, primary_waypoints: List[Tuple[float, float, float]], 
                     t_start: datetime, t_end: datetime) -> Tuple[ConflictResult, FlightPath]:
        """
        Check a primary mission against all simulated flights
        
        Args:
            primary_waypoints: List of (x, y, z) tuples defining the mission
            t_start: Start time of the mission
            t_end: End time of the mission
            
        Returns:
            ConflictResult: Result of the conflict check
            FlightPath: The timed primary flight path
        """
        primary_path = self._create_timed_primary_path(primary_waypoints, t_start, t_end)
        primary_flight = FlightPath("primary", primary_path)
        result = ConflictResult("clear")
        
        # Check against all simulated flights
        for sim_flight in self.simulated_flights:
            self._check_flight_conflict(primary_flight, sim_flight, result)
        
        if result.conflicts:
            result.status = "conflict"
        
        return result, primary_flight
    
    def _create_timed_primary_path(self, waypoints: List[Tuple[float, float, float]], 
                                  t_start: datetime, t_end: datetime) -> List[Waypoint]:
        """Assign timestamps to primary mission waypoints based on proportional distance"""
        if not waypoints:
            return []
        
        # Calculate total distance
        total_dist = 0.0
        positions = [np.array(wp) for wp in waypoints]
        for i in range(1, len(positions)):
            total_dist += VectorMath.distance(positions[i-1], positions[i])
        
        # Create timed waypoints
        timed_waypoints = [Waypoint(*waypoints[0], t_start)]
        current_dist = 0.0
        
        for i in range(1, len(waypoints)):
            seg_dist = VectorMath.distance(positions[i-1], positions[i])
            current_dist += seg_dist
            fraction = current_dist / total_dist if total_dist > 0 else 0
            time_offset = timedelta(seconds=fraction * (t_end - t_start).total_seconds())
            t_point = t_start + time_offset
            timed_waypoints.append(Waypoint(*waypoints[i], t_point))
        
        return timed_waypoints
    
    def _check_flight_conflict(self, primary_flight: FlightPath, 
                              sim_flight: FlightPath, 
                              result: ConflictResult) -> None:
        """Check for conflicts between primary and one simulated flight"""
        for p_seg in primary_flight.segments:
            p_start, p_end = p_seg
            if not p_start.t or not p_end.t:
                continue
                
            for s_seg in sim_flight.segments:
                s_start, s_end = s_seg
                if not s_start.t or not s_end.t:
                    continue
                
                # Check time overlap
                t_overlap_start = max(p_start.t, s_start.t)
                t_overlap_end = min(p_end.t, s_end.t)
                if t_overlap_start > t_overlap_end:
                    continue
                
                # Calculate velocities
                dt_p = (p_end.t - p_start.t).total_seconds()
                dt_s = (s_end.t - s_start.t).total_seconds()
                
                if dt_p <= 0 or dt_s <= 0:
                    continue
                
                # Calculate relative motion
                Va = (p_end.position - p_start.position) / dt_p
                Vb = (s_end.position - s_start.position) / dt_s
                Vr = Va - Vb
                
                # Position vectors
                P0 = p_start.position
                S0 = s_start.position
                
                # Time offset vectors
                Tp = p_start.t.timestamp()
                Ts = s_start.t.timestamp()
                
                # Relative position vector
                D0 = P0 - S0 - Va*Tp + Vb*Ts
                
                # Quadratic coefficients for distance minimization
                a = np.dot(Vr, Vr)
                b = 2 * np.dot(D0, Vr)
                c = np.dot(D0, D0)
                
                # Time candidates for evaluation
                t_candidates = [t_overlap_start.timestamp(), t_overlap_end.timestamp()]
                
                # Add vertex if within time range
                if a > 1e-10:  # Avoid division by zero
                    t_min = -b / (2 * a)
                    if t_overlap_start.timestamp() <= t_min <= t_overlap_end.timestamp():
                        t_candidates.append(t_min)
                
                # Evaluate distance at candidate times
                min_dist_sq = float('inf')
                min_t = None
                for t_cand in t_candidates:
                    # Position at time t_cand
                    P_t = P0 + Va * (t_cand - Tp)
                    S_t = S0 + Vb * (t_cand - Ts)
                    dist_sq = np.dot(P_t - S_t, P_t - S_t)
                    
                    if dist_sq < min_dist_sq:
                        min_dist_sq = dist_sq
                        min_t = t_cand
                
                # Check if within safety buffer
                if min_dist_sq < self.safety_buffer_sq:
                    conflict_time = datetime.fromtimestamp(min_t)
                    primary_pos = primary_flight.position_at_time(conflict_time)
                    if primary_pos is not None:
                        min_dist = math.sqrt(min_dist_sq)
                        result.add_conflict(
                            time=conflict_time,
                            location=primary_pos,
                            drone_id=sim_flight.drone_id,
                            min_distance=min_dist
                        )

class ConflictVisualizer:
    """Visualization tools for drone flight paths and conflicts"""
    @staticmethod
    def plot_3d_paths(primary_flight: FlightPath, 
                     simulated_flights: List[FlightPath], 
                     conflicts: List[Dict] = None,
                     save_path: str = "flight_paths.png") -> None:
        """Create a 3D plot of all flight paths with conflicts highlighted"""
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot primary path
        primary_positions = primary_flight.get_all_positions()
        ax.plot(primary_positions[:, 0], primary_positions[:, 1], primary_positions[:, 2], 
               'b-o', linewidth=3, markersize=6, label='Primary Drone')
        
        # Plot simulated paths
        colors = plt.cm.tab10(np.linspace(0, 1, len(simulated_flights)))
        for i, flight in enumerate(simulated_flights):
            positions = flight.get_all_positions()
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], '--', 
                    color=colors[i], linewidth=1.5, alpha=0.7, label=f'Drone {flight.drone_id}')
        
        # Plot conflicts
        if conflicts:
            conflict_points = np.array([c['location'] for c in conflicts])
            ax.scatter(conflict_points[:, 0], conflict_points[:, 1], conflict_points[:, 2], 
                      c='red', s=120, marker='X', label='Conflict Points')
            
            # Add conflict labels
            for i, conflict in enumerate(conflicts):
                loc = conflict['location']
                ax.text(loc[0], loc[1], loc[2] + 5, 
                        f"Conflict {i+1}\nDrone: {conflict['drone_id']}\nDist: {conflict['min_distance']:.1f}m",
                        fontsize=8, color='darkred')
        
        # Set labels and title
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_zlabel('Altitude (m)', fontsize=12)
        ax.set_title('Drone Flight Paths and Conflicts', fontsize=14)
        ax.legend(loc='upper left', fontsize=10)
        
        # Add grid and adjust view
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('w')
        ax.yaxis.pane.set_edgecolor('w')
        ax.zaxis.pane.set_edgecolor('w')
        ax.view_init(elev=25, azim=45)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        print(f"Saved 3D plot to {save_path}")
        plt.show()
    
    @staticmethod
    def animate_4d_paths(primary_flight: FlightPath, 
                        simulated_flights: List[FlightPath], 
                        conflicts: List[Dict] = None,
                        filename: str = 'drone_animation.mp4',
                        fps: int = 10) -> None:
        """Create a 4D animation (3D space + time) of drone flights"""
        # Find global time range
        all_flights = [primary_flight] + simulated_flights
        min_time = min(flight.min_time for flight in all_flights)
        max_time = max(flight.max_time for flight in all_flights)
        total_seconds = (max_time - min_time).total_seconds()
        
        # Create figure
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Initialize elements
        primary_line, = ax.plot([], [], [], 'b-o', linewidth=3, markersize=6, 
                               label='Primary Drone', alpha=0.8)
        sim_lines = []
        time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12)
        conflict_scatter = ax.scatter([], [], [], c='red', s=120, marker='X', 
                                     label='Conflict', alpha=0.9, depthshade=False)
        
        # Create drone markers
        primary_marker = ax.scatter([], [], [], c='blue', s=80, marker='o')
        sim_markers = []
        colors = plt.cm.tab10(np.linspace(0, 1, len(simulated_flights)))
        for i, flight in enumerate(simulated_flights):
            marker = ax.scatter([], [], [], c=colors[i], s=60, marker='^')
            sim_markers.append(marker)
        
        # Set up plot limits
        all_positions = np.vstack([flight.get_all_positions() for flight in all_flights])
        min_vals = all_positions.min(axis=0) - 20
        max_vals = all_positions.max(axis=0) + 20
        
        ax.set_xlim(min_vals[0], max_vals[0])
        ax.set_ylim(min_vals[1], max_vals[1])
        ax.set_zlim(min(0, min_vals[2]), max_vals[2] + 20)
        
        # Set labels and title
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_zlabel('Altitude (m)', fontsize=12)
        ax.set_title('4D Drone Flight Animation', fontsize=14)
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, linestyle='--', alpha=0.6)
        
        # Animation update function
        def update(frame):
            current_time = min_time + timedelta(seconds=frame * total_seconds / 100)
            time_text.set_text(f'Time: {current_time.strftime("%H:%M:%S")}')
            
            # Update primary drone
            primary_pos = primary_flight.position_at_time(current_time)
            if primary_pos is not None:
                # Update path
                xs = [wp.position[0] for wp in primary_flight.waypoints if wp.t <= current_time]
                ys = [wp.position[1] for wp in primary_flight.waypoints if wp.t <= current_time]
                zs = [wp.position[2] for wp in primary_flight.waypoints if wp.t <= current_time]
                primary_line.set_data(xs, ys)
                primary_line.set_3d_properties(zs)
                primary_marker._offsets3d = ([primary_pos[0]], [primary_pos[1]], [primary_pos[2]])
            
            # Update simulated drones
            for i, flight in enumerate(simulated_flights):
                sim_pos = flight.position_at_time(current_time)
                if sim_pos is not None:
                    # Update path
                    xs = [wp.position[0] for wp in flight.waypoints if wp.t <= current_time]
                    ys = [wp.position[1] for wp in flight.waypoints if wp.t <= current_time]
                    zs = [wp.position[2] for wp in flight.waypoints if wp.t <= current_time]
                    
                    if i < len(sim_lines):
                        sim_lines[i].set_data(xs, ys)
                        sim_lines[i].set_3d_properties(zs)
                    else:
                        line, = ax.plot(xs, ys, zs, '--', linewidth=1.5, alpha=0.7,
                                      color=colors[i])
                        sim_lines.append(line)
                    
                    # Update marker
                    sim_markers[i]._offsets3d = ([sim_pos[0]], [sim_pos[1]], [sim_pos[2]])
            
            # Update conflicts
            conflict_points = []
            if conflicts:
                for conflict in conflicts:
                    if abs((conflict['time'] - current_time).total_seconds()) < 1:
                        conflict_points.append(conflict['location'])
            
            if conflict_points:
                conflict_points = np.array(conflict_points)
                conflict_scatter._offsets3d = (conflict_points[:, 0], 
                                             conflict_points[:, 1], 
                                             conflict_points[:, 2])
            else:
                conflict_scatter._offsets3d = ([], [], [])
            
            return primary_line, *sim_lines, primary_marker, *sim_markers, conflict_scatter, time_text
        
        # Create animation
        anim = FuncAnimation(fig, update, frames=100, interval=1000//fps, blit=True)
        
        # Save animation
        try:
            writer = FFMpegWriter(fps=fps, bitrate=1800)
            anim.save(filename, writer=writer)
            print(f"Saved 4D animation to {filename}")
        except Exception as e:
            print(f"Error saving animation: {str(e)}")
            plt.close()
            return
        
        plt.close()

def run_simulation():
    """Main function to run the deconfliction system"""
    print("="*50)
    print("Drone Deconfliction System")
    print("="*50)
    
    # Configuration
    SAFETY_BUFFER = 10.0  # meters
    
    # Create simulated flights (hardcoded)
    now = datetime.now()
    conflict_time = now + timedelta(minutes=5)
    simulated_flights = [
        FlightPath("drone1", [
            Waypoint(50, 100, 30, now + timedelta(minutes=3)),
            Waypoint(150, 200, 50, conflict_time),
            Waypoint(250, 150, 40, now + timedelta(minutes=7))
        ]),
        FlightPath("drone2", [
            Waypoint(200, 100, 20, now + timedelta(minutes=2)),
            Waypoint(100, 200, 60, conflict_time),
            Waypoint(300, 300, 30, now + timedelta(minutes=8))
        ]),
        FlightPath("drone3", [
            Waypoint(0, 300, 40, now + timedelta(minutes=3)),
            Waypoint(150, 100, 60, now + timedelta(minutes=6)),
            Waypoint(300, 0, 20, now + timedelta(minutes=9))
        ])
    ]
    
    print(f"Created {len(simulated_flights)} simulated flights in memory")
    
    # Get primary mission from user
    primary_mission = [
    # (0, 0, 0),        # Start

    # (312.5, 437.5, 112.5),# Will conflict with drone1 and drone2

    # (150, 150, 50),   
    # (300, 0, 20)      # Will conflict with drone3
    
    # duration = 9
    ]
    
    print("\n" + "="*50)
    print("Primary Mission Configuration")
    print("="*50)
    
    num_points = int(input("Enter the number of mission waypoints: "))
    
    for i in range(num_points):
        print(f"\nWaypoint {i+1}:")
        x = float(input("  X coordinate : "))
        y = float(input("  Y coordinate : "))
        z = float(input("  Z coordinate : "))
        primary_mission.append((x, y, z))

    # Get time window
    print("\nEnter mission time window")
    start_time = datetime.now()
    print(f"  Start time will be current time: {start_time}")
    
    duration = float(input("Enter mission duration in minutes: "))
    end_time = start_time + timedelta(minutes=duration)
    print(f"  End time: {end_time}")
    
    # Initialize deconfliction engine
    engine = DeconflictionEngine(safety_buffer=SAFETY_BUFFER)
    
    # Add simulated flights to engine
    for flight in simulated_flights:
        engine.add_simulated_flight(flight)
    
    # Check for conflicts
    result, primary_flight = engine.check_mission(primary_mission, start_time, end_time)
    
    # Display results
    print("\n" + "="*50)
    print("Deconfliction Results")
    print("="*50)
    print(result)
    
    if result.status == "conflict":
        print("\n" + result.detailed_report())
    
    # Visualization
    print("\n" + "="*50)
    print("Generating Visualizations")
    print("="*50)
    
    # Generate 3D plot
    plot_file = "drone_paths.png"
    ConflictVisualizer.plot_3d_paths(primary_flight, simulated_flights, 
                                    result.conflicts, plot_file)
    
    # Generate 4D animation
    anim_file = "drone_animation.mp4"
    fps = int(input("Enter animation frame rate (FPS, 5-30): ") or 10)
    ConflictVisualizer.animate_4d_paths(primary_flight, simulated_flights, 
                                       result.conflicts, anim_file, fps)
    
    print("\nSimulation complete!")
    print(f"- 3D plot saved to: {plot_file}")
    print(f"- 4D animation saved to: {anim_file}")

if __name__ == "__main__":
    # Check if FFmpeg is available for animation
    try:
        from matplotlib.animation import FFMpegWriter
    except ImportError:
        print("FFmpeg not found. Animations will not be saved.")
    
    run_simulation()
