"""
Real-time 3D Visualization for RPLIDAR A2M12 - Stable Window Version
====================================================================
Fixed window management for continuous display.

Requirements:
- pip install rplidar-roboticia pyserial matplotlib numpy
- Run as Administrator on Windows if permission errors occur
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Force TkAgg backend for better stability
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import time
import threading
import queue
import sys
import os
from rplidar import RPLidar
import serial.tools.list_ports
import warnings
import signal
warnings.filterwarnings('ignore')

# Configuration
PORT_NAME = 'COM3'  # Update this based on your system
BAUD_RATE = 256000
MAX_POINTS_DISPLAY = 8000  # Maximum points to display at once
HISTORY_SIZE = 10  # Number of scans to keep in memory for trail effect
UPDATE_RATE = 30  # FPS for visualization (reduced for stability)
Z_LAYERS = 5  # Number of Z layers for 3D effect
Z_SPACING = 50  # Vertical spacing between layers (mm)
MAX_DISTANCE = 6000  # Maximum distance to display (mm)
MIN_DISTANCE = 150  # Minimum distance to display (mm)
RECONNECT_DELAY = 3  # Seconds to wait before reconnecting

class LidarVisualizer3D:
    def __init__(self, port_name=PORT_NAME, baudrate=BAUD_RATE):
        self.port_name = port_name
        self.baudrate = baudrate
        self.lidar = None
        self.scan_queue = queue.Queue(maxsize=50)
        self.scan_thread = None
        self.running = False
        self.connected = False
        self.total_scans = 0
        self.start_time = time.time()
        self.window_closed = False
        
        # Data storage
        self.point_history = deque(maxlen=HISTORY_SIZE)
        self.current_points = np.array([]).reshape(0, 3)
        self.all_points = []
        
        # Animation object
        self.ani = None
        
    def setup_plot(self):
        """Initialize the 3D plot with interactive features."""
        print("📊 Setting up visualization window...")
        
        # Create figure
        self.fig = plt.figure(figsize=(12, 9))
        
        # Set window title
        self.fig.canvas.manager.set_window_title('RPLIDAR A2M12 - Live 3D Mapping')
        
        # Handle window close event
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
        # Create 3D subplot
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Configure plot aesthetics
        self.ax.set_xlabel('X Distance (mm)', color='cyan', fontsize=10)
        self.ax.set_ylabel('Y Distance (mm)', color='cyan', fontsize=10)
        self.ax.set_zlabel('Z Layer', color='cyan', fontsize=10)
        self.ax.set_title('RPLIDAR A2M12 - Real-time 3D Environment Map', 
                         color='white', fontsize=14, fontweight='bold', pad=20)
        
        # Set viewing limits
        self.ax.set_xlim([-MAX_DISTANCE, MAX_DISTANCE])
        self.ax.set_ylim([-MAX_DISTANCE, MAX_DISTANCE])
        self.ax.set_zlim([0, Z_LAYERS * Z_SPACING])
        
        # Configure grid
        self.ax.grid(True, alpha=0.2, linewidth=0.5)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        
        # Initialize empty scatter plots
        self.scatters = []
        colors = ['red', 'orange', 'yellow', 'lime', 'cyan', 'blue', 'purple']
        sizes = np.linspace(20, 3, HISTORY_SIZE)
        
        for i in range(min(HISTORY_SIZE, len(colors))):
            color_idx = i % len(colors)
            scatter = self.ax.scatter([], [], [], 
                                     c=colors[color_idx], 
                                     s=sizes[i], 
                                     alpha=0.7 - i*0.05,
                                     marker='o',
                                     edgecolors='none')
            self.scatters.append(scatter)
        
        # Add LIDAR position marker
        self.ax.scatter([0], [0], [0], c='white', s=150, marker='*', 
                       edgecolors='red', linewidth=2, label='LIDAR Position', zorder=1000)
        
        # Add text elements
        self.status_text = self.ax.text2D(0.02, 0.98, 'Initializing...', 
                                         transform=self.ax.transAxes,
                                         color='lime', 
                                         fontsize=9,
                                         verticalalignment='top',
                                         fontfamily='monospace',
                                         bbox=dict(boxstyle="round,pad=0.3", 
                                                 facecolor='black', 
                                                 alpha=0.7))
        
        self.info_text = self.ax.text2D(0.02, 0.85, '', 
                                       transform=self.ax.transAxes,
                                       color='white', 
                                       fontsize=8,
                                       verticalalignment='top',
                                       fontfamily='monospace',
                                       bbox=dict(boxstyle="round,pad=0.3", 
                                               facecolor='black', 
                                               alpha=0.7))
        
        # Add distance reference rings
        self.add_distance_rings()
        
        # Add legend
        self.ax.legend(loc='upper right', fontsize=8, framealpha=0.8)
        
        # Set initial view angle
        self.ax.view_init(elev=25, azim=45)
        
        # Apply tight layout
        plt.tight_layout()
        
        print("✅ Visualization window ready!")
        
    def add_distance_rings(self):
        """Add circular distance reference rings."""
        theta = np.linspace(0, 2*np.pi, 50)
        for radius in [1000, 2000, 3000, 4000, 5000]:
            x_ring = radius * np.cos(theta)
            y_ring = radius * np.sin(theta)
            z_ring = np.zeros_like(x_ring)
            self.ax.plot(x_ring, y_ring, z_ring, 
                        color='gray', alpha=0.15, linewidth=0.5, linestyle='--')
            # Add labels
            self.ax.text(radius, 0, 0, f'{radius/1000:.0f}m', 
                        color='gray', fontsize=7, alpha=0.4)
    
    def on_close(self, event):
        """Handle window close event."""
        print("\n🔴 Visualization window closed by user")
        self.window_closed = True
        self.running = False
    
    def connect_lidar(self, retry=True):
        """Connect to the RPLIDAR sensor."""
        max_retries = 3 if retry else 1
        retry_count = 0
        
        while retry_count < max_retries and not self.connected:
            try:
                print(f"\n🔄 Connection attempt {retry_count + 1}/{max_retries}")
                print(f"   Port: {self.port_name}")
                print(f"   Baudrate: {self.baudrate}")
                
                # Clean up any existing connection
                if self.lidar:
                    try:
                        self.lidar.stop()
                        self.lidar.disconnect()
                    except:
                        pass
                    self.lidar = None
                
                time.sleep(1)
                
                # Create new connection
                self.lidar = RPLidar(self.port_name, baudrate=self.baudrate, timeout=3)
                
                # Get device info
                info = self.lidar.get_info()
                print(f"\n✅ Connected Successfully!")
                print(f"   Model: {info.get('model', 'Unknown')}")
                print(f"   Firmware: {info.get('firmware', 'Unknown')}")
                
                # Check health
                health = self.lidar.get_health()
                print(f"   Health Status: {health[0]}")
                
                self.connected = True
                return True
                    
            except PermissionError as e:
                print(f"\n❌ PERMISSION ERROR: {e}")
                print("\n💡 Try these solutions:")
                print("   1. Close this window and run as Administrator")
                print("   2. Unplug and replug the LIDAR")
                print("   3. Check if another program is using COM3")
                
                # List ports
                self.list_available_ports()
                
                retry_count += 1
                if retry_count < max_retries:
                    print(f"\n⏳ Retrying in {RECONNECT_DELAY} seconds...")
                    time.sleep(RECONNECT_DELAY)
                
            except Exception as e:
                print(f"\n❌ Connection failed: {e}")
                retry_count += 1
                if retry_count < max_retries:
                    print(f"\n⏳ Retrying in {RECONNECT_DELAY} seconds...")
                    time.sleep(RECONNECT_DELAY)
        
        return False
    
    def list_available_ports(self):
        """List available serial ports."""
        print("\n📡 Available Serial Ports:")
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("   No ports found")
            return []
        
        for port, desc, hwid in sorted(ports):
            print(f"   • {port}: {desc}")
        return [p[0] for p in ports]
    
    def scan_worker(self):
        """Worker thread for continuous scanning."""
        print("\n🔄 Starting scan worker thread...")
        
        while self.running:
            try:
                if not self.connected:
                    print("📡 Attempting to reconnect to LIDAR...")
                    if not self.connect_lidar(retry=True):
                        time.sleep(RECONNECT_DELAY)
                        continue
                
                # Start scanning
                print("📊 Starting scan iterations...")
                for scan in self.lidar.iter_scans():
                    if not self.running:
                        break
                    
                    # Filter measurements
                    valid_scan = []
                    for quality, angle, dist in scan:
                        if MIN_DISTANCE < dist < MAX_DISTANCE and quality > 10:
                            valid_scan.append((angle, dist))
                    
                    if valid_scan:
                        self.total_scans += 1
                        # Add to queue
                        try:
                            self.scan_queue.put(valid_scan, timeout=0.01)
                        except queue.Full:
                            # Clear old data
                            try:
                                self.scan_queue.get_nowait()
                                self.scan_queue.put(valid_scan, timeout=0.01)
                            except:
                                pass
                            
            except Exception as e:
                print(f"⚠️ Scan error: {e}")
                self.connected = False
                time.sleep(1)
        
        print("🔴 Scan worker thread stopped")
    
    def process_scan_data(self, scan):
        """Convert scan to 3D points."""
        if not scan:
            return np.array([]).reshape(0, 3)
        
        points_3d = []
        
        for z_layer in range(Z_LAYERS):
            z_value = z_layer * Z_SPACING
            
            for angle, distance in scan:
                if z_layer == 0 or (z_layer < Z_LAYERS * (1 - distance/MAX_DISTANCE)):
                    angle_rad = np.radians(angle)
                    x = distance * np.cos(angle_rad)
                    y = distance * np.sin(angle_rad)
                    z = z_value + np.random.uniform(-10, 10)
                    points_3d.append([x, y, z])
        
        return np.array(points_3d) if points_3d else np.array([]).reshape(0, 3)
    
    def update_plot(self, frame):
        """Animation update function."""
        if self.window_closed or not self.running:
            return self.scatters + [self.status_text, self.info_text]
        
        try:
            # Get latest scan
            scan_data = None
            while not self.scan_queue.empty():
                try:
                    scan_data = self.scan_queue.get_nowait()
                except:
                    break
            
            if scan_data:
                # Process to 3D
                new_points = self.process_scan_data(scan_data)
                
                if len(new_points) > 0:
                    # Update history
                    self.point_history.append(new_points)
                    
                    # Update scatter plots
                    for i, points in enumerate(self.point_history):
                        if i < len(self.scatters) and len(points) > 0:
                            # Subsample if needed
                            max_pts = MAX_POINTS_DISPLAY // max(1, len(self.point_history))
                            if len(points) > max_pts:
                                idx = np.random.choice(len(points), max_pts, replace=False)
                                points = points[idx]
                            
                            self.scatters[i]._offsets3d = (points[:, 0], 
                                                          points[:, 1], 
                                                          points[:, 2])
                    
                    # Update stats
                    num_points = sum(len(p) for p in self.point_history)
                    distances = np.linalg.norm(new_points[:, :2], axis=1)
                    closest = np.min(distances) if len(distances) > 0 else 0
                    
                    # Update text
                    runtime = int(time.time() - self.start_time)
                    status = "🟢 CONNECTED" if self.connected else "🔴 DISCONNECTED"
                    self.status_text.set_text(f"Status: {status} | Time: {runtime}s | Scans: {self.total_scans}")
                    
                    self.info_text.set_text(
                        f"Points: {num_points:,}\n"
                        f"Closest: {closest:.0f}mm\n"
                        f"Queue: {self.scan_queue.qsize()}"
                    )
                    
                    # Rotate view slowly
                    if frame % 3 == 0:
                        self.ax.view_init(elev=25, azim=self.ax.azim + 0.3)
            
            # Update status periodically even without new data
            elif frame % 30 == 0:
                runtime = int(time.time() - self.start_time)
                status = "🟢 CONNECTED" if self.connected else "🔴 WAITING"
                self.status_text.set_text(f"Status: {status} | Time: {runtime}s | Scans: {self.total_scans}")
            
        except Exception as e:
            print(f"Update error: {e}")
        
        return self.scatters + [self.status_text, self.info_text]
    
    def run(self):
        """Main visualization loop."""
        print("\n" + "="*60)
        print(" RPLIDAR A2M12 - 3D VISUALIZATION ".center(60))
        print("="*60)
        
        # Setup plot first
        self.setup_plot()
        
        # Initial connection
        if not self.connect_lidar(retry=True):
            print("\n⚠️ Initial connection failed, but continuing...")
        
        # Start scan thread
        self.running = True
        self.scan_thread = threading.Thread(target=self.scan_worker, daemon=True)
        self.scan_thread.start()
        
        print("\n🚀 Visualization started!")
        print("📌 Controls: Drag to rotate, Scroll to zoom")
        print("🛑 Close window or press Ctrl+C to stop\n")
        
        try:
            # Create animation
            self.ani = FuncAnimation(
                self.fig, 
                self.update_plot,
                interval=1000//UPDATE_RATE,
                blit=False,
                repeat=True,
                cache_frame_data=False
            )
            
            # Show plot - this will block until window is closed
            plt.show(block=True)
            
        except KeyboardInterrupt:
            print("\n⚫ Interrupted by user")
        except Exception as e:
            print(f"\n❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown."""
        print("\n🧹 Cleaning up...")
        
        # Stop running
        self.running = False
        
        # Stop animation
        if self.ani:
            self.ani.event_source.stop()
        
        # Stop scan thread
        if self.scan_thread and self.scan_thread.is_alive():
            print("   Stopping scan thread...")
            self.scan_thread.join(timeout=2)
        
        # Disconnect LIDAR
        if self.lidar:
            try:
                print("   Disconnecting LIDAR...")
                self.lidar.stop()
                self.lidar.disconnect()
            except:
                pass
        
        # Close plots
        plt.close('all')
        
        print("✅ Cleanup complete!")

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully."""
    print("\n\n🛑 Received interrupt signal")
    sys.exit(0)

def main():
    """Main entry point."""
    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Check admin on Windows
        if sys.platform == 'win32':
            import ctypes
            if not ctypes.windll.shell32.IsUserAnAdmin():
                print("\n⚠️  WARNING: Not running as Administrator!")
                print("   You may encounter permission errors.")
                print("   Right-click your terminal and 'Run as Administrator'\n")
                time.sleep(2)
        
        # Create and run visualizer
        visualizer = LidarVisualizer3D()
        visualizer.run()
        
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n👋 Goodbye!")
        # Ensure everything closes
        plt.close('all')

if __name__ == '__main__':
    main()
