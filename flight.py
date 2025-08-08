###################################
# IMPORTS

# Imports for crazyflie (the drone)
import logging
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import pandas as pd
import ast
from multiprocessing import SimpleQueue

# Imports for qualisys (the motion capture system)
import asyncio
import xml.etree.cElementTree as ET
from threading import Thread
import qtm_rt as qtm
from scipy.spatial.transform import Rotation

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Function to load data from the Excel file and process it
def load_and_process_csv(file_path):
    # Load the data from the CSV file
    df = pd.read_csv(file_path)

    # Display the loaded DataFrame to check
    print('Loaded DataFrame:')
    print(df.head())

    # Convert columns to numpy arrays
    times = df['Time (s)'].to_numpy()
    x_coords = df['X (ft)'].to_numpy()
    y_coords = df['Y (ft)'].to_numpy()
    velocity = df['Velocity (ft/s)'].to_numpy()

    # Return the processed arrays
    return times, x_coords, y_coords, velocity

###################################
# PARAMETERS

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/105/2M/E7E7E7E7E7' # <-- FIXME

# Specify the variables you want to log at 100 Hz from the drone
variables = [
    # State estimates (custom observer)
    'ae483log.p_x',
    'ae483log.p_y',
    'ae483log.p_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # State estimates (default observer)
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Desired position (custom controller)
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Desired position (default controller)
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
    # Mocap
    'ae483log.p_x_mocap',
    'ae483log.p_y_mocap',
    'ae483log.p_z_mocap',
    'ae483log.psi_mocap',
    'ae483log.theta_mocap',
    'ae483log.phi_mocap',
]

# Specify the IP address of the motion capture system
ip_address = '128.174.245.190'

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_50' # <-- FIXME

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [51, 52, 53, 54]


###################################
# CLIENT FOR CRAZYFLIE

class CrazyflieClient:
    def __init__(self, uri, use_controller=True, use_observer=True, marker_deck_ids=None):
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.marker_deck_ids = marker_deck_ids
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._connected)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)
        print(f'CrazyflieClient: Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def _connected(self, uri):
        print(f'CrazyflieClient: Connected to {uri}')
    
    def _fully_connected(self, uri):
        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', marker_deck_ids[3])


        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)
        
        # Reset the ae483 observer
        self.cf.param.set_value('ae483par.reset_observer', 1)

        # Enable the controller (1 for default, 6 for ae483)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 6)
            self.cf.param.set_value('stabilizer.estimator', 2)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)
            self.cf.param.set_value('stabilizer.estimator', 2)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
            self.cf.param.set_value('stabilizer.estimator', 2)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)
            self.cf.param.set_value('stabilizer.estimator', 2)

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._log_data)
                logconf.error_cb.add_callback(self._log_error)
                logconf.start()
            except KeyError as e:
                print(f'CrazyflieClient: Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'CrazyflieClient: Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')
        
        print(f'CrazyflieClient: Fully connected to {uri}')
        self.is_fully_connected = True

    def _connection_failed(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} failed: {msg}')

    def _connection_lost(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} lost: {msg}')

    def _disconnected(self, uri):
        print(f'CrazyflieClient: Disconnected from {uri}')
        self.is_fully_connected = False
    
    def _log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp / 1e3)
            self.data[v.name]['data'].append(data[v.name])

    def _log_error(self, logconf, msg):
        print(f'CrazyflieClient: Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'CrazyflieClient: Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def move_smooth(self, p_inW_1, p_inW_2, yaw, v):
        print(f'Move smoothly from {p_inW_1} to {p_inW_2} with yaw {yaw} degrees at {v} meters / second')

        # Make sure p_inW_1 and p_inW_2 are numpy arrays
        p_inW_1 = np.array(p_inW_1)
        p_inW_2 = np.array(p_inW_2)
        
        # Compute distance from p_inW_1 to p_inW_2
        d = np.linalg.norm(p_inW_1-p_inW_2)              # <-- FIXME (A)
        
        # Compute time it takes to move from p_inW_1 to p_inW_2 at desired speed
        dt = d/v             # <-- FIXME (B)
        
        # Get start time
        start_time = time.time()

        # Repeat until the current time is dt seconds later than the start time
        while True:
            # Get the current time
            t = time.time()
            
            # Compute what fraction of the distance from p_inW_1 to p_inW_2
            # should have been travelled by the current time
            if t < start_time:
                s = 0
            elif t < start_time + dt:
                s = (t-start_time)/dt
            else:
                s = 1          
            
            # Compute where the drone should be at the current time, in the
            # coordinates of the world frame
            p_inW_des = (1-s)*p_inW_1+s*p_inW_2  # <-- FIXME (D)
            
            # Send the desired position (and yaw angle) to the drone
            self.cf.commander.send_position_setpoint(p_inW_des[0], p_inW_des[1], p_inW_des[2], yaw)

            # Stop if the move is complete (i.e., if the desired position is at p_inW_2)
            # otherwise pause for 0.1 seconds before sending another desired position
            if s >= 1:
                time.sleep(0.1)
                return
            else:
                time.sleep(0.1)

    def move_along_curve(self, curve, x0, y0, z0, yaw0):
        file_path = curve  # Update with the actual file path
        times, x_coords, y_coords, velocity = load_and_process_csv(file_path)

        print('Loaded File')

        # Convert times, x_coords, y_coords, velocity to numpy arrays
        times = np.array(times)
        x_coords = np.array(x_coords)
        y_coords = np.array(y_coords)
        velocity = np.array(velocity)

        # Z-coordinate is constant, as passed into the function
        z_coords = z * np.ones_like(x_coords)

        # Stack the coordinates (X, Y, Z)
        coords = np.vstack((x_coords, y_coords, z_coords)).T

        # Get start time
        start_time = time.time()

        print('Begin Flight')

        # Loop through each segment of the path
        for i in range(len(coords) - 1):
            p_inW_1 = coords[i]
            p_inW_2 = coords[i + 1]

            # Calculate the distance between the points
            d = np.linalg.norm(p_inW_1 - p_inW_2)

            # Calculate the time it should take to move based on the velocity at this segment
            dt = d / velocity[i]

            # Get the time at which to start moving this segment
            segment_start_time = time.time() - start_time

            # Loop to move the drone in the current segment
            while True:
                current_time = time.time() - segment_start_time  # Time elapsed in the current segment
                
                # Calculate the fraction of the path to be covered at this time point
                s = min(current_time / dt, 1.0)  # Ensure s doesn't exceed 1.0
                
                # Interpolate the position along the path
                p_inW_des = (1 - s) * p_inW_1 + s * p_inW_2

                # Send the desired position to the drone
                self.cf.commander.send_position_setpoint(x0 + p_inW_des[0], y0 + p_inW_des[1], z0 + p_inW_des[2], yaw0)

                # If the segment is complete, break out of the loop for this segment
                if s >= 1.0:
                    time.sleep(0.15)
                    break
                else:
                    time.sleep(0.15)  # Small sleep to control the rate of position updates

            print(f"Segment {i + 1} completed, moving to the next one.")
        return

    
    
    def stop(self, dt):
        print(f'CrazyflieClient: Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()


###################################
# CLIENT FOR QUALISYS

class QualisysClient(Thread):
    def __init__(self, ip_address, marker_deck_name, pose_queue):
        Thread.__init__(self)
        self.ip_address = ip_address
        self.marker_deck_name = marker_deck_name
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True
        self.data = {
            'time': [],
            'x': [],
            'y': [],
            'z': [],
            'yaw': [],
            'pitch': [],
            'roll': [],
        }
        self.pose_queue = pose_queue
        self.start()

    def close(self):
        self.pose_queue.put('END')
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while (self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        print('QualisysClient: Connect to motion capture system')
        self.connection = await qtm.connect(self.ip_address, version='1.24')
        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip() for index, label in enumerate(xml.findall('*/Body/Name'))]
        await self.connection.stream_frames(
            components=['6d'],
            on_packet=self._on_packet,
        )

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()
        
        if bodies is None:
            print(f'QualisysClient: No rigid bodies found')
            return
        
        if self.marker_deck_name not in self.qtm_6DoF_labels:
            print(f'QualisysClient: Marker deck {self.marker_deck_name} not found')
            return
         
        index = self.qtm_6DoF_labels.index(self.marker_deck_name)
        position, orientation = bodies[index]

        # Get time in seconds, with respect to the qualisys clock
        t = packet.timestamp / 1e6

        # Get position of marker deck (x, y, z in meters)
        x, y, z = np.array(position) / 1e3
        
        # Get orientation of marker deck (yaw, pitch, roll in radians)
        R = Rotation.from_matrix(np.reshape(orientation.matrix, (3, -1), order='F'))
        yaw, pitch, roll = R.as_euler('ZYX', degrees=False)

        # Store time, position, and orientation
        self.data['time'].append(t)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['z'].append(z)
        self.data['yaw'].append(yaw)
        self.data['pitch'].append(pitch)
        self.data['roll'].append(roll)

        # Check if the measurements are valid
        if np.isfinite(x):
            # Convert orientation to quaternion
            qx, qy, qz, qw = R.as_quat()
            # Check if the queue of measurements is empty. We do this because
            # we do not want to create a backlog of measurements. I bet there
            # is a better way of handling this - to *replace* the measurement
            # in the queue with the current one. Revisit this another time!
            if self.pose_queue.empty():
                # Put pose in queue to send to the drone
                self.pose_queue.put((x, y, z, qx, qy, qz, qw))

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()

def send_poses(client, queue):
    print('Start sending poses')
    while True:
        pose = queue.get()
        if pose == 'END':
            print('Stop sending poses')
            break
        x, y, z, qx, qy, qz, qw = pose
        client.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)


###################################
# FLIGHT CODE

if __name__ == '__main__':
    # Specify whether or not to use the motion capture system
    use_mocap = True
    # Initialize radio
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    drone_client = CrazyflieClient(
        uri,
        use_controller=True,
        use_observer=True,
        marker_deck_ids=marker_deck_ids if use_mocap else None,
    )

    # Wait until the client is fully connected to the drone
    while not drone_client.is_fully_connected:
        time.sleep(0.1)
    
    # Create and start the client that will connect to the motion capture system
    if use_mocap:
        pose_queue = SimpleQueue()
        Thread(target=send_poses, args=(drone_client, pose_queue)).start()
        mocap_client = QualisysClient(ip_address, marker_deck_name, pose_queue)

        drone_client.stop(2.0)

        x_0 = mocap_client.data["x"][-1]

        y_0 = mocap_client.data["y"][-1]

        z_0 = mocap_client.data["z"][-1]

        psi_0 = mocap_client.data["yaw"][-1]

        phi_0 = mocap_client.data["roll"][-1]

        theta_0 = mocap_client.data["pitch"][-1]

    drone_client.stop(2.0)

    x = x_0
    y = y_0
    z = z_0

    psi = psi_0

    '''drone_client.move(x, y, z+0.2, psi, 2.)'''

    # Graceful takeoff
    drone_client.move(x, y, z+0.20, psi, 1.)
    drone_client.move_smooth([x, y, z+0.2], [x, y, z+0.5], psi, 0.20)
    drone_client.move(x, y, z+0.5, psi, 1.0)

    drone_client.move(x, y, z+0.5, psi, 2.0)

    drone_client.move_along_curve("smoothed_sampled_points.csv",x,y,z+0.5,psi)


    '''for i in range(1):
        # Move in a square (with a pause at each corner)
        drone_client.move_smooth([x, y, z+0.5], [x+0.5, y, z+0.5], psi, 0.20)
        drone_client.move(x+0.5, y, z+0.5, psi, 1.0)
        drone_client.move_smooth([x+0.5, y, z+0.5], [x+0.5, y+0.5, z+0.5], psi, 0.20)
        drone_client.move(x+0.5, y+0.5, z+0.5, psi, 1.0)
        drone_client.move_smooth([x+0.5, y+0.5, z+0.5], [x, y+0.5, z+0.5], psi, 0.20)
        drone_client.move(x+0.0, y+0.5, z+0.5, psi, 1.0)
        drone_client.move_smooth([x+0.0, y+0.5, z+0.5], [x+0.0, y+0.0, z+0.5], psi, 0.20)
        drone_client.move(x, y, z+0.5, psi, 1.0)'''
    
    
    '''# Graceful landing
    drone_client.move(x, y, z+0.5, psi, 5.0)
    drone_client.move_smooth([x, y, z+0.50], [x+0., y+0., z+0.20], psi, 0.20)
    drone_client.move(x, y, z+0.20, psi, 1.0)'''

    drone_client.stop(2.0)

    # Disconnect from the drone
    drone_client.disconnect()


    '''# Graceful takeoff
    drone_client.move(x, y, z+0.20, psi, 1.)
    drone_client.move_smooth([x, y, z+0.2], [x, y, z+0.5], psi, 0.20)
    drone_client.move(x, y, z+0.5, psi, 1.0)

    drone_client.move(x, y, z+0.5, psi, 5.0)

    drone_client.move_along_curve('smoothed_sampled_points.csv', x, y, z+0.5, psi)

    # Graceful landing
    drone_client.move(x, y, z+0.5, psi, 1.0)
    drone_client.move_smooth([x, y, z+0.50], [x+0., y+0., z+0.20], psi, 0.20)
    drone_client.move(x, y, z+0.20, psi, 1.0)'''

    # Disconnect from the motion capture system
    if use_mocap:
        mocap_client.close()

    # Assemble flight data from both clients
    data = {}
    data['drone'] = drone_client.data
    data['mocap'] = mocap_client.data if use_mocap else {}

    # Write flight data to a file
    with open('hour_glass.json', 'w') as outfile:
        json.dump(data, outfile, sort_keys=False)