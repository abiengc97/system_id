import rosbag
import csv
import math

# Specify the bag file and output CSV file
bag = rosbag.Bag('system.bag')
output_file = 'system_id_data.csv'

with open(output_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'roll_cmd', 'roll_actual', 'pitch_cmd', 'pitch_actual'])  # Column headers

    roll_cmd = None
    pitch_cmd = None

    for topic, msg, t in bag.read_messages(topics=['/mavros/setpoint_attitude/cmd_vel', '/mavros/local_position/pose']):
        print(f"Reading topic: {topic} at time: {t.to_sec()}")  # Debug print
        if topic == '/mavros/setpoint_attitude/cmd_vel':
            roll_cmd = msg.twist.angular.x  # Commanded roll
            pitch_cmd = msg.twist.angular.y  # Commanded pitch
            print(f"Commanded roll: {roll_cmd}, Commanded pitch: {pitch_cmd}")  # Debug print
        elif topic == '/mavros/local_position/pose':
            # Extract actual roll and pitch from quaternion orientation
            q = msg.pose.orientation
            sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
            pitch_actual = math.asin(sin_pitch)
            roll_actual = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x**2 + q.y**2))
            print(f"Actual roll: {roll_actual}, Actual pitch: {pitch_actual}")  # Debug print
            
            # Ensure that command values are available before writing to CSV
            if roll_cmd is not None and pitch_cmd is not None:
                writer.writerow([t.to_sec(), roll_cmd, roll_actual, pitch_cmd, pitch_actual])
                print(f"Written to CSV: time={t.to_sec()}, roll_cmd={roll_cmd}, roll_actual={roll_actual}, pitch_cmd={pitch_cmd}, pitch_actual={pitch_actual}")  # Debug print

bag.close()
print(f"Data saved to {output_file}")