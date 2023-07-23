from geometry_msgs.msg import TransformStamped
from pathlib import Path
import rosbag
from tf2_msgs.msg import TFMessage


# Get the absolute path of the current file
current_file_path = Path(__file__).resolve()

print(current_file_path)
bag_name = '2D_24-08-13-00-00.bag'

raw_bag_path = current_file_path / '../..' / 'data/raw' / bag_name
processed_bag_path = current_file_path / '../..' / 'data/processed' / bag_name

# Open the input rosbag file
with rosbag.Bag(raw_bag_path.resolve(), 'r') as input_bag:
    # Open the output rosbag file in write mode
    with rosbag.Bag(processed_bag_path.resolve(), 'w') as output_bag:
        # Loop over the messages in the input bag
        for topic, msg, t in input_bag.read_messages():
            # Check if the message is a String message
            if msg._type == 'geometry_msgs/Pose':
                # Create a TransformStamped message
                transform = TransformStamped()

                # Set the header of the transform
                transform.header.stamp = t
                transform.header.frame_id = "base"
                transform.child_frame_id = "base_laser_link"

                # Set the pose of the transform
                transform.transform.translation.x = msg.position.x
                transform.transform.translation.y = msg.position.y
                transform.transform.translation.z = msg.position.z
                transform.transform.rotation.x = msg.orientation.x
                transform.transform.rotation.y = msg.orientation.y
                transform.transform.rotation.z = msg.orientation.z
                transform.transform.rotation.w = msg.orientation.w
                
                tf = TFMessage()
                tf.transforms = [transform]

                # Write the message to the output bag
                output_bag.write('/tf', tf, t)

            if msg._type == 'sensor_msgs/LaserScan':
                output_bag.write(topic, msg, t)
