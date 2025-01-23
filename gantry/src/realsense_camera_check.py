import pyrealsense2 as rs

def list_realsense_devices():
    # Create a context object. This object owns the handles to all connected devices
    context = rs.context()
    
    # Get device count
    device_count = len(context.query_devices())
    print(f"Number of RealSense devices connected: {device_count}")
    
    # Iterate through the devices
    for i, device in enumerate(context.query_devices()):
        print(f"\nDevice {i}:")
        print(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")
        print(f"Name: {device.get_info(rs.camera_info.name)}")
        print(f"Product ID: {device.get_info(rs.camera_info.product_id)}")
        
        # Get sensor information
        sensors = device.query_sensors()
        for j, sensor in enumerate(sensors):
            print(f"  Sensor {j}:")
            print(f"  Type: {sensor.get_info(rs.camera_info.name)}")

if __name__ == "__main__":
    list_realsense_devices()