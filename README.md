from pymavlink import mavutil  # Import the mavutil library from pymavlink for drone communication
import cv2  # Import OpenCV for image processing
import numpy as np  # Import NumPy for numerical operations

# Establish a connection to the drone via TCP
connection = mavutil.mavlink_connection('tcp:localhost:5762')
# Wait for the drone's heartbeat to ensure connection is alive
connection.wait_heartbeat()

# Define a dictionary to map flight modes to their corresponding IDs
mode_map = {
    "LOITER": 5,
    "AUTO": 3,
    "RTL": 6,
    "MANUAL": 0,
    "GUIDED": 4
}

# Function to change the drone's flight mode
def set_mode(mode):
    if mode not in mode_map:
        print(f"Invalid mode: {mode}")
        return

    # Get the corresponding mode ID from the mode_map
    mode_id = mode_map[mode]

    # Send a MAVLink command to set the drone's mode
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0, 0
    )

    # Wait for an acknowledgment from the drone
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

# Function to move the drone in the local NED (North-East-Down) frame
def move_drone(north, east, altitude):
    # Send a MAVLink command to move the drone to the specified position
    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,  # Mask to specify which axes to control
        north, east, -altitude,  # Target positions in North, East, and Down
        0, 0, 0,  # Target velocities (not used here)
        0, 0, 0,  # Target accelerations (not used here)
        0, 0  # Yaw and Yaw rate (not used here)
    )

# Function to command the drone to land
def land_drone():
    # Send a MAVLink command to land the drone
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    # Wait for an acknowledgment from the drone
    msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

# Function to detect QR codes and move the drone accordingly
def track_and_move():
    # Initialize the webcam for capturing video
    cap = cv2.VideoCapture(0)
    # Get the width and height of the video frames
    frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    # Initialize a QR code detector
    qr_code_detector = cv2.QRCodeDetector()

    while True:
        ret, frame = cap.read()  # Capture a frame from the webcam
        if not ret:
            break  # Exit if no frame is captured

        # Detect and decode the QR code from the frame
        decoded_text, points, _ = qr_code_detector.detectAndDecode(frame)
        if points is not None and len(points) == 4:
            points = points.reshape(-1, 2)  # Reshape the points array

            # Draw a rectangle around the detected QR code
            cv2.rectangle(frame, tuple(points[0]), tuple(points[2]), (255, 0, 255), 2)

            # Calculate the center of the QR code
            cx = int(np.mean(points[:, 0]))
            cy = int(np.mean(points[:, 1]))

            # Calculate the movement required to center the drone on the QR code
            move_x = (cx - frame_width / 2) / frame_width
            move_y = (cy - frame_height / 2) / frame_height

            # Scale the movement and set the altitude
            scale_factor = 5
            north = move_y * scale_factor
            east = move_x * scale_factor
            altitude = 10

            # Print the calculated position to move the drone
            print(f"Moving drone to North: {north}, East: {east}, Altitude: {altitude}")
            move_drone(north, east, altitude)  # Move the drone

        else:
            print("QR code not detected or invalid.")

        # Display the video frame with the QR code rectangle
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit loop if 'q' key is pressed
            break

    # Release the webcam and close OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

# Main control loop for interacting with the drone
while True:
    # Prompt user for input to select a command
    girdi = input("Enter a command (1=Arm, 2=TakeOff, 3=Land, 4=Track QR, 5=Change Mode): ")

    if girdi == "1":
        # Arm the drone
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    elif girdi == "2":
        # Command the drone to take off
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

    elif girdi == "3":
        # Land the drone
        land_drone()

    elif girdi == "4":
        # Start tracking the QR code and moving the drone accordingly
        track_and_move()

    elif girdi == "5":
        # Change the flight mode of the drone
        mode = input("Enter a mode (LOITER, AUTO, RTL, MANUAL, GUIDED): ").upper()
        set_mode(mode)

    else:
        print("Please enter a valid command: 1, 2, 3, 4, or 5.")
