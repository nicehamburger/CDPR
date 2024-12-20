import numpy as np
import cv2
import time
import socket
import math

current_mouse_pos = [0, 0]
pos_changed = False

def get_homography_matrix(input_points, output_points):
    """
    Given points from the image frame and the real-world frame, compute
    the homography matrix H, which maps input points to output points.

    Arguments:
    input_points: A 2xN numpy array of input points (e.g., image frame points).
    output_points: A 2xN numpy array of output points (e.g., real-world points).

    Returns:
    A 3x3 homography matrix that represents the perspective transformation.
    """
    # Transpose the input and output points to make them Nx2 arrays.
    input_points = input_points.transpose()
    output_points = output_points.transpose()

    # Validate that input and output points have the same dimensions.
    if input_points.shape != output_points.shape:
        raise ValueError("Input points and output points do not match dimensions")

    # Ensure points are represented as 2D coordinates.
    if input_points.shape[1] != 2:
        raise ValueError("Incorrect format, make sure points form columns")

    # Ensure there are at least 4 point correspondences to compute the homography matrix.
    n = input_points.shape[0]
    if n < 4:
        raise ValueError("Need at least 4 points to compute the homography matrix")

    A = []  # Initialize the matrix to hold the equations for Singular Value Decomposition

    # Loop through each pair of corresponding points to populate the A matrix.
    for i in range(n):
        X, Y = input_points[i]   # Input point (image frame).
        x, y = output_points[i]  # Output point (real-world frame).

        # Each correspondence contributes two rows to matrix A.
        A.append([-X, -Y, -1, 0, 0, 0, x * X, x * Y, x])  # Row 1 of the pair.
        A.append([0, 0, 0, -X, -Y, -1, y * X, y * Y, y])  # Row 2 of the pair.

    A = np.array(A)  # Convert A to a numpy array.

    # Perform Singular Value Decomposition (SVD) to solve Ah = 0.
    _, _, Vt = np.linalg.svd(A)

    # The homography matrix H is the last row of V (or Vt[-1]), reshaped into a 3x3 matrix.
    H = Vt[-1].reshape((3, 3))
    return H

def get_pixel_position(event, x, y, flags, param):
    """
    Callback function to capture mouse click events and record pixel positions.

    Arguments:
    event: The type of mouse event (e.g., left-button click).
    x, y: Coordinates of the mouse click.
    flags: Any relevant flags passed by OpenCV.
    param: A list containing:
           - An integer counter for how many points have been recorded.
           - A 2xN numpy array to store the clicked points.

    This function updates the provided array with up to 4 points from mouse clicks.
    """
    if event == cv2.EVENT_LBUTTONDOWN:  # Check if the left mouse button was clicked.
        print(f"Mouse clicked at position: ({x}, {y})")  # Print the click coordinates.

        if param[0] < 4:  # Check if fewer than 4 points have been recorded.
            # Store the clicked point in the array at the current counter index.
            (param[1])[0, param[0]] = x  # Store the x-coordinate.
            (param[1])[1, param[0]] = y  # Store the y-coordinate.

            param[0] += 1  # Increment the counter.
        else:
            # If 4 points have already been recorded, update a global variable with the mouse position.
            global current_mouse_pos
            global pos_changed

            # Set the global flags and variables to reflect the new position.
            pos_changed = True
            current_mouse_pos[0] = x
            current_mouse_pos[1] = y

# Set up the TCP server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 65432))  # Bind to all interfaces on port 65432
server_socket.listen(1)
print("Waiting for a connection...")

# Wait for a client to connect
conn, addr = server_socket.accept()
print(f"Connected by {addr}")

video_capture = cv2.VideoCapture(1)
success, frame = video_capture.read()

# Index, pin, pout
calibration = [0, np.zeros(shape=(2, 4)), np.array([[30, 50, 30, 20], [40, 30, 20, 30]])]

H = None
roi = None
# Initial Coordinates
x_old = 30
y_old = 30

# Calibration phase
while success:
    cv2.imshow("Frame", frame)
    cv2.setMouseCallback("Frame", get_pixel_position, calibration)

    if calibration[0] == 4:
        H = get_homography_matrix(calibration[1], calibration[2])
        roi = cv2.selectROI("Frame", frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Frame")
        break

    if cv2.waitKey(30) & 0xFF == 27:
        print("Exiting")
        break

    success, frame = video_capture.read()

# Now we have the homography matrix
success, frame = video_capture.read()
tracker = cv2.TrackerCSRT_create()
tracker.init(frame, roi)
prev_time = time.time()

lost_object_recalib = False

while success:
    cv2.imshow("Tracking", frame)

    if cv2.waitKey(30) & 0xFF == ord('r') or lost_object_recalib:
        roi = cv2.selectROI("New Frame", frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("New Frame")
        tracker = cv2.TrackerCSRT_create()
        tracker.init(frame, roi)
        lost_object_recalib = False

    s, boundbox = tracker.update(frame)
    if s:
        x, y, w, h = [int(v) for v in boundbox]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green color for bounding box
        cv2.imshow("Tracking", frame)
        point = np.array([[x + (w / 2)], [y + (h / 2)], [1]])
        point = np.matmul(H, point)
        point = point[:2] / point[2]
        cur_time = time.time()

        if cur_time - prev_time > 1.5:
            print("Sent: ", point[0, 0], point[1, 0])
            conn.sendall(f"{point[0, 0]}, {point[1, 0]}".encode())
            prev_time = cur_time
        
    else:
        lost_object_recalib = True

    if cv2.waitKey(30) & 0xFF == 27:
        print("Exiting")
        conn.sendall(f"-500.0, -500.0".encode())
        break
    success, frame = video_capture.read()

conn.close()
server_socket.close()
cv2.destroyAllWindows()