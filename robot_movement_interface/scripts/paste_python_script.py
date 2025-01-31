import rmi_api

# Define positions
default_position = (0.0, 0.0, 0.0)
cube_positions = [(100, 200, 15), (140, 200, 15), (180, 200, 15), (100, 250, 15), (140, 250, 15), (180, 250, 15)]
camera_position = (-100, 200, 200)
not_ok_bin_position = (500, 200, 100)

# Move to default position
rmi_api.move_to(*default_position)

# Loop through each cube
for cube_position in cube_positions:
    # Pick up cube
    rmi_api.move_to(*cube_position)
    rmi_api.grab()

    # Check cube
    rmi_api.move_to(*camera_position)
    cube_ok = rmi_api.check_cube()

    # Sort cube
    if cube_ok:
        rmi_api.move_to(*cube_position)
    else:
        rmi_api.move_to(*not_ok_bin_position)
    rmi_api.release()

# Finalize
rmi_api.move_to(*default_position)
rmi_api.open()