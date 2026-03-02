from controller import Robot, Keyboard

TIME_STEP = 64
MAX_SPEED = 6.28  # radians/sec

def main():
    robot = Robot()
    keyboard = robot.getKeyboard()
    keyboard.enable(TIME_STEP)

    motor_names = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
    motors = []

    # Get and enable Camera
    camera = robot.getDevice("camera")
    width = 640
    if camera:
        camera.enable(TIME_STEP)
        width = camera.getWidth()
        print("Camera enabled. Feed now looking forward.")
    else:
        print("Warning: could not find 'camera' device.")

    # Get and enable Lidar
    lidar = robot.getDevice("lidar")
    if lidar:
        lidar.enable(TIME_STEP)
        lidar.enablePointCloud()
        print("Lidar enabled.")
    else:
        print("Warning: could not find 'lidar' device.")

    # Get and enable Pan/Tilt Motors
    pan_motor = robot.getDevice("pan_motor")
    tilt_motor = robot.getDevice("tilt_motor")
    if pan_motor:
        pan_motor.setPosition(0.0)
        pan_motor.setVelocity(1.0)
    if tilt_motor:
        tilt_motor.setPosition(0.0)
        tilt_motor.setVelocity(1.0)

    pan_pos = 0.0
    tilt_pos = 0.0

    for name in motor_names:
        motor = robot.getDevice(name)
        if motor:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            motors.append(motor)
        else:
            print(f"Warning: could not find motor '{name}'")

    if len(motors) < 4:
        print("Error: Could not find all 4 wheels.")
        return

    mode = 2  # 1 = Auto, 2 = Manual
    options = ["Default Speed", "High Speed", "Eco Mode"]
    current_option = 0
    current_max_speed = MAX_SPEED
    in_options_menu = False

    print("--- Robot Initialized ---")
    print("Mode: MANUAL. Keys: W/A/S/D to move, Q/E to slide sideways, Space to stop.")
    print("Camera: Arrow keys to Pan/Tilt.")
    print("Other: Press '1' for AUTO, '2' for MANUAL.")
    print("Options: Press 'O' to cycle options, 'X' to exit/reset.")

    while robot.step(TIME_STEP) != -1:
        key = keyboard.getKey()
        # Standardize key (handle both upper and lower case)
        key_char = ""
        if key >= 0:
            try:
                key_char = chr(key).upper()
            except ValueError:
                key_char = ""

        if key == ord('1'):
            if mode != 1:
                mode = 1
                print(">> Switched to AUTO Mode")
        elif key == ord('2'):
            if mode != 2:
                mode = 2
                print(">> Switched to MANUAL Mode")
        
        # Options logic
        if key_char == 'O':
            current_option = (current_option + 1) % len(options)
            print(f"[OPTION SELECTED] {options[current_option]}")
            # Apply option effects
            if current_option == 1: # High Speed
                current_max_speed = MAX_SPEED * 1.5
            elif current_option == 2: # Eco Mode
                current_max_speed = MAX_SPEED * 0.5
            else:
                current_max_speed = MAX_SPEED
        elif key_char == 'X':
            print(">> Exiting/Resetting Options")
            current_option = 0
            current_max_speed = MAX_SPEED
            print(f"[OPTION RESET] {options[0]}")

        forward = 0.0
        sideways = 0.0
        rotate = 0.0
        
        # Camera Control (Arrow Keys)
        if key == Keyboard.LEFT: pan_pos += 0.05
        elif key == Keyboard.RIGHT: pan_pos -= 0.05
        elif key == Keyboard.UP: tilt_pos += 0.05
        elif key == Keyboard.DOWN: tilt_pos -= 0.05
        
        # Constrain camera movement
        pan_pos = max(min(pan_pos, 1.57), -1.57)
        tilt_pos = max(min(tilt_pos, 1.57), -1.57)
        
        if pan_motor: pan_motor.setPosition(pan_pos)
        if tilt_motor: tilt_motor.setPosition(tilt_pos)

        if mode == 2:
            # Manual Mode
            if key_char == 'W': forward = current_max_speed
            elif key_char == 'S': forward = -current_max_speed
            elif key_char == 'A': rotate = -current_max_speed
            elif key_char == 'D': rotate = current_max_speed
            elif key_char == 'Q': sideways = -current_max_speed
            elif key_char == 'E': sideways = current_max_speed
            
        elif mode == 1:
            # Auto Mode - Search and approach Red Sphere
            if camera:
                image = camera.getImageArray()
                if image:
                    red_pixels = 0
                    sum_x = 0
                    
                    # Sub-sampled to save cpu time
                    for x in range(0, width, 4):
                        for y in range(0, camera.getHeight(), 4):
                            r = image[x][y][0]
                            g = image[x][y][1]
                            b = image[x][y][2]
                            
                            # Simple logic for detecting bright red
                            if r > 150 and g < 100 and b < 100:
                                red_pixels += 1
                                sum_x += x
                                
                    if red_pixels > 10:
                        center_x = sum_x / red_pixels
                        error = center_x - (width / 2.0)
                        
                        # Use LIDAR to stop if close
                        min_dist = 10.0
                        if lidar:
                            range_image = lidar.getRangeImage()
                            if range_image:
                                center_i = len(range_image) // 2
                                # look at center 30 rays
                                center_rays = range_image[center_i - 15 : center_i + 15]
                                valid_rays = [r for r in center_rays if r > 0.05 and r < float('inf')]
                                if valid_rays:
                                    min_dist = min(valid_rays)
                                
                        if min_dist < 0.2:
                            # Reached! Stop.
                            forward = 0.0
                            rotate = 0.0
                        else:
                            forward = MAX_SPEED * 0.8  # Move forward
                            
                            # Adjust rotation based on center of red blob
                            if error > 20: 
                                rotate = MAX_SPEED * 0.4 # turn right
                                forward = MAX_SPEED * 0.4
                            elif error < -20: 
                                rotate = -MAX_SPEED * 0.4 # turn left
                                forward = MAX_SPEED * 0.4
                    else:
                        # turn slowly to search for the red ball
                        rotate = MAX_SPEED * 0.5
                        
        # Mecanum logic combinations
        fl = forward + sideways + rotate
        fr = forward - sideways - rotate
        bl = forward - sideways + rotate
        br = forward + sideways - rotate
        
        # Determine maximum requested speed to scale if exceeding max_speed
        max_val = max(abs(fl), abs(fr), abs(bl), abs(br))
        if max_val > MAX_SPEED:
            fl = (fl / max_val) * MAX_SPEED
            fr = (fr / max_val) * MAX_SPEED
            bl = (bl / max_val) * MAX_SPEED
            br = (br / max_val) * MAX_SPEED
            
        motors[0].setVelocity(fl)  # front-left
        motors[1].setVelocity(fr)  # front-right
        motors[2].setVelocity(bl)  # back-left
        motors[3].setVelocity(br)  # back-right

if __name__ == '__main__':
    main()
