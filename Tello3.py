from djitellopy import tello

# Create a variale to control the Tello Drone
maverick = tello.Tello()

# Connect to the drone and take off
maverick.connect()
maverick.takeoff()

# Land the drone
maverick.land()