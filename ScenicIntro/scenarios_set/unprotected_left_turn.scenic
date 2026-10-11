model scenic.simulators.newtonian.model

param render = True

# Layout setup
lane_width = 3.5
ego_start_y = -30

# 1. Ego Vehicle (Traveling North at 5 m/s)
ego = new Object at (0, ego_start_y),
    facing 0 deg,
    with width 2.0,
    with length 4.5,
    with speed 5

# 2. Adversary Vehicle (Approaching South at 8-14 m/s)
adversary = new Object at (lane_width, Range(20, 35)),
    facing 180 deg,
    with width 2.0,
    with length 4.5,
    with speed Range(8, 14)

# 3. Parked Obstacle (Delivery van / truck)
obstacle = new Object at (lane_width, Range(6, 14)),
    facing 180 deg,
    with width 2.5,
    with length 7.0,
    with speed 0

# 4. Temporal Safety Monitor
monitor CollisionMonitor():
    while True:
        if (distance to adversary) < 3.0:
            print("ALERT: Near miss or collision detected!")
            terminate simulation
        wait

# Termination conditions
terminate when ego.position.y > ego_start_y + 50
terminate after 120 steps
