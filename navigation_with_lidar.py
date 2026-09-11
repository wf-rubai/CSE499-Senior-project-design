import math
import numpy as np
import pygame

# ============================================================
# 1. SIMULATION PARAMETERS
# ============================================================
MAP_X = 15.0
MAP_Y = 15.0

# Boat
BOAT_SPEED = 0.05
TURN_SPEED = math.radians(3)

# Minimum turning radius
RMIN = 0.5

# Goal
GOAL = np.array([13.0, 13.0])
GOAL_TOLERANCE = 0.5

# LiDAR
MAX_RANGE = 2.0
NUM_RAYS = 360
RAY_STEP = 0.02

# Obstacle safety distance
SAFE = 0.8

# Pygame
SCALE = 50
WIDTH = int(MAP_X * SCALE)
HEIGHT = int(MAP_Y * SCALE)

FPS = 60

# ============================================================
# 2. ENVIRONMENT
# ============================================================
rectangles = [
    (3, 2, 1, 8),
    (6, 6, 5, 1),
    (9, 0, 1, 5),
    (11, 8, 1, 5),
]

trees = [
    (5, 12, 0.5),
    (8, 10, 0.5),
    (2, 11, 0.5),
]

# ============================================================
# 3. HELPER FUNCTIONS
# ============================================================
def wrap_angle(angle):
    """
    Normalize angle to [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi

def world_to_screen(x, y):
    """
    Convert simulation coordinates to Pygame coordinates.

    Simulation:
        x -> right
        y -> up

    Pygame:
        x -> right
        y -> down
    """

    sx = int(x * SCALE)
    sy = int(HEIGHT - y * SCALE)

    return sx, sy

def point_inside_obstacle(x, y):

    for rx, ry, width, height in rectangles:

        if (rx <= x <= rx + width and
                ry <= y <= ry + height):

            return True

    for cx, cy, radius in trees:

        distance_squared = (
            (x - cx) ** 2 +
            (y - cy) ** 2
        )

        if distance_squared <= radius ** 2:
            return True

    return False

# ============================================================
# 4. SENSOR READING
# ============================================================
def sensor_reading(position, heading):
    x0, y0 = position

    relative_angles = np.linspace(0, 2 * math.pi, NUM_RAYS, endpoint=False)

    distances = []
    points = []

    for relative_angle in relative_angles:
        if relative_angle < 135 * math.pi / 180 or relative_angle > 225 * math.pi / 180:
            world_angle = heading + relative_angle

            dx = math.cos(world_angle)
            dy = math.sin(world_angle)

            detected_distance = MAX_RANGE

            for distance in np.arange(0, MAX_RANGE, RAY_STEP):
                x = x0 + distance * dx
                y = y0 + distance * dy

                # Map boundary
                if (x < 0 or x > MAP_X or y < 0 or y > MAP_Y):
                    detected_distance = distance
                    break

                # Obstacle
                if point_inside_obstacle(x, y):
                    detected_distance = distance
                    break

            distances.append(detected_distance)

            if detected_distance < MAX_RANGE:
                px = x0 + detected_distance * dx
                py = y0 + detected_distance * dy

                points.append([px, py])
        else:
            SMALL_RANGE = 0.5
            distances.append(SMALL_RANGE)
            points.append([x0 + SMALL_RANGE * math.cos(heading + relative_angle),
                           y0 + SMALL_RANGE * math.sin(heading + relative_angle)])

    distances = np.array(distances)
    points = np.array(points)

    def sector_min(center_deg, width_deg):
        angle_error = np.array([
            abs(
                math.degrees(
                    wrap_angle(a - math.radians(center_deg))
                )
            )
            for a in relative_angles
        ])

        valid = angle_error <= width_deg
        if np.any(valid):
            return np.min(distances[valid])

        return MAX_RANGE

    front = sector_min(0, 15)
    left = sector_min(90, 20)
    right = sector_min(-90, 20)

    return (relative_angles, distances, points, front, left, right)

# ============================================================
# 5. MANUAL CONTROL
# ============================================================
def manual_control(position, heading, keys):

    new_position = position.copy()
    new_heading = heading

    # --------------------------------------------------------
    # LEFT / RIGHT = rotate
    # --------------------------------------------------------

    if keys[pygame.K_LEFT]:
        new_heading += TURN_SPEED

    if keys[pygame.K_RIGHT]:
        new_heading -= TURN_SPEED

    # --------------------------------------------------------
    # UP = move forward
    # DOWN = move backward
    # --------------------------------------------------------

    velocity = 0

    if keys[pygame.K_UP]:
        velocity = BOAT_SPEED

    if keys[pygame.K_DOWN]:
        velocity = -BOAT_SPEED

    new_x = (
        new_position[0] +
        velocity * math.cos(new_heading)
    )

    new_y = (
        new_position[1] +
        velocity * math.sin(new_heading)
    )

    # --------------------------------------------------------
    # Collision prevention
    # --------------------------------------------------------

    if (
        0 <= new_x <= MAP_X and
        0 <= new_y <= MAP_Y and
        not point_inside_obstacle(new_x, new_y)
    ):

        new_position[0] = new_x
        new_position[1] = new_y

    return new_position, new_heading

# ============================================================
# 6. AUTONOMOUS CONTROL
# ============================================================
def find_gaps(angles, distances):
    gaps = []
    start_angle = None
    end_angle = None
    for angle, distance in zip(angles, distances):
        if distance >= MAX_RANGE and start_angle is None:
            start_angle = angle
        elif distance < MAX_RANGE and start_angle is not None:
            end_angle = angle
            mean_angle = (start_angle + end_angle) / 2
            gaps.append(mean_angle)
            start_angle = None
            end_angle = None

    return gaps

def autonomous_control(position, heading, angles, distances, front, left, right, mode):
    # --------------------------------------------------------
    # Goal direction
    # --------------------------------------------------------
    dx = GOAL[0] - position[0]
    dy = GOAL[1] - position[1]

    target_heading = math.atan2(dy, dx)

    error = wrap_angle(
        target_heading - heading
    )

    error_deg = math.degrees(error)

    # --------------------------------------------------------
    # Check whether goal reached
    # --------------------------------------------------------
    goal_distance = math.sqrt(
        dx ** 2 + dy ** 2
    )

    if goal_distance <= GOAL_TOLERANCE:
        return position, heading, "GOAL"

    # --------------------------------------------------------
    # STATE MACHINE
    # --------------------------------------------------------

    GO_TO_GOAL = 0
    FOLLOW_LEFT = 1
    FOLLOW_RIGHT = 2

    # --------------------------------------------------------
    # GO TO GOAL
    # --------------------------------------------------------
    if mode == GO_TO_GOAL:
        # Front obstacle
        if front < SAFE:
            # Choose the side with more free space
            if left > right:
                mode = FOLLOW_LEFT
            else:
                mode = FOLLOW_RIGHT

        else:
            # Simple bang-bang heading controller
            if error_deg > 10:
                heading += TURN_SPEED

            elif error_deg < -10:
                heading -= TURN_SPEED

            else:
                # Move forward
                new_x = ( position[0] + BOAT_SPEED * math.cos(heading) )

                new_y = ( position[1] + BOAT_SPEED * math.sin(heading) )

                if ( 0 <= new_x <= MAP_X and 0 <= new_y <= MAP_Y and not point_inside_obstacle(new_x, new_y) ):
                    position = np.array([ new_x, new_y ])

    # --------------------------------------------------------
    # FOLLOW LEFT
    # --------------------------------------------------------
    elif mode == FOLLOW_LEFT:
        # Keep obstacle on left
        if front < SAFE:
            # Turn right
            heading -= TURN_SPEED

        elif left < SAFE:
            # Obstacle is close on left
            # turn slightly right
            heading -= TURN_SPEED * 0.5

            new_x = ( position[0] + BOAT_SPEED * math.cos(heading) )

            new_y = ( position[1] + BOAT_SPEED * math.sin(heading) )

            if ( not point_inside_obstacle(new_x, new_y) ):
                position = np.array([ new_x, new_y ])

        else:

            # Obstacle disappeared from left
            # try to move toward goal

            if error_deg > 10:
                heading += TURN_SPEED

            elif error_deg < -10:
                heading -= TURN_SPEED

            else:

                new_x = ( position[0] + BOAT_SPEED * math.cos(heading) )

                new_y = (position[1] + BOAT_SPEED * math.sin(heading))

                if (
                    not point_inside_obstacle(new_x, new_y)
                ):

                    position = np.array([new_x, new_y])

            # Return to goal following
            if (
                front > SAFE and
                abs(error_deg) < 15
            ):
                mode = GO_TO_GOAL

    # --------------------------------------------------------
    # FOLLOW RIGHT
    # --------------------------------------------------------
    elif mode == FOLLOW_RIGHT:
        # Keep obstacle on right
        if front < SAFE:
            # Turn left
            heading += TURN_SPEED

        elif right < SAFE:
            # Obstacle close on right
            heading += TURN_SPEED * 0.5

            new_x = (position[0] + BOAT_SPEED * math.cos(heading))

            new_y = (position[1] + BOAT_SPEED * math.sin(heading))

            if (not point_inside_obstacle(new_x, new_y)):
                position = np.array([new_x, new_y ])

        else:
            # Try to return toward goal

            if error_deg > 10:
                heading += TURN_SPEED

            elif error_deg < -10:
                heading -= TURN_SPEED

            else:
                new_x = (position[0] + BOAT_SPEED * math.cos(heading))

                new_y = (position[1] + BOAT_SPEED * math.sin(heading))

                if (not point_inside_obstacle(new_x, new_y)):
                    position = np.array([new_x, new_y])

            if (front > SAFE and abs(error_deg) < 15):
                mode = GO_TO_GOAL

    return position, heading, mode

# ============================================================
# 7. DRAW ENVIRONMENT
# ============================================================
def draw_environment(screen, position, heading, lidar_points, mode, front, left, right):

    screen.fill((20, 20, 20))

    # --------------------------------------------------------
    # Draw obstacles
    # --------------------------------------------------------

    for rx, ry, width, height in rectangles:

        pygame.draw.rect(
            screen,
            (20, 20, 20),
            pygame.Rect(
                int(rx * SCALE),
                int(HEIGHT - (ry + height) * SCALE),
                int(width * SCALE),
                int(height * SCALE)
            )
        )

    for cx, cy, radius in trees:

        pygame.draw.circle(
            screen,
            (20, 20, 20),
            world_to_screen(cx, cy),
            int(radius * SCALE)
        )

    # --------------------------------------------------------
    # Draw LiDAR point cloud
    # --------------------------------------------------------

    for point in lidar_points:

        px, py = world_to_screen(
            point[0],
            point[1]
        )

        bx, by = world_to_screen(
            position[0],
            position[1]
        )

        # Line from LiDAR center to detected point
        pygame.draw.line(
            screen,
            (0, 255, 100),
            (bx, by),
            (px, py),
            1
        )

        pygame.draw.circle(
            screen,
            (0, 255, 100),
            (px, py),
            2
        )

    # --------------------------------------------------------
    # Draw boat
    # --------------------------------------------------------

    bx, by = world_to_screen(
        position[0],
        position[1]
    )

    pygame.draw.circle(
        screen,
        (255, 60, 60),
        (bx, by),
        8
    )

    # Heading line
    hx = (
        position[0] +
        0.5 * math.cos(heading)
    )

    hy = (
        position[1] +
        0.5 * math.sin(heading)
    )

    hx, hy = world_to_screen(hx, hy)

    pygame.draw.line(
        screen,
        (255, 200, 50),
        (bx, by),
        (hx, hy),
        3
    )

    # --------------------------------------------------------
    # Draw goal
    # --------------------------------------------------------

    gx, gy = world_to_screen(
        GOAL[0],
        GOAL[1]
    )

    pygame.draw.circle(
        screen,
        (255, 50, 50),
        (gx, gy),
        10,
        3
    )

    pygame.draw.line(
        screen,
        (255, 50, 50),
        (gx - 8, gy),
        (gx + 8, gy),
        3
    )

    pygame.draw.line(
        screen,
        (255, 50, 50),
        (gx, gy - 8),
        (gx, gy + 8),
        3
    )

    # --------------------------------------------------------
    # Information
    # --------------------------------------------------------

    font = pygame.font.SysFont(None, 24)

    if mode == "MANUAL":
        mode_text = "MODE: MANUAL"
    elif mode == "GOAL":
        mode_text = "GOAL REACHED"
    else:
        mode_text = "MODE: AUTONOMOUS"

    text1 = font.render(
        mode_text,
        True,
        (255, 255, 255)
    )

    text2 = font.render(
        "Press S = switch mode",
        True,
        (200, 200, 200)
    )

    text3 = font.render(
        f"Front: {front:.2f} m",
        True,
        (200, 200, 200)
    )

    text4 = font.render(
        f"Left: {left:.2f} m",
        True,
        (200, 200, 200)
    )

    text5 = font.render(
        f"Right: {right:.2f} m",
        True,
        (200, 200, 200)
    )

    screen.blit(text1, (10, 10))
    screen.blit(text2, (10, 35))
    screen.blit(text3, (10, 60))
    screen.blit(text4, (10, 85))
    screen.blit(text5, (10, 110))

# ============================================================
# 8. MAIN
# ============================================================
def main():

    pygame.init()

    screen = pygame.display.set_mode(
        (WIDTH, HEIGHT)
    )

    pygame.display.set_caption(
        "LiDAR Boat Navigation Simulation"
    )

    clock = pygame.time.Clock()

    # --------------------------------------------------------
    # Initial boat position
    # --------------------------------------------------------

    position = np.array([2.0, 2.0], dtype=float)

    # Boat initially points right
    heading = 0.0

    # --------------------------------------------------------
    # Modes
    # --------------------------------------------------------

    MANUAL = 0
    AUTONOMOUS = 1

    control_mode = MANUAL

    # Autonomous state
    GO_TO_GOAL = 0
    FOLLOW_LEFT = 1
    FOLLOW_RIGHT = 2

    auto_mode = GO_TO_GOAL

    running = True

    while running:

        # ====================================================
        # EVENTS
        # ====================================================

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            # S switches between manual/autonomous
            if event.type == pygame.KEYDOWN:

                if event.key == pygame.K_s:

                    if control_mode == MANUAL:

                        control_mode = AUTONOMOUS

                        # Start autonomous mode
                        auto_mode = GO_TO_GOAL

                    else:

                        control_mode = MANUAL

        # ====================================================
        # SENSOR READING
        # ====================================================
        (angles, distances, lidar_points, front, left, right) = sensor_reading(position, heading)

        # ====================================================
        # CONTROL
        # ====================================================
        if control_mode == MANUAL:

            keys = pygame.key.get_pressed()

            position, heading = manual_control(
                position,
                heading,
                keys
            )

            display_mode = "MANUAL"

        else:

            position, heading, auto_mode = autonomous_control(
                    position,
                    heading,
                    angles,
                    distances,
                    front,
                    left,
                    right,
                    auto_mode
                )

            if auto_mode == "GOAL":

                display_mode = "GOAL"

            else:

                display_mode = "AUTONOMOUS"

        # ====================================================
        # DRAW
        # ====================================================

        draw_environment(
            screen,
            position,
            heading,
            lidar_points,
            display_mode,
            front,
            left,
            right
        )

        pygame.display.flip()

        clock.tick(FPS)

    pygame.quit()

# ============================================================
# RUN
# ============================================================
if __name__ == "__main__":
    main()
    
    position = np.array([2.0, 2.0], dtype=float)
    heading = 0.0
    
    (angles, distances, lidar_points, front, left, right) = sensor_reading(position, heading)
    gaps = find_gaps(angles, distances)
    print("Gaps (in degrees):", [math.degrees(gap) for gap in gaps])
