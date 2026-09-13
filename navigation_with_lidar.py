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
RMIN = 0.5
DT = 0.5

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

# rectangles = [
#     (2.5, 3.0, 1.0, 5.0),
#     (5.0, 1.5, 5.0, 1.0),
#     (5.5, 5.0, 1.0, 5.0),
#     (8.0, 9.0, 4.0, 1.0),
#     (10.5, 3.0, 1.0, 4.0),
# ]

# trees = [
#     (4.0, 11.5, 0.6),
#     (7.5, 3.5, 0.6),
#     (9.0, 7.0, 0.6),
#     (12.5, 12.0, 0.6),
#     (3.0, 13.0, 0.6),
# ]

# ============================================================
# 3. HELPER FUNCTIONS
# ============================================================
def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def world_to_screen(x, y):
    sx = int(x * SCALE)
    sy = int(HEIGHT - y * SCALE)

    return sx, sy

def point_inside_obstacle(x, y):
    for rx, ry, width, height in rectangles:
        if (rx <= x <= rx + width and ry <= y <= ry + height):
            return True

    for cx, cy, radius in trees:
        distance_squared = ((x - cx) ** 2 + (y - cy) ** 2)

        if distance_squared <= radius ** 2:
            return True

    return False

# ============================================================
# 4. SENSOR READING
# ============================================================
def sensor_reading(position, heading):
    x0, y0 = position
    distances = []
    points = []
    relative_angles = np.linspace(0, 2 * math.pi, NUM_RAYS, endpoint=False)

    for relative_angle in relative_angles:
        if relative_angle < 135 * math.pi / 180 or relative_angle > 225 * math.pi / 180:
            world_angle = heading + relative_angle
            dx = math.cos(world_angle)
            dy = math.sin(world_angle)
            detected_distance = MAX_RANGE

            for distance in np.arange(0, MAX_RANGE, RAY_STEP):
                x = x0 + distance * dx
                y = y0 + distance * dy

                if (x < 0 or x > MAP_X or y < 0 or y > MAP_Y):
                    detected_distance = distance
                    break

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
            px = x0 + SMALL_RANGE * math.cos(heading + relative_angle)
            py = y0 + SMALL_RANGE * math.sin(heading + relative_angle)
            points.append([px, py])

    distances = np.array(distances)
    points = np.array(points)

    def sector_min(center_deg, width_deg):
        angle_error = np.array([abs(math.degrees(wrap_angle(a - math.radians(center_deg))))
            for a in relative_angles
        ])

        valid = angle_error <= width_deg
        if np.any(valid):
            return np.min(distances[valid])

        return MAX_RANGE

    front = sector_min(0, 15)
    left = sector_min(90, 20)
    right = sector_min(-90, 20)

    return relative_angles, distances, points, front, left, right

# ============================================================
# 5. MANUAL CONTROL
# ============================================================
def manual_control(position, heading, keys):
    new_position = position.copy()
    new_heading = heading
    velocity = 0

    if keys[pygame.K_LEFT]:
        new_heading += TURN_SPEED

    if keys[pygame.K_RIGHT]:
        new_heading -= TURN_SPEED

    if keys[pygame.K_UP]:
        velocity = BOAT_SPEED

    if keys[pygame.K_DOWN]:
        velocity = -BOAT_SPEED

    new_x = (new_position[0] + velocity * math.cos(new_heading))
    new_y = (new_position[1] + velocity * math.sin(new_heading))

    # Collision prevention
    if (0 <= new_x <= MAP_X and 0 <= new_y <= MAP_Y and not point_inside_obstacle(new_x, new_y)):
        new_position[0] = new_x
        new_position[1] = new_y

    return new_position, new_heading

# ============================================================
# 6. AUTONOMOUS CONTROL
# ============================================================
def calculate_gaps(angles, distances, heading, position):
    gaps = []
    results = []
    start = None
    free = distances >= MAX_RANGE
    dx = GOAL[0] - position[0]
    dy = GOAL[1] - position[1]
    
    goal_angle_world = math.atan2(dy, dx)
    goal_angle = wrap_angle(goal_angle_world - heading)
    
    relative_angles = np.array([wrap_angle(a) for a in angles])
    order = np.argsort(relative_angles)
    
    relative_angles = relative_angles[order]
    distances_sorted = distances[order]
    free = free[order]

    for i, is_free in enumerate(free):
        if is_free and start is None:
            start = i
        elif not is_free and start is not None:
            gaps.append((start, i - 1))
            start = None

    if start is not None:
        gaps.append((start, len(free) - 1))

    # Handle a gap crossing 360° → 0°
    if len(gaps) > 1 and gaps[0][0] == 0 and gaps[-1][1] == len(free) - 1:
        gaps[0] = (gaps[-1][0], gaps[0][1])
        gaps.pop()

    for start_idx, end_idx in gaps:
        start_angle = relative_angles[start_idx]
        end_angle = relative_angles[end_idx]
        gap_angle = abs(end_angle - start_angle)
        center_angle = (start_angle + end_angle) / 2
        start_distance = distances_sorted[start_idx]
        end_distance = distances_sorted[end_idx]
        gap_width = math.sqrt(start_distance ** 2 + end_distance ** 2 - 2 * start_distance * end_distance * math.cos(gap_angle))

        MIN_GAP_WIDTH = 1.254
        if gap_width < MIN_GAP_WIDTH:
            continue

        angular_error = abs(wrap_angle(center_angle - goal_angle))
        angular_error_norm = angular_error / math.pi
        width_score = 1.0 / (gap_width + 0.01)
        width_score = min(width_score, 1.0)
        average_distance = (start_distance + end_distance) / 2
        distance_score = 1.0 / (average_distance + 0.1)
        distance_score = min(distance_score, 1.0)
        
        W_ANGLE = 0.70
        W_WIDTH = 0.20
        W_DISTANCE = 0.10
        
        # print(f"angular_error_norm: {angular_error_norm:.4f}, width_score: {width_score:.4f}, distance_score: {distance_score:.4f}, goal_angle: {math.degrees(goal_angle):.2f}°")

        cost = W_ANGLE * angular_error_norm + W_WIDTH * width_score + W_DISTANCE * distance_score

        results.append({
            "start_angle": start_angle,
            "end_angle": end_angle,
            "center_angle": center_angle,
            "angular_width": gap_angle,
            "start_distance": start_distance,
            "end_distance": end_distance,
            "gap_width": gap_width,
            "goal_error": angular_error,
            "cost": cost
        })

    results.sort(key=lambda g: g["cost"])

    return results

def curvature_navigation(gaps, heading, position):

    if not gaps:
        return 0.0, 0.0, None

    dx, dy = GOAL - position
    goal_angle = wrap_angle(math.atan2(dy, dx) - heading)

    best_gap = gaps[0]
    gap_angle = best_gap["center_angle"]

    # Blend gap direction with goal direction
    target_angle = wrap_angle(0.7 * gap_angle + 0.3 * goal_angle)

    # Look-ahead distance
    LOOKAHEAD = 1.0

    # Pure-pursuit curvature
    curvature = 2.0 * math.sin(target_angle) / LOOKAHEAD

    # Curvature constraint
    MAX_CURVATURE = 1.0 / RMIN
    curvature = np.clip(curvature, -MAX_CURVATURE, MAX_CURVATURE)

    # Forward speed decreases during sharp turns
    speed = BOAT_SPEED * (1.0 - 0.7 * min(abs(curvature) / MAX_CURVATURE, 1.0))

    return curvature, speed, best_gap

def update_position(position, heading, curvature, speed):
    x, y = position

    omega = speed * curvature

    if abs(omega) < 1e-6:

        x += speed * math.cos(heading) * DT
        y += speed * math.sin(heading) * DT

    else:

        R = 1.0 / curvature

        x += R * (
            math.sin(heading + omega * DT) -
            math.sin(heading)
        )

        y -= R * (
            math.cos(heading + omega * DT) -
            math.cos(heading)
        )

        heading += omega * DT

    heading = wrap_angle(heading)

    return np.array([x, y]), heading

# ============================================================
# 7. DRAW ENVIRONMENT
# ============================================================
def draw_environment(screen, position, heading, lidar_points, mode, front, left, right):
    screen.fill((20, 20, 20))

    # Draw obstacles
    for rx, ry, width, height in rectangles:
        pygame.draw.rect(screen, (20, 20, 20), pygame.Rect(int(rx * SCALE), int(HEIGHT - (ry + height) * SCALE), int(width * SCALE), int(height * SCALE)))
        # pygame.draw.rect(screen, (200, 200, 200), pygame.Rect(int(rx * SCALE), int(HEIGHT - (ry + height) * SCALE), int(width * SCALE), int(height * SCALE)))

    for cx, cy, radius in trees:
        pygame.draw.circle(screen, (20, 20, 20), world_to_screen(cx, cy), int(radius * SCALE))
        # pygame.draw.circle(screen, (200, 200, 200), world_to_screen(cx, cy), int(radius * SCALE))

    # Draw LiDAR point cloud
    for point in lidar_points:
        px, py = world_to_screen(point[0], point[1])
        bx, by = world_to_screen(position[0], position[1])

        # Line from LiDAR center to detected point
        pygame.draw.line(screen, (0, 255, 100), (bx, by), (px, py), 1)
        pygame.draw.circle(screen, (0, 255, 100), (px, py), 2)

    # Draw boat
    bx, by = world_to_screen(position[0], position[1])

    pygame.draw.circle(screen, (255, 60, 60), (bx, by), 8)

    # Heading line
    hx = (position[0] + 0.5 * math.cos(heading))
    hy = (position[1] + 0.5 * math.sin(heading))
    hx, hy = world_to_screen(hx, hy)

    pygame.draw.line(screen, (255, 200, 50), (bx, by), (hx, hy), 3)

    # Draw goal
    gx, gy = world_to_screen(GOAL[0], GOAL[1])

    pygame.draw.circle(screen, (255, 50, 50), (gx, gy), 10, 3)
    pygame.draw.line(screen, (255, 50, 50), (gx - 8, gy), (gx + 8, gy), 3)
    pygame.draw.line(screen, (255, 50, 50), (gx, gy - 8), (gx, gy + 8), 3)

    # Information
    font = pygame.font.SysFont(None, 24)

    if mode == "MANUAL":
        mode_text = "MODE: MANUAL"
    elif mode == "GOAL":
        mode_text = "GOAL REACHED"
    else:
        mode_text = "MODE: AUTONOMOUS"

    text1 = font.render(mode_text, True, (255, 255, 255))
    text2 = font.render("Press S = switch mode", True, (200, 200, 200))
    text3 = font.render(f"Front: {front:.2f} m", True, (200, 200, 200))
    text4 = font.render(f"Left: {left:.2f} m", True, (200, 200, 200))
    text5 = font.render(f"Right: {right:.2f} m", True, (200, 200, 200))

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
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("LiDAR Boat Navigation Simulation")
    clock = pygame.time.Clock()

    # Initial boat position
    position = np.array([2.0, 2.0], dtype=float)
    heading = 0.0

    # Modes
    MANUAL = 0
    AUTONOMOUS = 1

    control_mode = MANUAL

    GO_TO_GOAL = 0

    auto_mode = GO_TO_GOAL
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    if control_mode == MANUAL:
                        control_mode = AUTONOMOUS
                        auto_mode = GO_TO_GOAL
                    else:
                        control_mode = MANUAL

        # SENSOR READING
        angles, distances, lidar_points, front, left, right = sensor_reading(position, heading)

        # CONTROL
        if control_mode == MANUAL:
            keys = pygame.key.get_pressed()
            position, heading = manual_control(position, heading, keys)
            display_mode = "MANUAL"
        else:
            proximity_to_goal = np.linalg.norm(GOAL - position)
            
            if proximity_to_goal <= GOAL_TOLERANCE:
                auto_mode = "GOAL"
            else:
                gaps = calculate_gaps(angles, distances, heading, position)
                curvature, speed, _ = curvature_navigation(gaps, heading, position)
                position, heading = update_position(position, heading, curvature, speed)

            if auto_mode == "GOAL":
                display_mode = "GOAL"
            else:
                display_mode = "AUTONOMOUS"

        # DRAW
        draw_environment(screen, position, heading, lidar_points, display_mode, front, left, right)

        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit()

# ============================================================
# RUN
# ============================================================
if __name__ == "__main__":
    main()
    
    position = np.array([5.8, 4.2], dtype=float)
    heading = 3
    
    (angles, distances, lidar_points, front, left, right) = sensor_reading(position, heading)
    # gaps = find_gaps(angles, distances)
    gaps = calculate_gaps(angles, distances, heading, position)
    for gap in gaps:
        sa = np.degrees(gap["start_angle"])
        ea = np.degrees(gap["end_angle"])
        ca = np.degrees(gap["center_angle"])
        aw = np.degrees(gap["angular_width"])
        sd = gap["start_distance"]
        ed = gap["end_distance"]
        gw = gap["gap_width"]
        ge = np.degrees(gap["goal_error"])
        c = gap["cost"]

        print(f"Start Angle: {sa:.2f}°, End Angle: {ea:.2f}°, Center Angle: {ca:.2f}°, Angular Width: {aw:.2f}°, Start Distance: {sd:.2f} m, End Distance: {ed:.2f} m, Gap Width: {gw:.2f} m, Goal Error: {ge:.2f}°, Cost: {c:.4f}")
    