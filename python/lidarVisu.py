import pygame
import numpy as np
import signal
#from wallDetect import WallDetect
import lidar

class LidarVisu:
    def __init__(self):
        signal.signal(signal.SIGINT, self.handler)
        self.lidar = lidar.Lidar()

        # Set up Pygame
        pygame.init()
        self.window_size = (800, 800)
        self.screen = pygame.display.set_mode(self.window_size)
        pygame.display.set_caption('Lidar Visualization')
        self.clock = pygame.time.Clock()

        # Initialize view properties
        self.zoom = 1.0
        self.offset = np.array([0, 0], dtype=float)

        # Set point color and thickness
        self.point_color = (255, 0, 0)  # Red color for lidar points
        self.point_thickness = 2

        # Set grid color and thickness
        self.grid_color = (150, 150, 150)
        self.grid_thickness = 1

        # Create WallDetect object
        #self.wall_detector = WallDetect(num_walls=2)

    @staticmethod
    def handler(signum, frame):
        exit(1)

    def draw_grid(self):
        for x in range(-100, 110):
            x_pos = int((x + self.offset[0]) * 40 * self.zoom + 400)
            pygame.draw.line(self.screen, self.grid_color, (x_pos, 0), (x_pos, 800), self.grid_thickness)
        for y in range(-100, 110):
            y_pos = int((y + self.offset[1]) * 40 * self.zoom + 400)
            pygame.draw.line(self.screen, self.grid_color, (0, y_pos), (800, y_pos), self.grid_thickness)

    def update(self):
        lidar.scan()
        x = lidar.ranges_list * np.cos(lidar.angles_list)
        y = lidar.ranges_list * np.sin(lidar.angles_list)
        data = np.column_stack((x, y)).astype(float)

        self.screen.fill((255, 255, 255))
        self.draw_grid()

        # Draw lidar data points on the screen
        for i in range(len(x)):
            x_pos = int((x[i] + self.offset[0]) * 40 * self.zoom + 400)
            y_pos = int((y[i] + self.offset[1]) * 40 * self.zoom + 400)
            pygame.draw.circle(self.screen, self.point_color, (x_pos, y_pos), self.point_thickness)

        # Detect walls in lidar data
        #x_walls, y_walls = self.wall_detector.detect_walls_in_pointcloud(data)

        # Draw walls on the screen
        #for x_wall, y_wall in zip(x_walls, y_walls):
           #x1_pos = int((x_wall[0] + self.offset[0]) * 40 * self.zoom + 400)
            #y1_pos = int((y_wall[0] + self.offset[1]) * 40 * self.zoom + 400)
            #x2_pos = int((x_wall[1] + self.offset[0]) * 40 * self.zoom + 400)
            #y2_pos = int((y_wall[1] + self.offset[1]) * 40 * self.zoom + 400)
            #pygame.draw.line(self.screen, (0, 255, 0), (x1_pos, y1_pos), (x2_pos, y2_pos), 2)


        pygame.display.flip()

    def run(self):
        running = True
        dragging = False
        prev_mouse_pos = (0, 0)
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:  # Mouse wheel scroll up
                        self.zoom *= 1.2
                    elif event.button == 5:  # Mouse wheel scroll down
                        self.zoom /= 1.2
                    elif event.button == 1:  # Left mouse button click
                        dragging = True
                        prev_mouse_pos = event.pos
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:  # Left mouse button release
                        dragging = False
                elif event.type == pygame.MOUSEMOTION:
                    if dragging:
                        mouse_dx = event.pos[0] - prev_mouse_pos[0]
                        mouse_dy = event.pos[1] - prev_mouse_pos[1]
                        self.offset[0] += mouse_dx / (40 * self.zoom)
                        self.offset[1] += mouse_dy / (40 * self.zoom)
                        prev_mouse_pos = event.pos

            self.update()
            self.clock.tick(130)

        pygame.quit()

if __name__ == "__main__":
    lidar_visu = LidarVisu()
    lidar_visu.run()
