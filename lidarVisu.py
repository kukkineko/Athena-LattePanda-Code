import lidar
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import signal
import util


class lidarVisu():
    def handler(signum, frame):
        del lid
        exit(1)

    def __init__(self):
        #init lidar
        self.lid = lidar.Lidar()

        signal.signal(signal.SIGINT, self.handler)

        # create a figure and set the title
        fig = plt.figure()
        #fig.canvas.set_window_title('Lidar Data')

        # create a subplot
        self.ax = fig.add_subplot(111)

        # set the x and y labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')

        # set the x and y limits
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])

        # get the initial x and y values
        x, y = self.lid.getGrid()

        # create a line object and set the initial data
        self.line, = self.ax.plot(x, y, 'ro', markersize=1)

        # create the animation object
        ani = animation.FuncAnimation(fig, self.update, interval=100)
        plt.show()

    def update(self, i):
        # get the new x and y values
        x, y = self.lid.getGrid()

        # update the line data
        self.line.set_xdata(x)
        self.line.set_ydata(y)

        # Save the points as an image using the create_image() function
        #util.create_image(x, y)

if __name__ == "__main__":
    lidar = lidarVisu()

