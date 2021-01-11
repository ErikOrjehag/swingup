
import numpy as np
from matplotlib import pyplot as plt

if __name__ == '__main__':

    # Create figure
    fig, ax = plt.subplots(1)
    fig.show()

    # Set x/y limits
    ax.set_xlim([0, 1])
    ax.set_ylim([-1, 1])
    fig.show()

    # Create line
    y = np.zeros(100)
    t = np.linspace(0, 1, len(y))
    line = ax.plot(t, y, 'r', animated=True)[0]
    
    # Manditory draw for some reason
    fig.canvas.draw()

    # Fetch background
    bg = fig.canvas.copy_from_bbox(ax.bbox)


    for i in range(1000):
        # Clear screen
        fig.canvas.restore_region(bg)
        
        # Update line
        y = np.array([np.sin(ti+0.01*i) for ti in t])
        line.set_ydata(y)

        # Draw line
        ax.draw_artist(line)

        # Blit screen
        fig.canvas.blit(ax.bbox)