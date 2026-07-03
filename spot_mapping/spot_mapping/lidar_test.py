from rplidar import RPLidar
import matplotlib.pyplot as plt
import math

PORT_NAME = "COM6"
lidar = RPLidar(PORT_NAME, timeout=3, baudrate=115200)

plt.ion()
fig, ax = plt.subplots()

for scan in lidar.iter_scans():
    xs = []
    ys = []
    for _, angle, dist in scan:
        theta = math.radians(angle)

        x = dist * math.cos(theta)
        y = - dist * math.sin(theta)

        xs.append(x)
        ys.append(y)

    ax.cla()

    # 3. Redraw the new points
    ax.scatter(xs, ys, s=1)
    ax.axis("equal")

    # Fixed limits keep the plot from constantly bouncing/resizing
    ax.set_xlim(-4000, 4000)
    ax.set_ylim(-4000, 4000)

    # 4. Pause for a split second to force the window to update
    plt.pause(0.01)