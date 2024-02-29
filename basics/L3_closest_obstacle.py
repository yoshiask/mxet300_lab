from time import sleep
from L2_vector import getNearest, polar2cart
from L1_log import tmpFile

if __name__ == "__main__":
    while True:
        r, alpha = getNearest()
        tmpFile(r, "Lab6_closest_distance.txt")
        tmpFile(alpha, "Lab6_closest_angle.txt")

        x, y = polar2cart(r, alpha)
        tmpFile(x, "Lab6_closest_x.txt")
        tmpFile(y, "Lab6_closest_y.txt")

        sleep(1)