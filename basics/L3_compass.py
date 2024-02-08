from L2_compass_heading import get_heading
from L1_log import *
import time

if __name__ == "__main__":    
    while True:
        heading = get_heading()
        tmpFile(heading, "Lab3_heading.txt")

        directions = [
            "South", "South West",
            "West", "North West",
            "North", "North East",
            "East", "South East"
        ]
        index = int(-(heading + 180) / 45)
        direction = directions[index]
        stringTmpFile(direction, "Lab3_direction.txt")

        time.sleep(1)