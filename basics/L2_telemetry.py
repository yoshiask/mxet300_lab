from L1_ina import readVolts
from L1_log import tmpFile
import time

if __name__ == "__main__":    
    while True:
        volts = round(readVolts(), 2)      # collect a reading
        tmpFile(volts, "Lab2_voltage.txt") # save the reading
        time.sleep(1)                      # pause