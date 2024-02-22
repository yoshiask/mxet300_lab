from time import sleep
from L1_encoder import readShaftPositions
from L1_log import tmpFile
from L2_kinematics import getMotion, getPdCurrent

if __name__ == "__main__":
    while True:
        phiDotL, phiDotR = getPdCurrent()
        xDot, thetaDot =  getMotion()
        print(f"φ_dotL: {phiDotL}")
        print(f"φ_dotR: {phiDotR}")
        print(f"x_dot: {xDot}")
        print(f"θ: {thetaDot}")
        print("\r\n")

        tmpFile(phiDotL, "Lab5_phiDotL.txt")
        tmpFile(phiDotR, "Lab5_phiDotR.txt")
        tmpFile(xDot, "Lab5_xDot.txt")
        tmpFile(thetaDot, "Lab5_thetaDot.txt")

        sleep(1)