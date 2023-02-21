import numpy as np
import math
import argparse 

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--axis", help="specify unit vector that points in direction of axis of rotation in x,y,z")
    parser.add_argument("--theta", help="amount of degrees turning")
    args = parser.parse_args()

    axis_str = args.axis
    theta_str = args.theta

    axis = axis_str.split(',')
    if (not len(axis) == 3):
        print("invalid rotation")
        return
    axis = [float(i) for i in axis]
    axis = np.array(axis); 
    theta = float(theta_str)

    if (not np.linalg.norm(axis) == 1):
        axis = axis / np.linalg.norm(axis)

    q0 = math.cos(theta/2)
    q1 = axis[0] * math.sin(theta/2)
    q2 = axis[1] * math.sin(theta/2)
    q3 = axis[2] * math.sin(theta/2)

    print(q0,q1,q2,q3)
    # print('q0:', q0)
    # print('q1:', q1)
    # print('q2:', q2)
    # print('q3:', q3)

if __name__ == '__main__':
    main()