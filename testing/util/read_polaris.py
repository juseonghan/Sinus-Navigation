import yaml
import argparse
import numpy as np

def main():

    # read in CLA 
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str)
    parser.add_argument('--save', type=str)
    args = parser.parse_args()

    # read in using pyyaml
    with open(args.data, "r") as f:
        try:
            y = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)

    # pose_data is a list of np arrays where each element of the list is
    # tx ty tz qx qy qz qw 
    poses = y['poses[]']
    num_frames = int(y['header']['num_frames'])
    pose_data = [0] * num_frames

    # loop through
    for pose_num, pose_dict in poses.items():
        # get frame number
        i1 = pose_num.find("[")
        i2 = pose_num.find("]")
        num = int(pose_num[i1+1:i2])
        
        translation = pose_dict['position']
        rotation = pose_dict['orientation']

        pose_data[num] = np.array([float(translation['x']), float(translation['y']), float(translation['z']), float(rotation['x']), float(rotation['y']), float(rotation['z']), float(rotation['w'])])
    
    # save list to file 
    save_data(args.save, pose_data)

def save_data(save_path, data):
    
    with open(save_path, "w") as f:
        for line in data:
            line_str = ''
            for val in line:
                line_str += str(val) + " "
            
            line_str += "\n"
            f.write(line_str)

    f.close()

if __name__ == '__main__':
    main()