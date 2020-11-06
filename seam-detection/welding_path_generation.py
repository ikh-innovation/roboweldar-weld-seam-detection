import numpy as np
import os

# TODO DEPRICATED********************
def get_dum_welding_paths():
    wpaths = []

    wpath1 = np.array([[0., 0., 0.], [0.1, 0., 0.], [0.2, 0., 0.], [0.3, 0., 0.]])
    wpaths.append(wpath1)
    wpath2 = np.array([[0., 0., 0.], [0., 0.1, 0.], [0., 0.1, 0.], [0., 0.1, 0.]])
    wpaths.append(wpath2)

    wpaths = np.array(wpaths)
    print(wpaths)

    return wpaths


def save_welding_paths(point_paths: np.ndarray, output_file="welding_paths.npy"):
    np.save(output_file, point_paths)


if __name__ == '__main__':
    wpaths = get_dum_welding_paths()
    save_welding_paths(wpaths)
