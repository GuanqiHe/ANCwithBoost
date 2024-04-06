import msgpack
import numpy as np
import matplotlib.pyplot as plt
import argparse
from scipy.io import savemat
import code


if __name__ == '__main__':

    import argparse

    # 1. 定义命令行解析器对象
    parser = argparse.ArgumentParser(
        description='plot ANC experiment msgpack data')

    parser.add_argument('-f', '--file', type=str,
                        default="output.bin", help="msgpack file")
    parser.add_argument('-xy', '--xy', nargs='*', type=str,
                        help='X,Y variable to plot')
    parser.add_argument('-s', '--save', type=str,
                        default="", help='save file as .mat')

    args = parser.parse_args()

    file = args.file
    name = args.xy
    fmat = args.save

    try:

        f = open(file, "rb")

        unpacker = msgpack.Unpacker(f)
        var_name = next(unpacker)
        print(var_name)
        value = next(unpacker)
        
        arr = np.array(value)

        data = {var_name[i]: arr[:, i] for i in range(len(var_name))}
        
        # print(data)

        f.close()

    except FileNotFoundError:
        raise Exception("Fail to open file")
    
    except StopIteration:
        raise Exception("Wrong data format")
    
    except Exception as e:
        raise e

    # print(data.keys())

    if fmat:
        mdic = {"data": data}
        savemat(fmat, mdic)

    if name:
        for var_name in name:
            x_name, y_name = var_name.split(",")

            try:
                x, y = data[x_name], data[y_name]
                plt.figure()
                plt.plot(x, y)
                plt.xlabel(x_name)
                plt.ylabel(y_name)
                plt.title(f"{x_name}-{y_name}")
            except KeyError:
                print(f"Fail to unpack X: {x_name}, Y: {y_name}")

        plt.show()
