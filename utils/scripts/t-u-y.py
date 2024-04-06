import msgpack
import numpy as np
import matplotlib.pyplot as plt
import argparse

if __name__ == '__main__':
    
    
    f = open("build/bin/u-y.bin","rb")
    uy = msgpack.load(f)
    f.close()
    
    f = open("build/bin/d-y.bin","rb")
    dy = msgpack.load(f)
    f.close()


    print(uy.keys(), dy.keys())

    
    plt.plot(uy['t'], uy['y'])
    plt.plot(dy['t'], dy['y'])
        
    plt.show()
        
        
