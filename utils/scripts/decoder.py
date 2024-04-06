import msgpack
import numpy as np
import matplotlib.pyplot as plt

f = open("output.bin","rb")
data = msgpack.load(f)
f.close()

var_name = data['state_name']
data = np.array(data['state_data'])
print(data.shape)

t = data[:,0]
theta1 = data[:,1]
dtheta1 = data[:,2]
x1 = data[:,3]
x2 = data[:,4]

# plt.plot(t,theta1,t,dtheta1)
plt.plot(t,-10*x1+10*x2)
plt.show()
