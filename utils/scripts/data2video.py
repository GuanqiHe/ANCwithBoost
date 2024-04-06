import matplotlib.pyplot as plt

import math

from matplotlib.animation import FuncAnimation

 

def circle_coords(r, step):

   coords = []

   t = 0

   while t < 2 * math.pi:

        coords.append((r*math.cos(t),r*math.sin(t)))

        t += step

    
   return coords

 

coords = circle_coords(1.5, 0.1)

from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()

ax.scatter(x=coords[0][0],y=coords[0][1],c='red', marker = 'o')
 

def update(i):

   ax.clear()

   ax.set_facecolor(plt.cm.Blues(.2))

   

   ax.set_xlim([-2,2])

   ax.set_ylim([-2,2])

   ax.set_title('circling')

   ax.scatter(x=coords[i][0],y=coords[i][1],c='red',marker='o')

   [spine.set_visible(False) for spine in ax.spines.values()] #移除图表轮廓
   
   
   
fig, ax =plt.subplots(figsize=(6,6))

anime = FuncAnimation(

   fig = fig,

   func = update,

   frames = len(coords),

   interval = 50

)

 

anime.save('circle.gif')