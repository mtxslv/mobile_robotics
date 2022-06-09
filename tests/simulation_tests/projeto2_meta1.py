
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection

import os
import sys
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '../../src/') ))

from configuration_space_mapping import *
from utils import *


# conectando ao simulador
client_id = connect_2_sim()
test_connection(client_id)

# indicando o nome dos objetos em cena
scene_objects = ['Cuboid_0','Cuboid_1','Cuboid_2',
                 'Cuboid_3','Cuboid_4','./dr20']
robot_name = './dr20'

# calculo do mapeamento
patches = mapping(client_id=client_id, scene_objects=scene_objects, robot_name=robot_name)

# onde o robo esta
robot_handle = get_object_handle(client_id,robot_name)
position_robot,_ = get_configuration(client_id, robot_handle)

# preparação do gráfico
nfloors = len(patches)
cmap = plt.get_cmap('RdYlBu')
colors = cmap(nfloors) # convert nfloors to colors that we can use later

fig,ax = plt.subplots(1)
collection = PatchCollection(patches)
ax.add_collection(collection)
ax.scatter(position_robot[0],position_robot[1])
collection.set_color(colors)
ax.autoscale_view()
plt.show()