import pandas as pd
import numpy as np
import pyvista as pv
from stl import mesh


#df = pd.DataFrame(np.random.randint(0,100,size=(100, 3)), columns=list('XYZ'))

#df.to_csv('points.csv')

points = np.genfromtxt('points.csv', delimiter=",", dtype=np.float32)
point_cloud = pv.PolyData(points)
m = point_cloud.reconstruct_surface()
m.save('mesh.stl')
#point_cloud.plot(eye_dome_lighting=True)

stl_data = mesh.Mesh.from_file('mesh.stl')
stl_data.save('mesh_copied.stl')
stl_data