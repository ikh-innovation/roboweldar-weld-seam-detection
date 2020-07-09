import pyvista as pv
from pyvista import examples
import numpy as np


plotter = pv.Plotter()
surf = pv.read("grass.obj")

plotter.add_mesh(surf, show_edges = True, color = 'gray')

side_size = surf.bounds[1] - surf.bounds[0]

surf2 = surf.copy()
surf2.rotate_x(90)
surf2.translate([0,surf.bounds[0]+40, 100 ])
surf2.clean()
plotter.add_mesh(surf2, color = 'gray')


surf3 = surf.copy()
surf3.rotate_y(90)
surf3.translate([surf.bounds[0]+40, 0, 100])
surf3.clean()
plotter.add_mesh(surf3, color = 'gray')


# inter = surf2.select_enclosed_points(surf3, check_surface=False)
# print(inter)
# plotter.add_mesh(inter, show_edges = True, color = 'red')
#

plotter.show()

mesh = surf + surf2 + surf3

mesh.triangulate(inplace=True)
mesh.compute_normals(inplace=True)
pv.save_meshio("welding_area.obj", mesh, "obj")