import numpy as np
import open3d
import open3d as o3d
from bpa import BPA

if 1:
    bpa = BPA(path='data/bunny_with_normals.txt', radius=0.005, visualizer=True)
    bpa.create_mesh(limit_iterations=10)#1000

a=np.array((3,2))
print(a)
#-----------------

draw_geometry=0

if draw_geometry:
    mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0,
                                                    height=1.0,
                                                    depth=1.0)
    mesh_box.compute_vertex_normals()
    mesh_box.paint_uniform_color([0.9, 0.1, 0.1])
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    mesh_sphere.compute_vertex_normals()
    mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
    mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3,
                                                              height=4.0)
    mesh_cylinder.compute_vertex_normals()
    mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.6, origin=[-2, -2, -2])


    print("We draw a few primitives using collection.")
    o3d.visualization.draw_geometries(
        [mesh_box, mesh_sphere, mesh_cylinder, mesh_frame])

#print("We draw a few primitives using + operator of mesh.")
#o3d.visualization.draw_geometries(
#    [mesh_box + mesh_sphere + mesh_cylinder + mesh_frame])
#----------------

test_random_plot_3d=0

if test_random_plot_3d:
    #http://www.open3d.org/docs/release/python_api/open3d.utility.Vector3dVector.html
    pcd = open3d.geometry.PointCloud()
    np_points = np.random.rand(100, 3)

    # From numpy to Open3D
    pcd.points = open3d.utility.Vector3dVector(np_points)

    # From Open3D to numpy
    np_points = np.asarray(pcd.points)



    open3d.visualization.draw_geometries([pcd],width=3000,height=3000)#,
                                      #zoom=0.3412,
                                      #front=[0.4257, -0.2125, -0.8795],
                                      #lookat=[2.6172, 2.0475, 1.532],
                                      #up=[-0.0694, -0.9768, 0.2024])


plot_sphere=0

if plot_sphere:
    # draw sphere
    u, v = np.mgrid[0:2*np.pi:50j, 0:np.pi:50j]
    x = np.cos(u)*np.sin(v)
    y = np.sin(u)*np.sin(v)
    z = np.cos(v)

    x=np.expand_dims(x.flatten(),axis=1)
    y=np.expand_dims(y.flatten(),axis=1)
    z=np.expand_dims(z.flatten(),axis=1)

    npnew=x
    npnew = np.hstack((npnew,y))
    npnew = np.hstack((npnew,z))

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(npnew)
    open3d.visualization.draw_geometries([pcd],width=1000,height=1000)#,
                                      #zoom=0.3412,
                                      #front=[0.4257, -0.2125, -0.8795],
                                      #lookat=[2.6172, 2.0475, 1.532],
                                      #up=[-0.0694, -0.9768, 0.2024])