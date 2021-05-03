import open3d as o3d
import numpy as np
from bpa import BPA
import time
from point import Point

if __name__ == "__main__":
    f = open("stanford-bunny.file")
    lines = f.readlines()
    f.close()

    points = []
    points_and_normals = {}
    facets = []
    normals = []

    for line in lines:
        splitted = line.split(" ")

        for i, element in enumerate(splitted):
            if "\n" in element:
                splitted[i] = element[:-1]

        if splitted != [] and 'v' in splitted[0]:
            p = Point(x=float(splitted[1])*1000, y=float(splitted[2])*1000, z=float(splitted[3])*1000, id=1)
            points.append(p)

        if splitted != [] and 'f' in splitted[0]:
            p = Point(x=int(splitted[1]), y=int(splitted[2]), z=int(splitted[3]), id=1)
            facets.append(p)

    for facet in facets:
        p1 = points[int(facet.x)-1]
        p2 = points[int(facet.y)-1]
        p3 = points[int(facet.z)-1]

        v1 = [p2.x - p1.x, p2.y - p1.y, p2.z - p1.z]
        v2 = [p1.x - p3.x, p1.y - p3.y, p1.z - p3.z]
        normal = np.cross(v1, v2)

        if p1 not in points_and_normals.keys():
            points_and_normals[p1] = normal
        if p2 not in points_and_normals.keys():
            points_and_normals[p2] = normal
        if p3 not in points_and_normals.keys():
            points_and_normals[p3] = normal

    f = open("output.txt", 'w')

    for k, v in points_and_normals.items():
        f.write(str(k.x) + " " + str(k.y) + " " + str(k.z) + " " + str(v[0]) + " " + str(v[1]) + " " + str(v[2]) + "\n")

    f.close()










