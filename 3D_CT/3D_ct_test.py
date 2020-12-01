import copy
import numpy as np
import open3d as o3d
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    
    #ORA_3d_axial = np.zeros((888,857,915))
    
    x = np.empty(0)
    y = np.empty(0)
    z = np.empty(0)
    
    x_list = x.tolist()
    y_list = y.tolist()
    z_list = z.tolist()
    
    for z_c in range(887):
        img = cv2.imread('../flower_CT_photo/ORA/[vg-data] ORA/volume_1/ORA-{0:03d}.tif'.format(z_c),0)
    
        #閾値設定
        threshold_value = 60
    
        for y_c in range(858):
            for x_c in range(915):
                if(img[y_c,x_c] >= threshold_value):
                    #ORA_3d_axial[z][y][x] = 1
                    x_list.append(x_c)
                    y_list.append(y_c)
                    z_list.append(z_c) 
                    
    x = np.asarray(x_list)
    y = np.asarray(y_list)
    z = np.asarray(z_list)
    
    #mesh_x, mesh_y, mesh_z = np.meshgrid(x, y, z)
    
    #xyz = np.zeros((np.size(mesh_x), 3))
    #xyz[:, 0] = np.reshape(mesh_x, -1)
    #xyz[:, 1] = np.reshape(mesh_y, -1)
    #xyz[:, 2] = np.reshape(mesh_z, -1)
    
    xyz = np.zeros((np.size(x), 3))
    xyz[:, 0] = np.reshape(x, -1)
    xyz[:, 1] = np.reshape(y, -1)
    xyz[:, 2] = np.reshape(z, -1)
    
    # pass xyz to Open3d.o3d.Geometry.PointCloud and visualize
    pcd = o3d.PointCloud()

    pcd.points = o3d.Vector3dVector(xyz)

    o3d.write_point_cloud("../output/ORA_axial.ply", pcd)