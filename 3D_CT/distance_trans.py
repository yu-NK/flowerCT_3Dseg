import copy
import numpy as np
import open3d as o3d
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import distance
import math

#平方根を整数で返す関数
#def isqrt(n):
#    x = n
#    y = (x + 1) // 2
#   while y < x:
#        x = y
#        y = (x + n // x) // 2
#    return x

def min(a,b):
    if(a>b):
        return b
    else:
        return a

if __name__ == "__main__":
    
    ORA_3d_axial = np.zeros((888,858,915))
    
    x = np.empty(0)
    y = np.empty(0)
    z = np.empty(0)
    
    x_list = x.tolist()
    y_list = y.tolist()
    z_list = z.tolist()
    
    for z_c in range(888):
        img = cv2.imread('../../flower_CT_photo/ORA/[vg-data] ORA/volume_1/ORA-{0:03d}.tif'.format(z_c),0)
    
        #閾値設定
        threshold_value = 60
    
        for y_c in range(858):
            for x_c in range(915):
                if(img[y_c,x_c] >= threshold_value):
                    ORA_3d_axial[z_c][y_c][x_c] = 1
        
    #forward scan
    for z_c in range(888):
        for y_c in range(858):
            d_f = 914
            for x_c in range(915):
                if(ORA_3d_axial[z_c][y_c][x_c] >= 1):
                    d_f = d_f + 1
                else:
                    d_f = 0
                ORA_3d_axial[z_c][y_c][x_c] = d_f ** 2
    
    #backward scan
    for z_c in range(888):
        for y_c in range(858):
            d_f = 914
            for x_c in range(914,0,-1):
                if(ORA_3d_axial[z_c][y_c][x_c] >= 1):
                    d_b = d_b + 1
                else:
                    d_b = 0
                ORA_3d_axial[z_c][y_c][x_c] = min(ORA_3d_axial[z_c][y_c][x_c],d_b**2)
    
    #Execution of transformation 2
    buff1 = [0]*858
    
    for z_c in range(888):
        for x_c in range(915):
            for y_c in range(858):
                buff1[y_c] = ORA_3d_axial[z_c][y_c][x_c]
            for y_c in range(858):
                d = buff1[y_c]
                if(d>0):
                    rMax = math.floor(math.sqrt(d))+1
                    rStart = min(rMax,y_c-1)
                    rStart = -1 * rStart
                    rEnd = min(rMax, 857-y_c)
                    for n in range(rStart,rEnd):
                        w = buff1[y_c+n] + n**2
                        if(w<d):
                            d = w
                ORA_3d_axial[z_c][y_c][x_c] = d
    
    #Excution of transformation 3
    buff2 = [0]*888
    
    for y_c in range(858):
        for x_c in range(915):
            for z_c in range(888):
                buff2[z_c] = ORA_3d_axial[z_c][y_c][x_c]
            for z_c in range(888):
                d = buff2[z_c]
                if(d>0):
                    rMax = math.floor(math.sqrt(d))+1
                    rStart = min(rMax,z_c-1)
                    rStart = -1 * rStart
                    rEnd = min(rMax, 886-z_c)
                    for n in range(rStart,rEnd):
                        w = buff2[z_c+n] + n**2
                        if(w<d):
                            d = w
                ORA_3d_axial[z_c][y_c][x_c] = d
    
    for z_c in range(887):
        for y_c in range(857):
            for x_c in range(914):
                if((ORA_3d_axial[z_c][y_c][x_c] < 30) & (ORA_3d_axial[z_c][y_c][x_c] > 0)):
                    x_list.append(x_c)
                    y_list.append(y_c)
                    z_list.append(z_c)

                    
    x = np.asarray(x_list)
    y = np.asarray(y_list)
    z = np.asarray(z_list)
    
    xyz = np.zeros((np.size(x), 3))
    xyz[:, 0] = np.reshape(x, -1)
    xyz[:, 1] = np.reshape(y, -1)
    xyz[:, 2] = np.reshape(z, -1)
    
    # pass xyz to Open3d.o3d.Geometry.PointCloud and visualize
    pcd = o3d.PointCloud()
    
    pcd.points = o3d.Vector3dVector(xyz)

    o3d.write_point_cloud("../../output/ORA_axial_0_30.ply", pcd)