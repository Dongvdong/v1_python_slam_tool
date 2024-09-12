'''
gnss 和 enu 坐标系相互转化

'''
# pip install pyproj

import numpy as np
from pyproj import Proj, Transformer
import pyproj
import math
            
from geographiclib.geodesic import Geodesic
import math




#from API_1GetGpsFromIMG import *




use_cgcs2000Towgs84=0 # 大疆采集的rtk默认坐标系是cgcs2000Towgs84 是否需要转化 貌似转化没啥区别
# # WGS-84定义的常数，用于CGCS2000系统（与WGS-84非常接近）
# 1-1 
def Api_cgcs2000Towgs84(Gnss_in):
    # 定义CGCS2000和WGS-84坐标系   
    cgcs2000 = Proj('epsg:4490')  # CGCS2000的EPSG代码
    wgs84 = Proj('epsg:4326')     # WGS-84的EPSG代码

    # 使用Transformer进行转换
    transformer = Transformer.from_proj(cgcs2000, wgs84, always_xy=True)

    # 示例坐标（经度, 纬度, 高度）
    #lon, lat, h = 116.391, 39.907, 50.0  # 高度为50米
    lon, lat, h = Gnss_in[1], Gnss_in[0], Gnss_in[2]  # 高度为50米
    
    Gnss_out=[-1,-1,-1]
    # 进行坐标转换
    x, y, z = transformer.transform(lon, lat, h)

    Gnss_out=[y,x,z]

    #print(f"输入 CGCS2000坐标: 经度={lon}, 纬度={lat}, 高度={h}")
    #print(f"输出   WGS-84坐标: 经度={x}, 纬度={y}, 高度={z}")
    return Gnss_out
# 1-2
def Api_wgs84Tocgcs2000(Gnss_in):
    # 定义CGCS2000和WGS-84坐标系
    cgcs2000 = Proj('epsg:4490')  # CGCS2000的EPSG代码
    wgs84 = Proj('epsg:4326')     # WGS-84的EPSG代码

    # 使用Transformer进行转换
    transformer = Transformer.from_proj(wgs84,cgcs2000 , always_xy=True)

    # 示例坐标（经度, 纬度, 高度）
    #lon, lat, h = 116.391, 39.907, 50.0  # 高度为50米
    lon, lat, h = Gnss_in[1], Gnss_in[0], Gnss_in[2]  # 高度为50米

    # 进行坐标转换
    x, y, z = transformer.transform(lon, lat, h)
    Gnss_out=[y,x,z]
    #print(f"输入   WGS-84坐标: 经度={x}, 纬度={y}, 高度={z}")
    #print(f"输出 CGCS2000坐标: 经度={lon}, 纬度={lat}, 高度={h}")
    return Gnss_out
    
#=============================================================
# WGS-84定义的常数，用于CGCS2000系统（与WGS-84非常接近）
a = 6378137.0  # 长半轴（单位：米）
b = 6356752.314245
#f = (a - b) / a
#f = 1 / 298.257223563  # 扁率  CGCS2000系统
f = 1 / 298.257223565  # 扁率  WGS-84
e2 = 2*f - f**2  # 第一偏心率的平方

pi = 3.14159265359
# 2-1-1 gps转换到ecef
def gnss_to_ecef(lat, lon, h):
    """将地理坐标（经度、纬度、高程）转换为ECEF坐标系"""
    lat = np.radians(lat)
    lon = np.radians(lon)
    N = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    X = (N + h) * np.cos(lat) * np.cos(lon)
    Y = (N + h) * np.cos(lat) * np.sin(lon)
    Z = (N * (1 - e2) + h) * np.sin(lat)
    return X, Y, Z


# 2-1-2 gps转换到ecef
def gnss_to_ecef1(lat_ref,lon_ref,h_ref):
    transformer = pyproj.Transformer.from_crs(
    {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},    
    {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
    )

    x_ref, y_ref, z_ref = transformer.transform(lon_ref, lat_ref, h_ref ,radians=False)
    
    to_ecef=[x_ref,y_ref,z_ref]
    return to_ecef

#2-2 

'''
功能： # 大地坐标系ECEF转化到gps
输入：
等待转换的ecef  坐标 x, y, z 
输出：
GPS   坐标 lat, lon, h

'''
def ecef_to_gnss(x, y, z):
    x=float(x)
    y=float(y)
    z=float(z)
   # Convert from ECEF cartesian coordinates to 
   # latitude, longitude and height.  WGS-84
    x2 = x ** 2 
    y2 = y ** 2 
    z2 = z ** 2 

    #a = 6378137.0000    # earth radius in meters
    #b = 6356752.3142    # earth semiminor in meters 
    e = math.sqrt (1-(b/a)**2) 
    b2 = b*b 
    e2 = e ** 2 
    ep = e*(a/b) 
    r = math.sqrt(x2+y2) 
    r2 = r*r 
    E2 = a ** 2 - b ** 2 
    F = 54*b2*z2 
    G = r2 + (1-e2)*z2 - e2*E2 
    c = (e2*e2*F*r2)/(G*G*G) 
    s = ( 1 + c + math.sqrt(c*c + 2*c) )**(1/3) 
    P = F / (3 * (s+1/s+1)**2 * G*G) 
    Q = math.sqrt(1+2*e2*e2*P) 
    ro = -(P*e2*r)/(1+Q) + math.sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) 
    tmp = (r - e2*ro) ** 2 
    U = math.sqrt( tmp + z2 ) 
    V = math.sqrt( tmp + (1-e2)*z2 ) 
    zo = (b2*z)/(a*V) 

    height = U*( 1 - b2/(a*V) ) 
    
    lat = math.atan( (z + ep*ep*zo)/r ) 

    temp = math.atan(y/x) 
    if x >=0 :    
        long = temp 
    elif (x < 0) & (y >= 0):
        long = pi + temp 
    else :
        long = temp - pi 

    lat0 = lat/(pi/180) 
    lon0 = long/(pi/180) 
    h0 = height 

    return lat0, lon0, h0


def ecef_to_gnss_1(x,y,z):
    transformer = pyproj.Transformer.from_crs(
    {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
    {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'}, 
    )
   
    lon, lat, h= transformer.transform(x, y, z )
    
    to_gnss=[lat,lon, h]

    #print(f"从          ENU坐标: 东={east}, 北={north}, 高={up}")
    #print(f"转换后的CGCS2000坐标: 纬度={lat}, 经度={lon},  高度={h}")
    return to_gnss



# 3-1 ecef转换到enu
'''
功能： # 大地坐标系 转化到GPS第一帧为原点的本地ENU坐标系
输入：
等待转换的ecef       坐标 x, y, z 
作为原点的GPS第一帧   坐标lat0, lon0, h0
输出：
本地第一帧GPS为原点的 ENU 坐标系 xEast, yNorth, zUp

'''
def ecef_to_enu(X, Y, Z, lat_ref, lon_ref, h_ref):
    """将ECEF坐标转换为ENU坐标"""

    # 参考点的ECEF坐标
    Xr, Yr, Zr = gnss_to_ecef(lat_ref, lon_ref, h_ref)
    
    # ECEF到ENU的旋转矩阵
    lat_ref = np.radians(lat_ref)
    lon_ref = np.radians(lon_ref)
    
    R = np.array([
        [-np.sin(lon_ref), np.cos(lon_ref), 0],
        [-np.sin(lat_ref)*np.cos(lon_ref), -np.sin(lat_ref)*np.sin(lon_ref), np.cos(lat_ref)],
        [np.cos(lat_ref)*np.cos(lon_ref), np.cos(lat_ref)*np.sin(lon_ref), np.sin(lat_ref)]
    ])
    


    # ECEF坐标差
    dX = X - Xr
    dY = Y - Yr
    dZ = Z - Zr
    
    # 计算ENU坐标
    enu = R @ np.array([dX, dY, dZ])
    return enu





# 3-2 enu转换到ecef

'''
功能： enu坐标转化到ecef坐标
输入：
等待转换的ENU坐标   坐标 xEast, yNorth, zUp
GPS第一帧原点      坐标 lat0, lon0, h0
输出：
ecef  坐标 x, y, z 
'''
def enu_to_ecef(east, north, up, lat_ref, lon_ref, h_ref):

 
    # 1 参考GNSS点 转化到ecef
    # 定义参考点的CGCS2000坐标（经度, 纬度, 高度）
    #lon_ref, lat_ref, h_ref = 116.391, 39.907, 50.0  # 示例参考点
    ref_ecef=gnss_to_ecef(lat_ref,lon_ref,h_ref)

    ecef_x_ref=ref_ecef[0]
    ecef_y_ref=ref_ecef[1]
    ecef_z_ref=ref_ecef[2]

    # 2 等待转换的enu点变换到到ecef坐标系下相对位移
    # 将参考点的地理坐标转换为弧度
    lat_ref = np.radians(lat_ref)
    lon_ref = np.radians(lon_ref)

    # ENU到ECEF的旋转矩阵
    
    R = np.array([
        [-np.sin(lon_ref), np.cos(lon_ref), 0],
        [-np.sin(lat_ref)*np.cos(lon_ref), -np.sin(lat_ref)*np.sin(lon_ref), np.cos(lat_ref)],
        [np.cos(lat_ref)*np.cos(lon_ref), np.cos(lat_ref)*np.sin(lon_ref), np.sin(lat_ref)]
    ])

    # 将ENU坐标转换为ECEF坐标
    # 定义ENU坐标（East, North, Up）
    #east, north, up = 100, 200, 30  # 示例ENU坐标
    enu_vector = np.array([east, north, up])
    ecef_vector = R.T @ enu_vector  # 使用矩阵转置进行旋转


    # 将ECEF坐标添加到参考点的ECEF坐标
    x = ecef_x_ref + ecef_vector[0]
    y = ecef_y_ref + ecef_vector[1]
    z = ecef_z_ref + ecef_vector[2]


    return x,y,z




# 4-1 将一个gps转换到enu
'''
功能： # gps直接转化到enu坐标系
       相对于指定GPS_ref为原点（一般都是第一帧）的enu坐标系
输入：
gnss_in   等待转换的GPS   坐标 lat, lon, h
gnss_ref  参考原点GPS     坐标 lat_ref, lon_ref, h_ref
输出：
enu坐标 x, y, z

'''
def API_gnss_to_enu(gnss_in, gnss_ref):



    
         
    lat=gnss_in[0]
    lon=gnss_in[1]
    alt=gnss_in[2]
    x, y, z = gnss_to_ecef(lat, lon, alt)

    #x1, y2, z3 = gnss_to_ecef1(lat, lon, alt)
    lat0=gnss_ref[0]
    lon0=gnss_ref[1]
    alt0=gnss_ref[2]
    e,n,u=ecef_to_enu(x, y, z, lat0, lon0, alt0)
    
    
    #print(f"ENU coordinates: E={e}, N={n}, U={u}")
    return e,n,u


'''
# 原始gnss输入 
名字 纬度 经度 高度
DJI_0002.JPG 34.032505638888885 108.76779925 514.638
DJI_0005.JPG 34.03267641666667 108.76781155555555 514.464
DJI_0011.JPG 34.03394725 108.76789833333333 514.635

转化为 
纬度 经度 高度
34.032505638888885 108.76779925 514.638
34.03267641666667 108.76781155555555 514.464
34.03394725 108.76789833333333 514.635

'''
def API_data0123_to_data123(data0123):

    data123=[]
    for data_i in data0123:

        data_0=float(data_i[1])
        data_1=float(data_i[2])
        data_2=float(data_i[3])
        data_ii=[data_0,data_1,data_2]
        data123.append(data_ii)
    return data123


'''
# 将gnss列表集中转换过去enu
输入：
纬度 经度 高度 列表
34.032505638888885 108.76779925 514.638
34.03267641666667 108.76781155555555 514.464
34.03394725 108.76789833333333 514.635
'''
def API_gnss3_to_enu3_List(gnss0Lat1Lon2H_List):

    # 4 将gps转滑到enu坐标系
    # 4-1 第一帧为参考点
    lat0=float(gnss0Lat1Lon2H_List[0][0])
    lon0=float(gnss0Lat1Lon2H_List[0][1])
    alt0=float(gnss0Lat1Lon2H_List[0][2])
    gnss_ref=[lat0,lon0,alt0]
    if use_cgcs2000Towgs84:gnss_ref=Api_cgcs2000Towgs84(gnss_ref)
    print("参考GNSS位置",gnss_ref)

    ENU_List=[]
    for gps_i in gnss0Lat1Lon2H_List:

        lat=float(gps_i[0])
        lon=float(gps_i[1])
        alt=float(gps_i[2])
        gnss_in=[lat,lon,alt]
        if use_cgcs2000Towgs84:gnss_in=Api_cgcs2000Towgs84(gnss_in)
        
        
        # 4-2 转化坐标系
        e, n, u = API_gnss_to_enu(gnss_in,gnss_ref)
        # e=round(e, 3)
        # n=round(n, 3)
        # u=round(u, 3)
        ENU_List.append([e,n,u])
        #print("gnss-enu 单位m",name_,"输入经纬度",lat,lon,alt,"转化后的enu",e, n, u )
    return ENU_List


# 测试
# Gnss_list_Read = API_read2txt(GPS_txt_name)
# 将txt数据去掉第一列
# Gnss0Lat1Lon2H=API_data0123_to_data123(Gnss_list_Read)
# ENU_List=API_gnss_to_enu_List(Gnss0Lat1Lon2H)

# 4-2 将一个enu在给定gnss参考原点下转换到gnss
'''
功能： # enu直接转化到gnss坐标系
       相对于指定GPS_ref为原点（一般都是第一帧）的enu坐标系
输入：
from_enu   等待转换的GPS   坐标 lat, lon, h
gnss_ref  参考原点GPS     坐标 lat_ref, lon_ref, h_ref
输出：
gnss坐标 lat, lon, h

'''
def API_enu_to_gnss(from_enu,gnss_ref):
    e=from_enu[0]
    n=from_enu[1]
    u=from_enu[2]
    lat0=gnss_ref[0]
    lon0=gnss_ref[1]
    alt0=gnss_ref[2]
    # enu转换到ecef 在指定gnss_ref参考点下
    x, y, z = enu_to_ecef(e,n,u,lat0, lon0, alt0)

    # 从ecef转换到gnss
    gnss_=ecef_to_gnss(x,y,z)
    return gnss_


'''
# 将enu列表集中转换过去gnss
输入：
参数1 enu_list_Read
e n u 列表
0 0 0
1 0 0
1 1 0
参数2 gnss_ref
参考gnss点

输出
gps 位置
'''
def API_enu3_to_gnss3_list(enu_list_Read,gnss_ref):

    #gnss_ref=[lat0,lon0,alt0]

    print("参考GNSS位置",gnss_ref)

    GNSS_List=[]
    for enu_i in enu_list_Read:
        name_=enu_i[0]
        e=float(enu_i[1])
        n=float(enu_i[2])
        u=float(enu_i[3])
        from_enu_=[e,n,u]
        gnss_out=API_enu_to_gnss(from_enu_,gnss_ref)
        GNSS_List.append([gnss_out[0],gnss_out[1],gnss_out[2]])
       
    return GNSS_List






#5-1 多个txt数据 gnss转化到enu
# 第一帧为参考帧
def API_gnss4_to_enu4_List(Gnss_list_Read):  
    #GPS_txt_name="d1_100mRTKColmap.txt"
    # 3读取txt
    #Gnss_list_Read = API_read2txt(GPS_txt_name)

  

    # 4 将gps转滑到enu坐标系
    # 4-1 第一帧为参考点
    lat0=float(Gnss_list_Read[0][1])
    lon0=float(Gnss_list_Read[0][2])
    alt0=float(Gnss_list_Read[0][3])
    gnss_ref=[lat0,lon0,alt0]
    if use_cgcs2000Towgs84:gnss_ref=Api_cgcs2000Towgs84(gnss_ref)
    print("参考GNSS位置",gnss_ref)

    ENU_List=[]
    for gps_i in Gnss_list_Read:

        lat=float(gps_i[1])
        lon=float(gps_i[2])
        alt=float(gps_i[3])
        gnss_in=[lat,lon,alt]
        if use_cgcs2000Towgs84:gnss_in=Api_cgcs2000Towgs84(gnss_in)
        
        name_=gps_i[0]
        # 4-2 转化坐标系
        e, n, u = API_gnss_to_enu(gnss_in,gnss_ref)

       

        # e=round(e, 3)
        # n=round(n, 3)
        # u=round(u, 3)
        ENU_List.append([name_,e,n,u])
        #print("gnss-enu 单位m",name_,"输入经纬度",lat,lon,alt,"转化后的enu",e, n, u )


    return ENU_List


#5-2 多个txt数据 enu转化到gnss 
# 第一帧为参考帧
def API_enu4_to_gnss4_list(enu_list_Read,gnss_ref):
   
    #enu_list_Read = API_read2txt(ENU_txt_name)

    #gnss_ref=[lat0,lon0,alt0]

    print("参考GNSS位置",gnss_ref)

    GNSS_List=[]
    for enu_i in enu_list_Read:
        name_=enu_i[0]
        e=float(enu_i[1])
        n=float(enu_i[2])
        u=float(enu_i[3])

        from_enu_=[e,n,u]
        gnss_out=API_enu_to_gnss(from_enu_,gnss_ref)
        GNSS_List.append([name_,gnss_out[0],gnss_out[1],gnss_out[2]])
       
    return GNSS_List


#def waitUse():
    #import numpy as np
    #from scipy.spatial.transform import Rotation as R
    # 将四元数转换为旋转矩阵
    # rotation = R.from_quat([qx, qy, qz, qw])
    # rotation_matrix = rotation.as_matrix()

    # # 将旋转矩阵转换为欧拉角 (Omega, Phi, Kappa)
    # # 摄影测量中通常使用 ZYX 旋转顺序
    # omega, phi, kappa = rotation.as_euler('ZYX', degrees=True)
    # print("旋转矩阵:\n", rotation_matrix)

    # print("Omega:", omega, "Phi:", phi, "Kappa:", kappa)

#===========================================================
# if __name__ == "__main__":
#     # 参数
#     # 0-1 gps照片路径
#     img_path_dir="0测试数据/d1_100mRTKColmap/images/gps_images/"
#     # 0-2 txt保存的名字
    
#     # 1-1从照片读取gnss数据
#     Gnss_list=API_read_directory(img_path_dir)
#     # 1-2保存gps txt
#     GPS_txt_name="data/1GNSS_from_img.txt"
#     API_Save2txt(GPS_txt_name,Gnss_list)
    
#     # 3 gps转化到enu  第一帧参考位置
#     # 3-1 读取GNSS数据 -名字 lat lon h
#     enu_list_Read = API_read2txt(GPS_txt_name)
#     # 3-2 gnss数据转换为enu
#     ENU_List = API_gnss4_to_enu4_List(enu_list_Read)
#     # 3-2 保存enu结果  -名字 e n u
#     ENU_txt_name="data/2ENU_from_GNSS.txt"
#     API_Save2txt(ENU_txt_name,ENU_List)

#     # 4 读取enu数据 转化到 gnss
#     # 4-1 获取gnss参考点 - 名字 纬 经 高
#     Gnss_list_Read = API_read2txt(GPS_txt_name)
#     img_name=Gnss_list_Read[0][0]
#     lat0=float(Gnss_list_Read[0][1])
#     lon0=float(Gnss_list_Read[0][2])
#     alt0=float(Gnss_list_Read[0][3])
#     gnss_ref=[lat0,lon0,alt0]
#     if use_cgcs2000Towgs84:gnss_ref=Api_cgcs2000Towgs84(gnss_ref)
#     print("参考GNSS位置",gnss_ref)
#     # 4-2 获取enu数据集 -名字 e n u
#     enu_list_Read=API_read2txt(ENU_txt_name)
#     # 4-3 ENU数据转化为gnss数据
#     GNSS_list_from_enu=API_enu4_to_gnss4_list(enu_list_Read,gnss_ref)
#     # 4-2 保存gnss结果 名字 纬 经 高
#     GNSS_From_ENU_txt_name="data/3GNSS_From_ENU.txt"
#     API_Save2txt(GNSS_From_ENU_txt_name,GNSS_list_from_enu)
    
#     # 5 数据转化 为3D-3D计算相似变换准备
#     #ENU_List  :名字 e n u 转化为:  e n u
#     ENU_List_3=API_data0123_to_data123(ENU_List) # 去掉第一列名字
#     GNSS_list_from_enu_3=API_data0123_to_data123(GNSS_list_from_enu)



