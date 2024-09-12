# -*- coding: utf-8 -*-
# conda activate py37gaosi  # 服务器
# activate py38  # 笔记本

import os
import numpy as np


import csv

# 读取数据到字典
def Read_GNSS_TXT(file_path):
    data_dict = {}
    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter=' ')
        for row in reader:
            if row:  # 检查行是否为空
                timestamp = float(row[0])
                latitude = float(row[1])
                longitude = float(row[2])
                altitude = float(row[3])
                data_dict[timestamp] = (latitude, longitude, altitude)
    return data_dict

# # 根据时间戳查找数据
def find_data_by_timestamp(data_dict, target_timestamp):
    return data_dict.get(target_timestamp, None)

# GNSS_file_path ="FHY_config/FHY_gps.txt"  # 替换为实际文件路径
# GNSS_LIST = Read_GNSS_TXT(GNSS_file_path)

# # 查找特定时间戳
# target_timestamp = 1453132359.160000  # 替换为你要查找的时间戳
# result = find_data_by_timestamp(GNSS_LIST, target_timestamp)

# if result:
#     print(f"数据: 时间戳={target_timestamp}, 纬度={result[0]}, 经度={result[1]}, 高度={result[2]}")
# else:
#     print("没有找到对应的时间戳数据。")


'''
# 原始gnss输入 四个数据
名字 纬度 经度 高度
DJI_0002.JPG 34.032505638888885 108.76779925 514.638
DJI_0005.JPG 34.03267641666667 108.76781155555555 514.464
DJI_0011.JPG 34.03394725 108.76789833333333 514.635

转化为  三个数据
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

# 遍历文件夹读取 文件名字
def API_read_file_list(img_path_dir):

    file_dir_name_list=[]
   
    for filename in os.listdir(img_path_dir):
        file_dir_name=img_path_dir+filename

        
        file_dir_name_list.append(file_dir_name)
    

      
    return file_dir_name_list


def API_Save2txt(txt_name,Gnss_list):

    with open(txt_name, 'w') as file:
        for row in Gnss_list:
            line = ' '.join(map(str, row))
            file.write(f"{line}\n")

    print(txt_name,"保存成功")


def API_read2txt(txt_name):
    
    print(txt_name,"读取txt数据成功")
    Gnss_list = []
    with open(txt_name, 'r') as file:
        for line in file:
            row = list(map(str, line.split()))
            Gnss_list.append(row)
            #print(row)
    return Gnss_list


def Read_pose_from_colamp_sparse(txt_name, cam_ID=1):
    
    print(txt_name,"读取colamp_sparse txt数据成功")
    #cam_ID=1 # 需要提取的相机位姿
    pose_list = []
    with open(txt_name, 'r') as file:
        line_i=0
        for line in file:
            if line_i<=3:
                '''
                # 0 Image list with two lines of data per image:
                # 1  IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
                # 2  POINTS2D[] as (X, Y, POINT3D_ID)
                # 3 Number of images: 600, mean observations per image: 8259.4249999999993
                '''
                print("行",line_i,"信息说明",line)
                line_i=line_i+1
                
                continue

           
            if line_i%2==0: # 奇数  IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
                #row = list(map(float, line.split(' '))) #map(float 全部转化为 float

                line=line.replace('\n', '')# 去除空格

                row = list(line.split(' '))#按照空格切割
                IMAGE_ID=float(row[0])
                QW=float(row[1])
                QX=float(row[2])
                QY=float(row[3])
                QZ=float(row[4])
                TX=float(row[5])
                TY=float(row[6])
                TZ=float(row[7])
                CAMERA_ID=int(row[8])
                IMAGE_NAME=row[9]

                print("IMAGE_ID:",IMAGE_ID,"IMAGE_NAME:",IMAGE_NAME,"CAMERA_ID",CAMERA_ID,"Qwxyz:",QW,QX,QY,QZ,"Txyz:",TX,TY,TZ)
                if cam_ID==CAMERA_ID:
                    pose_i=[IMAGE_NAME,TX,TY,TZ,QW,QX,QY,QZ,CAMERA_ID]
                    pose_list.append(pose_i)
                    
            '''
            elif  line_i%2==1:#偶数   POINTS2D[] as (X, Y, POINT3D_ID)
 
                continue
                # 是否需要读取3D点 
                
                line=line.replace('\n', '')# 去除空格
                row = list(line.split(' '))#按照空格切割
                j=0
                i=0
                while j < len(row):
                    
                    x=row[j]
                    y=row[j+1]
                    point_3d_id=row[j+2]
                    
                    print("序号",i,"像素坐标",x,y,"3D点ID",point_3d_id)
                    j=j+3
                    i=i+1
            '''
            line_i=line_i+1
    return pose_list
   

   
# def API_txt_to_Draw3D(list_name_xyz):
   
    
#     x_list=[]
#     y_list=[]
#     z_list=[]
#     for data_i in list_name_xyz:
#         nam_i=data_i[0]
#         x_i=float(data_i[1])
#         y_i=float(data_i[2])
#         z_i=float(data_i[3])
#         x_list.append(x_i)
#         y_list.append(y_i)
#         z_list.append(z_i)
#     return x_list,y_list,z_list
   


#====================测试========================
'''
if __name__ == "__main__":
   

    # 参数
    # 0-1 gps照片路径
    img_path_dir="E:/v0_Project/V0_Mybao/v8_slam/python工具/0测试数据/d1_100mRTKColmap/images/gps_images/"
    # 0-2 txt保存的名字
    GPS_txt_name="GPS.txt"

    # 1读取数据
    Gnss_list=API_read_directory(img_path_dir)

    # 2保存txt
    API_Save2txt(GPS_txt_name,Gnss_list)

    # 3读取txt
    Gnss_list_Read = API_read2txt(GPS_txt_name)

'''