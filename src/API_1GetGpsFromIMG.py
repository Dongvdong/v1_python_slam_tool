# -*- coding: utf-8 -*-
# conda activate py37gaosi  # 服务器
# activate py38  # 笔记本

import os
import numpy as np

#改进检测地区
import os
import exifread
import re
import sys
import requests
import json


# 遍历文件夹及子文件夹中的所有图片,逐个文件读取exif信息
'''
def get_pic_GPS(pic_dir):
    items = os.listdir(pic_dir)
    for item in items:
        path = os.path.join(pic_dir, item)
        if os.path.isdir(path):
            get_pic_GPS(path)
        else:
            imageread(path)
'''


def convert_altitude_to_decimal(altitude):
    #print(altitude)
    try:
        # 可能会抛出异常的代码
        a, b = altitude.strip().split('/')
        #print(f"a:{a},b:{b}")
        return float(a)/float(b)
    except ValueError as e:
        #print(f"不存在‘/’")
        #print(altitude)
        # 这里可以继续执行其他代码
        return float(altitude)

        
# 将经纬度转换为小数形式
def convert_to_decimal(*gps):
    # 度
    if '/' in gps[0]:
        deg = gps[0].split('/')
        if deg[0] == '0' or deg[1] == '0':
            gps_d = 0
        else:
            gps_d = float(deg[0]) / float(deg[1])
    else:
        gps_d = float(gps[0])
    # 分
    if '/' in gps[1]:
        minu = gps[1].split('/')
        if minu[0] == '0' or minu[1] == '0':
            gps_m = 0
        else:
            gps_m = (float(minu[0]) / float(minu[1])) / 60
    else:
        gps_m = float(gps[1]) / 60
    # 秒
    if '/' in gps[2]:
        sec = gps[2].split('/')
        if sec[0] == '0' or sec[1] == '0':
            gps_s = 0
        else:
            gps_s = (float(sec[0]) / float(sec[1])) / 3600
    else:
        gps_s = float(gps[2]) / 3600

    decimal_gps = gps_d + gps_m + gps_s
    # 如果是南半球或是西半球
    if gps[3] == 'W' or gps[3] == 'S' or gps[3] == "83" or gps[3] == "87":
        return str(decimal_gps * -1)
    else:
        return str(decimal_gps)


'''
# 如果提取 图像信息不需要机身自讨 精度更高
Image ImageDescription
Image Make
Image Model
Image Orientation
Image XResolution
Image YResolution
Image ResolutionUnit
Image Software
Image DateTime
Image YCbCrPositioning
Image ExifOffset
GPS GPSVersionID
GPS GPSLatitudeRef
GPS GPSLatitude
GPS GPSLongitudeRef
GPS GPSLongitude
GPS GPSAltitudeRef
GPS GPSAltitude
Image GPSInfo
Image XPComment
Image XPKeywords
Thumbnail Compression
Thumbnail XResolution
Thumbnail YResolution
Thumbnail ResolutionUnit
Thumbnail JPEGInterchangeFormat
Thumbnail JPEGInterchangeFormatLength
EXIF ExposureTime
EXIF FNumber
EXIF ExposureProgram
EXIF ISOSpeedRatings
EXIF ExifVersion
EXIF DateTimeOriginal
EXIF DateTimeDigitized
EXIF ComponentsConfiguration
EXIF CompressedBitsPerPixel
EXIF ShutterSpeedValue
EXIF ApertureValue
EXIF ExposureBiasValue
EXIF MaxApertureValue
EXIF SubjectDistance
EXIF MeteringMode
EXIF LightSource
EXIF Flash
EXIF FocalLength
EXIF MakerNote
EXIF FlashPixVersion
EXIF ColorSpace
EXIF ExifImageWidth
EXIF ExifImageLength
Interoperability InteroperabilityIndex
Interoperability InteroperabilityVersion
EXIF InteroperabilityOffset
EXIF ExposureIndex
EXIF FileSource
EXIF SceneType
EXIF CustomRendered
EXIF ExposureMode
EXIF WhiteBalance
EXIF DigitalZoomRatio
EXIF FocalLengthIn35mmFilm
EXIF SceneCaptureType
EXIF GainControl
EXIF Contrast
EXIF Saturation
EXIF Sharpness
EXIF SubjectDistanceRange
EXIF BodySerialNumber


'''

# 读取图片的经纬度和拍摄时间
def Api_1_1Get_ImageGPS(path):

    f = open(path, 'rb')
    GPS = {}
    gps_=[-1,-1,-1]
    
    try:
        tags = exifread.process_file(f)
    except:
        return gps_
    #print(tags)
    
    #for tag in tags:               
       # print(str(tag),str(tags[str(tag)]))
    

    # 南北半球标识
    if 'GPS GPSLatitudeRef' in tags:

        GPS['GPSLatitudeRef'] = str(tags['GPS GPSLatitudeRef'])
        # print(GPS['GPSLatitudeRef'])
    else:
        GPS['GPSLatitudeRef'] = 'N'  # 缺省设置为北半球

    # 东西半球标识
    if 'GPS GPSLongitudeRef' in tags:
        GPS['GPSLongitudeRef'] = str(tags['GPS GPSLongitudeRef'])
        # print(GPS['GPSLongitudeRef'])
    else:
        GPS['GPSLongitudeRef'] = 'E'  # 缺省设置为东半球

    # 海拔高度标识
    if 'GPS GPSAltitudeRef' in tags:
        GPS['GPSAltitudeRef'] = str(tags['GPS GPSAltitudeRef'])

    # 获取纬度
    if 'GPS GPSLatitude' in tags:
        lat = str(tags['GPS GPSLatitude'])
        # 处理无效值
        if lat == '[0, 0, 0]' or lat == '[0/0, 0/0, 0/0]':
            GPS['GPSLatitude']=-1
            
        else:
            deg, minu, sec = [x.replace(' ', '') for x in lat[1:-1].split(',')]
            # 将纬度转换为小数形式
            GPS['GPSLatitude'] = convert_to_decimal(deg, minu, sec, GPS['GPSLatitudeRef'])

    # 获取经度
    if 'GPS GPSLongitude' in tags:
        lng = str(tags['GPS GPSLongitude'])
        # print(lng)

        # 处理无效值
        if lng == '[0, 0, 0]' or lng == '[0/0, 0/0, 0/0]':
            GPS['GPSLongitude']=-1
           
        else:
            deg, minu, sec = [x.replace(' ', '') for x in lng[1:-1].split(',')]
            # 将经度转换为小数形式
            GPS['GPSLongitude'] = convert_to_decimal(deg, minu, sec, GPS['GPSLongitudeRef'])  # 对特殊的经纬度格式进行处理

    # 获取海拔高度
    if 'GPS GPSAltitude' in tags:

        height = str(tags["GPS GPSAltitude"])
        GPS['GPSAltitude'] = convert_altitude_to_decimal(height)

    # if 'GPS RelativeAltitude' in tags:

    #     height = str(tags["GPS RelativeAltitude"])
    #     GPS['RelativeAltitude'] = convert_altitude_to_decimal(height)

        
    # 获取图片拍摄时间
    # if 'Image DateTime' in tags:
    #     GPS["DateTime"] = str(tags["Image DateTime"])
    #     print(GPS["DateTime"])
    # elif "EXIF DateTimeOriginal" in tags:
    #     GPS["DateTime"] = str(tags["EXIF DateTimeOriginal"])
    #     print(GPS["DateTime"])
    # if 'Image Make' in tags:
    #     print('照相机制造商：', tags['Image Make'])
    # if 'Image Model' in tags:
    #     print('照相机型号：', tags['Image Model'])
    # if 'Image ExifImageWidth' in tags:
    #     print('照片尺寸：', tags['EXIF ExifImageWidth'],tags['EXIF ExifImageLength'])



    gps_=[float(GPS['GPSLatitude']) ,float(GPS['GPSLongitude']), float(GPS['GPSAltitude'])]
    #print(gps_)
    return gps_

'''
# 如果需要提取相机字条和机身姿态
读取信息查看

   tiff:Make="DJI"
   tiff:Model="FC6310R"  
   dc:format="image/jpg"
   drone-dji:AbsoluteAltitude="+514.64"
   drone-dji:RelativeAltitude="+100.09"
   drone-dji:GpsLatitude="34.03250565"
   drone-dji:GpsLongtitude="108.76779926"
   drone-dji:GimbalRollDegree="+0.00"  拍照时刻云台的Roll 欧拉角
   drone-dji:GimbalYawDegree="+93.50"
   drone-dji:GimbalPitchDegree="-89.90"
   drone-dji:FlightRollDegree="+2.10"  拍照时刻飞行器机体的Roll 欧拉角
   drone-dji:FlightYawDegree="+93.50"
   drone-dji:FlightPitchDegree="+1.00"
   drone-dji:FlightXSpeed="+0.00"
   drone-dji:FlightYSpeed="+0.00"
   drone-dji:FlightZSpeed="+0.00"
   drone-dji:CamReverse="0"
   drone-dji:GimbalReverse="0"
   drone-dji:SelfData="Undefined"
   drone-dji:CalibratedFocalLength="3666.666504"   焦距
   drone-dji:CalibratedOpticalCenterX="2736.000000"
   drone-dji:CalibratedOpticalCenterY="1824.000000"
   drone-dji:RtkFlag="50"
   drone-dji:RtkStdLon="0.01117"
   drone-dji:RtkStdLat="0.01132"
   drone-dji:RtkStdHgt="0.02493"
   drone-dji:DewarpFlag="1"
   drone-dji:PhotoDiff=""
   crs:Version="7.0"
   crs:HasSettings="False"
   crs:HasCrop="False"
   crs:AlreadyApplied="False">
  </rdf:Description>

'''
 

 #  读取单个照片的 gps信息 这个方法有问题  读取高度精度损失
def Api_1_2Get_Image_AllInfo(file_path):
    b = b"\x3c\x2f\x72\x64\x66\x3a\x44\x65\x73\x63\x72\x69\x70\x74\x69\x6f\x6e\x3e"
    a = b"\x3c\x72\x64\x66\x3a\x44\x65\x73\x63\x72\x69\x70\x74\x69\x6f\x6e\x20"

    aa=["\x3c\x72\x64\x66\x3a\x44\x65\x73\x63\x72\x69\x70\x74\x69\x6f\x6e\x20"]
    bb=["\x3c\x2f\x72\x64\x66\x3a\x44\x65\x73\x63\x72\x69\x70\x74\x69\x6f\x6e\x3e"]

    #xml format to save EXIF的数据规范
    # aa ['<rdf:Description ']
    #print("aa",aa)
    # bb ['</rdf:Description>']
    #print("bb",bb)



    # rb是读取二进制文件
    img = open(file_path, 'rb')
    # bytearray() 方法返回一个新字节数组
    data = bytearray()
    #标识符,
    flag = False

    for i in img.readlines():
       
        # 按行读取二进制信息，标签成对出现
        if a in i:
            flag = True
        if flag:
            #把第i行数据复制到新数组中
            data += i
        if b in i:
            break
    #print("======大疆精灵4照片中原始数据 =======\n",data)

    if len(data) > 0:
        data = str(data.decode('ascii'))#ascii 
        #print(data)
        #filter()函数用于过滤序列，过滤掉不符合条件的元素，返回符合条件的元素组成新列表。
        #filter(function,iterable) ,function -- 判断函数。iterable -- 可迭代对象
        #python允许用lambda关键字创造匿名函数。
        # 在 lambda 关键字之后、冒号左边为参数列表，可不带参数，也可有多个参数。若有多个参数，则参数间用逗号隔开，冒号右边为 lambda 表达式的返回值。
        #left--->right
        # judge condition 'drone-dji:' in x
        lines = list(filter(lambda x: 'drone-dji:' in x, data.split("\n")))
        #print("lines",lines)
        dj_data_dict = {}
       
        for d in lines:
            # remove 'drone-dji:'
            d = d.strip()[10:]
            # k is name
            # v is value
            k, v = d.split("=")
            v=v.replace('\"','')
            
            #print(k, v)
            dj_data_dict[k] = v
        return dj_data_dict
    else:
        dj_data_dict="error"


def Api_2Get_Image_GPS(img_path):


    #img_name=img_path[img_path.rfind('/')+1:img_path.rfind('.')]# 去掉后缀
    #print(img_name)
    dj_data_dict=Api_1_1Get_ImageGPS(img_path)

    GPS_lat_lon_h=[-1,-1,-1]
    if dj_data_dict !="error":
        

        #dj_data_dict['GpsLatitude'] # 这种方法会损失高度
        #dj_data_dict['GpsLongtitude']
        #dj_data_dict['RelativeAltitude']
      
        lat=float(dj_data_dict[0])
        lon=float(dj_data_dict[1])  
        absh=float(dj_data_dict[2])  

        GPS_lat_lon_h=[lat,lon,absh]
    
    #print(GPS)
    return GPS_lat_lon_h





# 遍历文件夹读取照片的GPS 高度保留厘米
def API_read_directory(img_path_dir):

           
   
    Gnss_list=[]
   
    for filename in os.listdir(img_path_dir):
        file_dir_name=img_path_dir+filename
        #print(filename)
        GPS_lat_lon_h=Api_2Get_Image_GPS(file_dir_name)
        gnss_temp=[]
        
        gnss_temp.append(filename)
        gnss_temp.append(GPS_lat_lon_h[0])
        gnss_temp.append(GPS_lat_lon_h[1])
        gnss_temp.append(GPS_lat_lon_h[2])
       

        #print("1 照片读取到的GNSS",gnss_temp)
        Gnss_list.append(gnss_temp)
    return Gnss_list


def API_Save2txt(GPS_txt_name,Gnss_list):

    with open(GPS_txt_name, 'w') as file:
        for row in Gnss_list:
            line = ' '.join(map(str, row))
            file.write(f"{line}\n")

    print(GPS_txt_name,"保存成功")


def API_read2txt(GPS_txt_name):
    
    print(GPS_txt_name,"读取txt数据成功")
    Gnss_list = []
    with open(GPS_txt_name, 'r') as file:
        for line in file:
            row = list(map(str, line.split()))
            Gnss_list.append(row)
            #print(row)
    return Gnss_list


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

    # 2保存txt 格式: name lat lon h
    API_Save2txt(GPS_txt_name,Gnss_list)

    # 3读取txt 格式: name lat lon h
    Gnss_list_Read = API_read2txt(GPS_txt_name)

'''