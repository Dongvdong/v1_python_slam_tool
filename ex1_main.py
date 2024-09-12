
from API_2Gps22ENU import *
from API_33DTo3D import *

    


##===========================================================
if __name__ == "__main__":

    '''
    # 参数
    # 0-1 gps照片路径
    #img_path_dir="0测试数据/d1_100mRTKColmap/images/gps_images/"
    # 0-2 txt保存的名字
    
    # 1-1从照片读取gnss数据
    #Gnss_list=API_read_directory(img_path_dir)
    # 1-2保存gps txt
    GPS_txt_name="data/test/FHY_gps.txt"
    #API_Save2txt(GPS_txt_name,Gnss_list)
    
    # 3 gps转化到enu  第一帧参考位置
    # 3-1 读取GNSS数据 -名字 lat lon h
    Gnss_list_Read = API_read2txt(GPS_txt_name)
    # 3-2 gnss数据转换为enu
    ENU_List=API_gnss4_to_enu4_List(Gnss_list_Read)
    # 3-2 保存enu结果  -名字 e n u
    ENU_txt_name="data/test/2ENU_from_GNSS.txt"
    API_Save2txt(ENU_txt_name,ENU_List)


    
    # # 4 数据转化 为3D-3D计算相似变换准备  colmap enu 变换到 gnss enu坐标系上
    # #ENU_List  :名字 e n u 转化为:  e n u
    # 4-1 读取gnss enu
    # 取出前400个数据计算
    ENU_GNSS_List_4= API_read2txt("data/test/2ENU_from_GNSS.txt")
    ENU_GNSS_List_4_400=[]
    for i in range(160 , len(ENU_GNSS_List_4)):
        ENU_GNSS_List_4_400.append(ENU_GNSS_List_4[i]) 
    ENU_GNSS_List_3_400=API_data0123_to_data123(ENU_GNSS_List_4_400) # 去掉第一列名字
    
    # 4-2 读取colmap enu
    ENU_colmap_list_4= API_read2txt("data/test/colmap_images_t.txt")
    ENU_colmap_list_3=API_data0123_to_data123(ENU_colmap_list_4) # 去掉第一列名字

    # 4-4 计算变换关系 points_src 到 points_dst
    points_src=ENU_colmap_list_3
    points_dst=ENU_GNSS_List_3_400 #ENU_GNSS_List_3_400
    RT_34, SR, T = API_pose_estimation_3dTo3d_ransac(points_src, points_dst) # 
   
    colmapenu_in_gnssenu_3=API_src3D_sRt_dis3D_list(points_src,points_dst,SR, T)
    
    # 4-5 保存计算结果 colmap enu变换到gnss enu坐标系下的新坐标
    colmapenu_in_gnssenu_4=[]
    for i in range(0,len(ENU_GNSS_List_4_400)):
        name=ENU_GNSS_List_4[i][0]
        #保存数据 名字 e n u
        li=[name,colmapenu_in_gnssenu_3[i][0],colmapenu_in_gnssenu_3[i][1],colmapenu_in_gnssenu_3[i][2]]
        colmapenu_in_gnssenu_4.append(li)
    # 保存数据
    colmapeEnu_from_GnssEnu_txt_name="data/test/3colmapeEnu_from_GnssEnu.txt"
    API_Save2txt(colmapeEnu_from_GnssEnu_txt_name,colmapenu_in_gnssenu_4)

    
    # 5 将colmap enu变换到gnss enu坐标系下的新坐标，转换到GNSS坐标下
    # 读取enu数据 转化到 gnss
    # 5-1 获取gnss参考点 - 名字 纬 经 高
    Gnss_list_Read = API_read2txt(GPS_txt_name)
    img_name=Gnss_list_Read[0][0]
    lat0=float(Gnss_list_Read[0][1])
    lon0=float(Gnss_list_Read[0][2])
    alt0=float(Gnss_list_Read[0][3])
    gnss_ref=[lat0,lon0,alt0]
    use_cgcs2000Towgs84=1
    if use_cgcs2000Towgs84:gnss_ref=Api_cgcs2000Towgs84(gnss_ref)
    print("参考GNSS位置",gnss_ref)
    # 5-2 获取enu数据集 -名字 e n u
    enu_list_Read=API_read2txt(colmapeEnu_from_GnssEnu_txt_name)
    # 5-3 ENU数据转化为gnss数据
    GNSS_list_from_enu=API_enu4_to_gnss4_list(enu_list_Read,gnss_ref)
    # 5-4 保存gnss结果 名字 纬 经 高
    GNSS_From_ENU_txt_name="data/test/3colmapenu_to_gnss.txt"
    API_Save2txt(GNSS_From_ENU_txt_name,GNSS_list_from_enu)
    # 5-5 调用可视化软件可视化轨迹 
    '''


    path_txt="0测试数据1/d1_100mRTKColmap/sparse/0/"

    cameras_path=path_txt+"cameras.txt"
    points3D_path=path_txt+"points3D.txt"
    images_path=path_txt+"images.txt"
    
    from API_0File_Read_Write import *
    cam_id=1# 相机的编号 视频帧还是照片帧
    pose_list=Read_pose_from_colamp_sparse(images_path,cam_id)
    #print(pose_list)
    API_Save2txt("colmap_nogps_pose.txt",pose_list)