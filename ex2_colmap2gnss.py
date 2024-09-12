import numpy as np
import src.API_2Gps22ENU as GNSS2ENU
import src.API_33DTo3D as RansacICP
import src.util as util
import cv2
import argparse

# pip install pyyaml
import yaml


def quaternion_conjugate(qvec):
    """计算四元数的共轭"""
    return np.array([qvec[0], -qvec[1], -qvec[2], -qvec[3]])


def quaternion_rotate_vector(qvec, vec):
    """使用四元数旋转一个向量"""
    qvec_w, qvec_x, qvec_y, qvec_z = qvec
    # 将向量表示为四元数 [0, x, y, z]
    vec_quat = np.array([0, vec[0], vec[1], vec[2]])
    # 四元数乘法
    q_conj = quaternion_conjugate(qvec)

    # 四元数相乘的公式 q * v * q^-1
    vec_rotated = quaternion_multiply(
        quaternion_multiply(qvec, vec_quat), q_conj
    )

    # 返回旋转后的向量
    return vec_rotated[1:]


def quaternion_multiply(q1, q2):
    """四元数相乘"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
        w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    ])


def camera_to_world(qvec, tvec):
    """将 qvec 和 tvec 从相机坐标系转换到世界坐标系"""
    # 1. 计算四元数的共轭
    qvec_conj = quaternion_conjugate(qvec)

    # 2. 旋转 tvec 并取反
    tvec_world = -quaternion_rotate_vector(qvec_conj, tvec)

    return qvec_conj, tvec_world

def find_timestamp(filename):

    dot_index = filename.rfind('.')
    image_name = filename[0:dot_index]
    
    return image_name

def read_colmapPose_imageText(path):
    """
    参考自 https://github.com/colmap/colmap/blob/dev/scripts/python/read_write_model.py
    """
    dict_video_colmap_xyz = {}
    with open(path, "r") as fid:
        while True:
            line = fid.readline()
            if not line:
                break
            line = line.strip()
            if len(line) > 0 and line[0] != "#" \
               and (line.find("jpg")!=-1 or line.find("png")!=-1 or line.find("JPG")!=-1):
                elems = line.split()
                image_id = int(elems[0])
                qvec = np.array(tuple(map(float, elems[1:5])))
                tvec = np.array(tuple(map(float, elems[5:8])))
                camera_id = int(elems[8])
                image_name = elems[9]

                # colmap默认的是从世界到相机，此处转换 qvec 和 tvec 从相机到世界
                qvec_wc, tvec_wc = camera_to_world(qvec, tvec)

                time_stamp = find_timestamp(image_name)

                # 保存相机在colmap世界坐标系下的位置到文件txt，格式为time_stamp x y z
                dict_video_colmap_xyz[time_stamp] = [tvec_wc[0],tvec_wc[1],tvec_wc[2]]

    return dict_video_colmap_xyz


def Read_gnss_int(gps_config_yaml):
    init_lat=0
    init_lon=0
    init_h=0

    with open(gps_config_yaml, 'r') as file:
        data = yaml.safe_load(file)
        if isinstance(data, dict):
            # 提取数据
            init_lat = data.get('Initial.lat', None)
            init_lon = data.get('Initial.lon', None)
            init_h = data.get('Initial.alt', None)
        else:
            print("读取GNSS原始点失败")
    
    print("初始参考点",init_lat,' ',init_lon,' ',init_h)
    init_gnss=[init_lat,init_lon,init_h]
    return init_gnss


def read_gnssPose_Text(txt_name,init_gnss):
    '''
    示例字典
    GNSS_LIST = {
        '2024-01-01T12:00:00Z': [34.052235, -118.243683, 100],
        '2024-01-01T12:05:00Z': [36.169941, -115.139832, 200],
        '2024-01-01T12:10:00Z': [37.774929, -122.419418, 300]
    }
    '''
    GNSS_ENU_LIST={}
    GNSS_LIST={}
    with open(txt_name, "r") as f1: # 读取所有镇的gps信息，
        
        for line in f1:
            line = line.strip()
            line = line.replace("  ", " ")
            elems = line.split(" ")  # 0 图像名称（不带后缀）1-3 lat lon alt
            time_stamp = str(elems[0])
            #init_gnss=[init_lat,init_lon,init_h]

            lat=float(elems[1])
            lon=float(elems[2])
            alt=float(elems[3])
            gnss_in=[lat,lon,alt]
            GNSS_LIST[time_stamp]=gnss_in
            #print("===",gnss_in)

            # 参考点的经纬度和高度（参考点的坐标）
            #init_gnss=GNSS2ENU.Api_cgcs2000Towgs84(init_gnss)
            #gnss_in=GNSS2ENU.Api_cgcs2000Towgs84(gnss_in)

            ref_lat=init_gnss[0]
            ref_lon=init_gnss[1]
            ref_height=init_gnss[2]

            #e, n, u = GNSS2ENU.API_gnss_to_enu(gnss_in,init_gnss)
            

            e, n, u = util.GPS2NED(ref_lat, ref_lon, ref_height,
                                    float(elems[1]), float(elems[2]), float(elems[3]))
            
            #print("误差",e1-e,n1-n,u1-u)

            '''
            from geographiclib.geodesic import Geodesic
            import math

            # 创建 Geodesic 对象
            geod = Geodesic.WGS84

            # 要转换的点的经纬度和高度
            point_lat = lat
            point_lon = lon
            point_height = alt

            # 计算参考点和目标点之间的 ENU 坐标
            result = geod.Inverse(ref_lat, ref_lon, point_lat, point_lon)

            # 计算水平距离和方位角
            distance = result['s12']
            azimuth = result['azi1']

            # 计算 ENU 坐标
            # 方位角是从北方向东的角度，需要转换为从东向北的角度（ENU坐标系）
            azimuth_enu = (azimuth + 90) % 360

            # 使用距离和方位角计算 ENU 坐标
            e = distance * math.sin(math.radians(azimuth_enu))
            n = distance * math.cos(math.radians(azimuth_enu))
            u = point_height - ref_height  # 上向坐标（高度差）
            '''

            # from pyproj import Proj, Transformer
            # # 创建 WGS84 坐标系和 ENU 坐标系转换器
            # transformer = Transformer.from_crs("epsg:4326", "epsg:4979", always_xy=True)
            
           

            # # # 将参考点和目标点的经纬度高度转换为 ECEF 坐标
            # ref_x, ref_y, ref_z = transformer.transform(ref_lon, ref_lat, ref_height)
            # point_x, point_y, point_z = transformer.transform(lon, lat, alt)

            # # 计算 ENU 坐标
            # e = point_x - ref_x
            # n = point_y - ref_y
            # u = point_z - ref_z


            #print("===", [ned_x,ned_y,ned_z])
            GNSS_ENU_LIST[time_stamp] = [e,n,u]
            

    return GNSS_ENU_LIST,GNSS_LIST



if __name__ == "__main__":
    # 0 -1 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description='处理文件路径参数')

    # 添加参数
    parser.add_argument('--colmap_res_txt', type=str, help='images.txt文件的路径')
    parser.add_argument('--gps_ned_ori', type=str, help='gps NED坐标系的原点xml文件的路径')
    parser.add_argument('--video_gps_txt', type=str, help='视频帧的gps信息文件的路径')
    parser.add_argument('--colmap2gnss_SRt_xml', type=str, help='输出的xml文件路径')

    # 解析参数
    args = parser.parse_args()

    # 使用参数
    colmap_res_txt = args.colmap_res_txt
    gps_ned_ori = args.gps_ned_ori
    video_gps_txt = args.video_gps_txt
    colmap2gnss_SRt_xml = args.colmap2gnss_SRt_xml

    # ========= 1 - 1 解析 colamp 位姿结果
    dict_video_colmap_xyz = read_colmapPose_imageText(colmap_res_txt)

    # ========= 1-2-1 读入gnss ned坐标系原点
    # 读取 YAML 文件
    init_gnss=Read_gnss_int(gps_ned_ori)

    # ========= 1-2-2 解析 gnss 位姿结果
    GNSS_ENU_LIST,GNSS_LIST = read_gnssPose_Text(video_gps_txt,init_gnss)

    # ========= 2-1 按时间戳(str)匹配GNSS和colmap数据
    points_src_colmap=[]
    points_dst_gnss=[]

    for gnss_time_stamp in GNSS_ENU_LIST.keys():
        #print("gnss-enu",GNSS_ENU_LIST[gnss_time_stamp],"",dict_video_colmap_xyz[gnss_time_stamp])
        if dict_video_colmap_xyz.get(gnss_time_stamp):
            points_src_colmap.append(dict_video_colmap_xyz[gnss_time_stamp])
            points_dst_gnss.append(GNSS_ENU_LIST[gnss_time_stamp])
           
    gps_ned_np = np.array(points_src_colmap)
    colmap_ned_np = np.array(points_dst_gnss)

    # ========= 2-2 计算变换关系
    #s, R, sR,t  = RansacICP.API_pose_estimation_3dTo3d_ransac(points_src_colmap, points_dst_gnss) # 
    #
    s, R, sR,t  = RansacICP.umeyama_alignment(gps_ned_np.T, colmap_ned_np.T) # 
   


    #========= 3-1 将 R T s分别写入xml中
    fs = cv2.FileStorage(colmap2gnss_SRt_xml, cv2.FILE_STORAGE_WRITE)
    fs.writeComment('这里的sRT是从colmap到GPS-NED坐标系下的', 0)
    fs.write('R', R)
    fs.write('t', t)
    fs.write('s', s)
    fs.release()


    # ======== 3-2 从 YAML 文件中读取数据
    fs = cv2.FileStorage(colmap2gnss_SRt_xml, cv2.FILE_STORAGE_READ)
    # 读取旋转矩阵 R
    R = fs.getNode('R').mat()
    # 读取平移向量 t
    t = fs.getNode('t').mat()
    # 读取尺度 s
    s = fs.getNode('s').real()
    # 释放文件存储
    fs.release()

    # 打印读取的数据以验证
    print('旋转矩阵 R:')
    print(R)
    print('位移向量 t:')
    print(t)
    print('尺度因子 s:')
    print(s)
    # 生成尺度矩阵
    S = np.diag([s, s, s])

    # 组合旋转矩阵、尺度矩阵和位移向量成变换矩阵
    # 变换矩阵的形式为 [R*s | t]
    # 其中 R*s 为旋转缩放矩阵，t 为平移向量
    T = np.hstack((S @ R, t.reshape(-1, 1)))

    # 将 T 变换矩阵扩展为 4x4 矩阵
    T_homogeneous = np.vstack((T, [0, 0, 0, 1]))

    # 打印变换矩阵
    print('变换矩阵 T:')
    print(T_homogeneous)


    #=========== 4 点云变换过去
    colmapenu_in_gnssenu_3 = RansacICP.API_src3D_sRt_dis3D_list(points_src_colmap, points_dst_gnss, sR, t)
    