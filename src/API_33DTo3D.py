
import random
import math

import numpy as np
import os


import typing
UmeyamaResult = typing.Tuple[np.ndarray, np.ndarray, float]

# evo 自带的评估 https://blog.csdn.net/hhaowang/article/details/105225595
def umeyama_alignment(x: np.ndarray, y: np.ndarray,
                      with_scale: bool = 1) -> UmeyamaResult:


    # 注意 x和 y要转置
    if x.shape != y.shape:
        raise ("data matrices must have the same shape")

    # m = dimension, n = nr. of data points
    m, n = x.shape

    # means, eq. 34 and 35
    mean_x = x.mean(axis=1)
    mean_y = y.mean(axis=1)

    # variance, eq. 36
    # "transpose" for column subtraction
    sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis])**2)

    # covariance matrix, eq. 38
    outer_sum = np.zeros((m, m))
    for i in range(n):
        outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
    cov_xy = np.multiply(1.0 / n, outer_sum)

    # SVD (text betw. eq. 38 and 39)
    u, d, v = np.linalg.svd(cov_xy)
    if np.count_nonzero(d > np.finfo(d.dtype).eps) < m - 1:
        raise ("Degenerate covariance rank, "
                                "Umeyama alignment is not possible")

    # S matrix, eq. 43
    s = np.eye(m)
    if np.linalg.det(u) * np.linalg.det(v) < 0.0:
        # Ensure a RHS coordinate system (Kabsch algorithm).
        s[m - 1, m - 1] = -1

    # rotation, eq. 40
    R = u.dot(s).dot(v)

    # scale & translation, eq. 42 and 41
    s = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
    t = mean_y - np.multiply(s, R.dot(mean_x))


    sR = s*R


    #self.scale(s)
    # se3 = np.eye(4)
    # se3[:3, :3] = R
    # se3[:3, 3] = t
   
    return s,R,sR,t




def API_pose_estimation_3dTo3d_ransac(points_src, points_dst): #NED -> slam
    p = np.array(points_src, dtype=float)
    q = np.array(points_dst, dtype=float)
    print("len(points_src): ", len(points_src), " ", len(points_dst))
    # 1.计算s并去质心
    mean_p = np.mean(p, axis=0)
    mean_q = np.mean(q, axis=0)

    p_norm = p - mean_p
    q_norm = q - mean_q

    # 计算距离比
    iter_num = 0
    _s = 0
    inliner_num = 0
    while iter_num < 2000:
        # 随机挑选两个元素
        _list = []
        # print("len(points_src): ",len(points_src))
        # if len(points_src) < 2:
        #     break
        inx_1, inx_2 = random.sample(range(len(points_src)), 2)
        # print("inx_1: ",inx_1)
        # print("inx_2: ",inx_2)
        p_r = np.array([points_src[inx_1], points_src[inx_2]], dtype=float)
        q_r = np.array([points_dst[inx_1], points_dst[inx_2]], dtype=float)
        _list.append(inx_1)
        _list.append(inx_2)
        # 计算s

        p_norm_r = p_r - mean_p
        q_norm_r = q_r - mean_q

        # 所有点的xyz平方求和
        d1_list = []
        d2_list = []
        for i in range(len(q_norm_r)):
            d1 = q_norm_r[i]
            d2 = p_norm_r[i]
            dist1 = math.sqrt(np.sum(d1**2))
            dist2 = math.sqrt(np.sum(d2**2))
            d1_list.append(dist1)
            d2_list.append(dist2)
        s_r = np.sum(d1_list) / np.sum(d2_list)

        # 计算其他点s的误差值
        inliner_p = [points_src[inx_1], points_src[inx_2]]
        inliner_q = [points_dst[inx_1], points_dst[inx_2]]

        for inx in range(len(points_src)):
            # 计算点不参与验证
            if inx == inx_1 or inx == inx_2:
                continue
            p_src = np.array(points_src[inx])
            q_dst = np.array(points_dst[inx])
            # 分别计算到质心距离
            p_src_norm = p_src - mean_p
            q_dst_norm = q_dst - mean_q
            p_src_norm_dist = math.sqrt(np.sum(p_src_norm**2))
            q_dst_norm_dist = math.sqrt(np.sum(q_dst_norm**2))
            # 计算误差
            cal_dist = p_src_norm_dist * s_r
            error = cal_dist - q_dst_norm_dist
            if abs(error) < 3:
                inliner_p.append(points_src[inx])
                inliner_q.append(points_dst[inx])
                _list.append(inx)

        # 利用所有内点计算新的s
        p_r = np.array(inliner_p)
        q_r = np.array(inliner_q)
        p_norm_f = p_r - mean_p
        q_norm_f = q_r - mean_q

        d1_list = []
        d2_list = []
        for i in range(len(q_norm_f)):
            d1 = q_norm_f[i]
            d2 = p_norm_f[i]
            dist1 = math.sqrt(np.sum(d1**2))
            dist2 = math.sqrt(np.sum(d2**2))
            d1_list.append(dist1)
            d2_list.append(dist2)

        s_final = np.sum(d1_list) / np.sum(d2_list)
        # 记录内点数最高的模型
        if inliner_num < len(inliner_p):
            _s = s_final
            inliner_num = len(inliner_p)
            inx_list = _list
        iter_num += 1

    s = _s

    # 2.用s缩放src到dst尺度下
    p = s * p
    mean_p = np.mean(p, axis=0)
    p_norm = p - mean_p

    # 2.计算q1*q2^T(注意顺序：q2->q1，x是dst,y是src)
    N = len(p)

    W = np.zeros((3, 3))
    for i in range(N):
        x = q_norm[i, :]     # 每一行数据
        x = x.reshape(3, 1)  # 3行1列格式 一维数组借助reshape转置
        y = p_norm[i, :]
        y = y.reshape(1, 3)
        W += np.matmul(x, y)

    # 3.SVD分解W
    # python 线性代数库中svd求出的V与C++ Eigen库中求的V是转置关系
    U, sigma, VT = np.linalg.svd(W, full_matrices=True)
    # 旋转矩阵R
    R = np.matmul(U, VT)    # 这里无需再对V转置
    # 在寻找旋转矩阵时，有一种特殊情况需要注意。有时SVD会返回一个“反射”矩阵，这在数值上是正确的，但在现实生活中实际上是无意义的。
    # 通过检查R的行列式（来自上面的SVD）并查看它是否为负（-1）来解决。如果是，则V的第三列乘以-1。
    # 验证R行列式是否为负数   参考链接:https://blog.csdn.net/sinat_29886521/article/details/77506426
    if np.linalg.det(R) < 0:
        det = np.linalg.det(np.matmul(U, VT))
        # det 值为 -1
        mat = np.array([[1, 0, 0], [0, 1, 0], [0, 0, det]])
        ne_VT = np.matmul(VT, mat)
        R = np.matmul(U, ne_VT)
    # 平移向量
    T = mean_q - np.matmul(R, mean_p)   # dst - src
    T = T.reshape(3, 1)
    sR = s*R
    RT_34 = np.c_[sR, T]

    # 4.计算误差值
    p = np.array(points_src)
    error_sum = 0
    inx_list2 = []
    error_ENU = []
    for i in range(N):
        src = p[i, :]
        dst = q[i, :]
        src = src.reshape(3, 1)
        dst = dst.reshape(3, 1)
        test_dst = np.matmul(sR, src) + T

        error_Mat = test_dst - dst
        error_Mat2 = error_Mat**2
        error = np.sum(error_Mat2)
        error_ENU.append(math.sqrt(error))
        if error < 3:
            inx_list2.append(i)
        error_sum += error

    print("mean error:", math.sqrt(error_sum/N))
    print("max error:", max(error_ENU))
    print("RT_34:", RT_34)
    print("R:", R)
    return s, R, sR, T


# 根据 srt 将单个目标点云变换到指定坐标系下
def API_src3D_sRt_dis3D_one(points_src,SR,T):
           
    points_src_ = [[points_src[0]], [points_src[1]], [points_src[2]]]
    points_dis_ = np.matmul(SR,points_src_) + T
    #points_dis_ = SR @ points_src_ + T
    points_dis_t=[points_dis_[0][0],points_dis_[1][0],points_dis_[2][0]]

    return points_dis_t

# 将1组3d点 根据 srt变换到另一个坐标系下
def API_src3D_sRt_dis3D_list(points_src,points_dst,SR,T):


    points_dis_t_list=[]
    error_sum=0 # 误差计算

    for p_i in range(0,len(points_src)):
        # 根据srt计算便函后的x y z平移点
        points_dis_t=API_src3D_sRt_dis3D_one(points_src[p_i],SR,T)   
        #print("原始点",points_src[p_i],"变换后的点",points_dis_t,"真值",points_dst[p_i])
        points_dis_t_list.append(points_dis_t)
        
        # =========整体转换后的计算误差===============
        points_dis_t = np.array(points_dis_t)
        points_dst[p_i] = np.array(points_dst[p_i])
        error_Mat = points_dis_t - points_dst[p_i]
        error_Mat2 = error_Mat**2
        error = np.sum(error_Mat2)
        error_sum += error
      
    error_sum=math.sqrt(error_sum/len(points_src))
    print("平均误差:",error_sum)
    return points_dis_t_list
    

'''
if __name__ == "__main__":
    
    
    # points_src=[[1,1,1],[2,2,2],[3,3,3]]
    # points_dst=[[11,11,11],[21,21,21],[31,31,31]]
    # RT_34, SR, T = API_pose_estimation_3dTo3d_ransac(points_src, points_dst) # 
    # points_dis_t_list=API_src3D_sRt_dis3D_list(points_src,points_dst,SR, T)
    # print("变换后的列表",points_dis_t_list)

'''

'''
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
'''