import numpy as np
import subprocess

def read_colmap_images_file(images_file_path):
    with open(images_file_path, 'r') as file:
        lines = file.readlines()
    
    camera_poses = []
    for line in lines:
        if line.startswith('#'):
            
            continue
        elements = line.split()
        #IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        if len(elements) == 10:
            image_id = int(elements[0])
            qw, qx, qy, qz = map(float, elements[1:5])
            tx, ty, tz = map(float, elements[5:8])
            camera_id= elements[8]
            image_name= elements[9]
            camera_poses.append((image_id, qw, qx, qy, qz, tx, ty, tz))
         
    
    return camera_poses

# 四元数转化为R矩阵
def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    # Compute rotation matrix from quaternion
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])
    return R

# R矩阵和t矩阵转换为T矩阵
def camera_pose_to_T_4x4(qw, qx, qy, qz, tx, ty, tz):
    R = quaternion_to_rotation_matrix(qw, qx, qy, qz)
    t = np.array([tx, ty, tz]).reshape((3, 1))
    
    R_c2w = np.linalg.inv(R)
    t_c2w = -R_c2w*t
    T_3x4 = np.hstack((R_c2w, t_c2w))
    T_4x4 = np.vstack((T_3x4, [0, 0, 0, 1]))
    return T_4x4


'''
保存结果
1 0.9999959937483184 0.0017871597024803404 -0.002195119019744949 -3.627443722866289 -0.001794385269706037 0.9999929645029 -0.003294104765415675 2.6009038857471354 0.0021892164846989523 0.0032980304576372007 0.9999921651324492 -0.04164464363873901 0.0 0.0 0.0 1.0
2 0.9998690255343146 0.015487330590349544 -0.004698336753091023 -3.0289516912179733 -0.015498656201883329 0.9998770470171884 -0.002383800353051685 2.6328973337052757 0.004660840374443873 0.0024563060421309737 0.9999861214675089 -0.04999252057720493 0.0 0.0 0.0 1.0
3 0.9999581152158454 -0.0035085152289396505 -0.008453291362668305 1.0832811811035812 0.0034921662045554625 0.9999920049504796 -0.0019480272893019784 2.48650826266406 0.008460058461596155 0.0019184253981854682 0.9999623728195066 0.014663663633349009 0.0 0.0 0.0 1.0


'''
def images_pose_T4x4(camera_poses,txt_name):
    T_4x4_list=[]
    for i, pose in enumerate(camera_poses):
        image_id, qw, qx, qy, qz, tx, ty, tz = pose
        T_Rt4x4 = camera_pose_to_T_4x4(qw, qx, qy, qz, tx, ty, tz)
        
        T_4x4_list_i=[]
        T_4x4_list_i.append(image_id)
        for T_row  in T_Rt4x4:
            for T_col in T_row:
                T_4x4_list_i.append(T_col)
        T_4x4_list.append(T_4x4_list_i)
        #print(image_id,T_Rt4x4)
        
    with open(txt_name, 'w') as file:
        for row in T_4x4_list:
            
            line = ' '.join(map(str, row))
            file.write(f"{line}\n")
  
    #np.savetxt(out_txt, T_4x4_list)
    return T_4x4_list
        
    # Call the rendering script with the extrinsics file
    #subprocess.run(['python', render_script_path, '--extrinsics', extrinsics_file, '--output', f'{output_dir}/rendered_{image_id}.png'])

# if __name__ == "__main__":

#     path_txt="0测试数据1/d1_100mRTKColmap/sparse/0/"
#     colmap_images_path = path_txt+"images.txt"

   
#     camera_poses_T_4x4_output_dir = 'data/output/T_Rt4x4_.txt'
   
   
#     camera_poses_q_t = read_colmap_images_file(colmap_images_path)
#     T_4x4_list=images_pose_T4x4(camera_poses_q_t,camera_poses_T_4x4_output_dir)