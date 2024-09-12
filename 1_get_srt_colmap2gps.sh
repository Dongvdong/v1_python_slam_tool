#!/bin/bash
source ~/anaconda3/etc/profile.d/conda.sh #激活conda环境

data_dir="/home/r9000k/v2_project/data/NWPU/"
colmap_res_txt="${data_dir}/sparse/0/images.txt"   # 图像colmap重建位姿结果
video_gps_txt="${data_dir}/FHY_config/FHY_gps.txt" # 图像GNSS数据txt (时间戳 经纬高 空格分开)
gps_ned_ori="${data_dir}/FHY_config/GNSS_config.yaml" #图像gnss参考点输入文件
colmap2gnss_SRt_xml="${data_dir}/sparse/srt_colmap2gnss.yaml" #计算的srt关系保存路径

# pip install geographiclib
conda activate gaussian_splatting
python ex2_colmap2gnss.py --colmap_res_txt $colmap_res_txt \
                          --gps_ned_ori $gps_ned_ori \
                          --video_gps_txt $video_gps_txt \
                          --colmap2gnss_SRt_xml $colmap2gnss_SRt_xml