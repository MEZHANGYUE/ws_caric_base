# -*- coding: utf-8 -*-
import open3d as o3d
import os

def merge_pcd_files(input_folder, output_file):
    # 创建一个空的点云对象
    combined_cloud = o3d.geometry.PointCloud()

    # 获取输入文件夹中的所有pcd文件
    pcd_files = [f for f in os.listdir(input_folder) if f.endswith(".pcd")]

    # 逐个加载并合并pcd文件
    for pcd_file in pcd_files:
        file_path = os.path.join(input_folder, pcd_file)
        cloud = o3d.io.read_point_cloud(file_path)
        combined_cloud += cloud

    # 保存合并后的点云到一个新的pcd文件
    o3d.io.write_point_cloud(output_file, combined_cloud)

if __name__ == "__main__":
    # 指定输入文件夹和输出文件
    input_folder = "/home/yue/ws_caric/src/record/output"
    output_file = "/home/yue/ws_caric/src/record/output/cloud.pcd"

    # 执行合并操作
    merge_pcd_files(input_folder, output_file)
