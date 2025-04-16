import os

def rename_npy_files(folder_path):
    # 获取文件夹中的所有文件
    files = os.listdir(folder_path)
    
    # 过滤出所有 .npy 文件
    npy_files = [file for file in files if file.endswith('.npy')]
    
    # 遍历 .npy 文件并重命名
    for i, file in enumerate(npy_files):
        # 创建新的文件名
        new_name = f"Area_3_pipe_{i}.npy"
        
        # 构建完整的旧文件路径和新文件路径
        old_file_path = os.path.join(folder_path, file)
        new_file_path = os.path.join(folder_path, new_name)
        
        # 重命名文件
        os.rename(old_file_path, new_file_path)
        print(f"Renamed: {file} to {new_name}")

# 使用示例
folder_path = './dataset_npy'  # 替换为你的文件夹路径
rename_npy_files(folder_path)
