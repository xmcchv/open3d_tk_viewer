import os
import shutil

def process_point_cloud(input_folder, output_folder):
    # 确保输出文件夹存在
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    # 遍历输入文件夹中的所有txt文件
    txt_files = [f for f in os.listdir(input_folder) if f.endswith('.txt')]
    
    for i, txt_file in enumerate(txt_files, start=1):
        # 创建场景文件夹
        scene_folder = os.path.join(output_folder, f'pipe_{i}')
        os.makedirs(scene_folder, exist_ok=True)
        
        # 创建Annotations子文件夹
        annotations_folder = os.path.join(scene_folder, 'Annotations')
        os.makedirs(annotations_folder, exist_ok=True)
        
        # 初始化存储不同类别点的列表
        body_points = []
        quexian_points = []
        all_points = []
        
        # 读取原始文件
        input_path = os.path.join(input_folder, txt_file)
        with open(input_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                # print(len(parts))
                if len(parts) >= 4:  # 确保有xyz和标签
                    # print(parts)
                    x, y, z, label_str = parts[:4]
                    # print(label_str)
                    try:
                        label = int(float(label_str))
                    except ValueError:
                        continue  # 如果标签不是数字，跳过该点
                    
                    # 根据标签设置RGB值并创建新行
                    if label == 0:
                        rgb = '255 255 255'
                        new_line = f"{x} {y} {z} {rgb}\n"
                        body_points.append(new_line)
                        # print(f"Added to body_points: {new_line.strip()}")  # Debug line
                    elif label == 1:
                        rgb = '0 0 0'
                        new_line = f"{x} {y} {z} {rgb}\n"
                        quexian_points.append(new_line)
                        # print(f"Added to quexian_points: {new_line.strip()}")  # Debug line
                    # else:
                        # print(f"Label {label} not processed (not 0 or 1)")  # Debug line
                    all_points.append(new_line)
        print(f"Number of body points: {len(body_points)}")  # Debug line
        print(f"Number of quexian points: {len(quexian_points)}")  # Debug line
        print(f"Number of all points: {len(all_points)}")  # Debug line

        # 写入pipe_i.txt
        pipe_path = os.path.join(scene_folder, f'pipe_{i}.txt')
        with open(pipe_path, 'w') as f:
            f.writelines(all_points)
        
        # 写入body_i.txt
        body_path = os.path.join(annotations_folder, f'bg_{i}.txt')
        with open(body_path, 'w') as f:
            f.writelines(body_points)
        
        # 写入quexian_i.txt
        quexian_path = os.path.join(annotations_folder, f'grab_{i}.txt')
        with open(quexian_path, 'w') as f:
            f.writelines(quexian_points)
        
        print(f'Processed {txt_file} -> pipe_{i}')

if __name__ == '__main__':
    input_folder = './dataset'  # 替换为你的输入文件夹路径
    output_folder = './semantic_seg_dataset'      # 输出文件夹名称
    
    process_point_cloud(input_folder, output_folder)
    print('All files processed successfully!')