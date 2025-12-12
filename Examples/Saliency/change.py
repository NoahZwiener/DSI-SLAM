import re

def parse_level_file(level_file_path):
    """解析level文件（新格式），构建(x, y)到level的映射字典"""
    level_map = {}
    # 正则匹配新格式：[ORBextractor]: KeyPoint at level 4: (731.981, 207.36)
    pattern = r'\[ORBextractor\]: KeyPoint at level (\d+): \((\d+\.?\d*), (\d+\.?\d*)\)'
    
    with open(level_file_path, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            match = re.match(pattern, line)
            if match:
                level = match.group(1)  # 提取level值（数字）
                x = match.group(2)      # 提取x坐标（整数/浮点数）
                y = match.group(3)      # 提取y坐标（整数/浮点数）
                # 转换为浮点数作为键，避免100和100.0不匹配
                key = (float(x), float(y))
                level_map[key] = level
            else:
                print(f"警告：第{line_num}行格式不匹配，跳过：{line}")
    return level_map

def generate_new_file(first_file_path, level_file_path, output_file_path):
    """按第一个文件顺序，生成包含x,y,S,level的新文件（适配新格式）"""
    # 第一步：解析level文件获取映射关系
    level_map = parse_level_file(level_file_path)
    
    # 第二步：解析第一个文件并输出新文件（第一个文件无分号）
    with open(first_file_path, 'r', encoding='utf-8') as f_in, \
         open(output_file_path, 'w', encoding='utf-8') as f_out:
        
        for line_num, line in enumerate(f_in, 1):
            line = line.strip()
            if not line:
                continue
            # 分割行内容（仅空格分隔，无分号）
            parts = line.split()
            if len(parts) != 3:
                print(f"警告：第{line_num}行格式错误（需3个字段），跳过：{line}")
                continue
            
            # 提取x、y、S（无分号，直接取）
            x_str, y_str, s_str = parts
            
            # 转换为浮点数匹配level映射
            try:
                x = float(x_str)
                y = float(y_str)
            except ValueError:
                print(f"警告：第{line_num}行x/y不是数值，跳过：{line}")
                continue
            
            # 查找对应的level
            key = (x, y)
            if key not in level_map:
                print(f"警告：第{line_num}行({x}, {y})未找到对应level，跳过")
                continue
            
            # 按格式写入新文件：x, y, S, level
            output_line = f"{x_str}, {y_str}, {s_str}, {level_map[key]}"
            f_out.write(output_line + '\n')
    
    print(f"处理完成！新文件已保存至：{output_file_path}")



if __name__ == "__main__":
    # ====================== 请修改以下文件路径 ======================
    FIRST_FILE_PATH = "kitti_05_saliency.txt"   # 第一个输入文件路径
    LEVEL_FILE_PATH = "keypoint_levels.txt"  # 第二个输入文件路径
    OUTPUT_FILE_PATH = "output.txt" # 输出文件路径
    # ===============================================================
    
    generate_new_file(FIRST_FILE_PATH, LEVEL_FILE_PATH, OUTPUT_FILE_PATH)
