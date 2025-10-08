# -*- coding: utf-8 -*-


def convert_comma_to_space(input_file, output_file):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            # 替换逗号为空格
            new_line = line.replace(',', ' ')
            outfile.write(new_line)
            
# 输入文件和输出文件路径
input_file = 'test_processed.txt'  # 替换为你的输入文件路径
output_file = 'comsol_result_final.txt'  # 替换为你的输出文件路径

# 调用函数处理文件
convert_comma_to_space(input_file, output_file)
