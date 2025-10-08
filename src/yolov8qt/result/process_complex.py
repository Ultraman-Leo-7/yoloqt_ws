# -*- coding: utf-8 -*-
# 分离实部和虚部

import re

def process_complex_numbers(input_file, output_file):
    # 打开输入文件和输出文件
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        # 跳过前9行标题
        for _ in range(9):
            outfile.write(next(infile))  # 将标题行原样写入输出文件

        # 处理数据行
        for line in infile:
            # 使用正则表达式匹配复数部分
            line = re.sub(r'([+-]?\d+\.\d+)([+-])i', r'\1,0', line)  # 处理纯虚部的情况
            line = re.sub(r'([+-]?\d+\.\d+)([+-])(\d+\.\d+)i', r'\1,\3', line)  # 处理复数部分
            outfile.write(line)

# 输入文件和输出文件路径
input_file = 'test.txt'  # 替换为你的输入文件路径
output_file = 'test_processed.txt'  # 替换为你的输出文件路径

# 调用函数处理文件
process_complex_numbers(input_file, output_file)



