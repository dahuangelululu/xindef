import os
import json
from add.jiaodu import *
keypoint_class = ['center', 'far', 'zero', 'pi']
txtfile = 'real_2.txt'

with open(txtfile, 'w', encoding='utf-8') as txt_file:  # 使用'w'模式打开文件，如果文件已存在则会被清空
    os.chdir('val')
    for labelme_path in os.listdir():
        if not labelme_path.endswith('.json'):
            continue  # 跳过非JSON文件

        with open(labelme_path, 'r', encoding='utf-8') as f:
            labelme = json.load(f)
            bbox_keypoints_dict = {}

            for each_ann in labelme['shapes']:
                if each_ann['shape_type'] == 'rectangle':
                    bbox_top_left_x = int(min(each_ann['points'][0][0], each_ann['points'][1][0]))
                    bbox_bottom_right_x = int(max(each_ann['points'][0][0], each_ann['points'][1][0]))
                    bbox_top_left_y = int(min(each_ann['points'][0][1], each_ann['points'][1][1]))
                    bbox_bottom_right_y = int(max(each_ann['points'][0][1], each_ann['points'][1][1]))

                    # 初始化yolo_str并添加边界框信息（如果需要的话）
                    yolo_str = ''  # 这里可以根据需要添加边界框信息到yolo_str中

                    # 查找属于这个边界框的关键点
                    for point_ann in labelme['shapes']:
                        if point_ann['shape_type'] == 'point' and \
                                bbox_top_left_x <= int(point_ann['points'][0][0]) <= bbox_bottom_right_x and \
                                bbox_top_left_y <= int(point_ann['points'][0][1]) <= bbox_bottom_right_y:
                            label = point_ann['label']
                            x = int(point_ann['points'][0][0])
                            y = int(point_ann['points'][0][1])
                            bbox_keypoints_dict[label] = [x, y]

                            # 构建yolo_str，包含所有关键点的信息
                    for each_class in keypoint_class:
                        if each_class in bbox_keypoints_dict:
                            x, y = bbox_keypoints_dict[each_class]
                            yolo_str += '{:.5f} {:.5f}  '.format(x, y)
                        else:
                            yolo_str += '0 0  '

                            # 写入txt文件
                    # 使用空格分割字符串
                    yolo_array = yolo_str.split()
                    # 将字符串列表中的每个元素转换为浮点数（或整数，如果适用）
                    yolo_array = [float(num) for num in yolo_array]
                    result = out_jiandan(yolo_array)
                    end = str(result)
                    # print(yolo_array)
                    yolo_str += end
                    txt_file.write(yolo_str.strip() + '\n')  # 去除末尾的空格并添加一个换行符