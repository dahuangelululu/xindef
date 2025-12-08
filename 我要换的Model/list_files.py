import os
import csv

# 你要扫描的文件夹路径（修改成你自己的）
folder = r"E:\keypoint-4_ test"

# 输出文件
output_file = "file_list.txt"

with open(output_file, "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["文件名", "完整路径"])
    for root, dirs, files in os.walk(folder):
        for file in files:
            full_path = os.path.join(root, file)
            writer.writerow([file, full_path])

print(f"文件路径已保存到 {output_file}")