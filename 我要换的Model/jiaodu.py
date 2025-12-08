import numpy as np
import math

def out(landmarks):
    A = [landmarks[0],landmarks[1]]#center
    B = [landmarks[2],landmarks[3]]#far
    C = [landmarks[4],landmarks[5]]#zero
    D = [landmarks[6],landmarks[7]]#end
    # 计算向量AB和AC的坐标
    AB = np.array(A) - np.array(B)
    AC = np.array(A) - np.array(C)

    # 计算向量的点积和模长
    dot_product = np.dot(AB, AC)

    norm_AB = np.linalg.norm(AB)
    norm_AC = np.linalg.norm(AC)

    # 计算夹角θ（弧度）
    theta = np.arccos(dot_product / (norm_AB * norm_AC))

    if C[0]<A[0] :
        if C[1] < A[1]:#zero在左上方
            if B[0]>=C[0] and B[1]<=D[1]:
        # 计算夹角θ（弧度）
                theta = theta
        # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            if B[1]>=C[1] and B[0]<=D[0]:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
        if C[1]>A[1]:#zero在左下方
            if B[1]<=C[1] and B[0]<=D[0]:
                theta = theta
        # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            if B[0]>=C[0] and B[1]>=D[1]:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
    if C[0]>A[0] :
        if C[1] > A[1]:#zero在右下方
            if B[0]<=C[0] and B[1]>=D[1]:
                theta = theta
        # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            if B[1]<=C[1] and B[0]>=D[0]:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
        if C[1]<A[1]:#zero在右上方
            if B[1]>=C[1] and B[0]>=D[0]:
                theta = theta
        # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            if B[0]<=C[0] and B[1]<=D[1]:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
    if C[0]==A[0]:#zero与end连线竖直
        if C[1]<A[1]:
            if B[0]>=A[0]:
                theta = theta
                # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            else:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
        else:
            if B[0]<=A[0]:
                theta = theta
                # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            else:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
    if C[1]==A[1]:#zero与end连线水平
        if C[0]<A[0]:
            if B[1]<=A[1]:
                theta = theta
                # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            else:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')
        else:
            if B[1]>=A[1]:
                theta = theta
                # 将弧度转换为角度（以度为单位）
                theta_degrees = np.degrees(theta)
                print('夹角小于180°')
            else:
                theta = 2 * math.pi - theta
                theta_degrees = np.degrees(theta)
                print('夹角大于180°')

    end = (theta_degrees / 180) * 50
    print("夹角 θ（弧度）：", theta)
    print("夹角 θ（度）：", theta_degrees)
    print("最终读数：", end)
    return end


def out_jiandan(landmarks):
    A = [landmarks[0],landmarks[1]]#center
    B = [landmarks[2],landmarks[3]]#far
    C = [landmarks[4],landmarks[5]]#zero
    # 计算向量AB和AC的坐标
    AB = np.array(A) - np.array(B)
    AC = np.array(A) - np.array(C)

    # 计算向量的点积和模长
    dot_product = np.dot(AB, AC)
    norm_AB = np.linalg.norm(AB)
    norm_AC = np.linalg.norm(AC)
    if C[0] < B[0]:
        # 计算夹角θ（弧度）
        theta = np.arccos(dot_product / (norm_AB * norm_AC))
        # 将弧度转换为角度（以度为单位）
        theta_degrees = np.degrees(theta)
        print('夹角小于180°')
    else:
        theta = 2 * math.pi - np.arccos(dot_product / (norm_AB * norm_AC))
        theta_degrees = np.degrees(theta)
        print('夹角大于180°')
    end = (theta_degrees / 180) * 50
    print("夹角 θ（弧度）：", theta)
    print("夹角 θ（度）：", theta_degrees)
    print("最终读数：", end)
    return end