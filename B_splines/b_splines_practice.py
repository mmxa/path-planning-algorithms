import matplotlib.pyplot as plt
import math

t = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1.0]   # 节点向量
P = [-0.5, 0.8, 1.2, 1.3, 1.0]   # 控制点
k = 3
n = 5


def recursion_N(k, i, u):           # degree k, segment index i and  x axis for definition
    if k == 0:
        if t[i] <= u < t[i+1]:
            return 1
        else:
            return 0
    res = (u-t[i]) / (t[i+k] - t[i]) * recursion_N(k-1, i, u) + \
          (t[i+k+1] - u) / (t[i+k+1] - t[i+1]) * recursion_N(k-1, i+1, u)
    return res


if __name__ == '__main__':
    C = []
    for i in range(101):
        temp = 0.0
        for j in range(n):                # 单个点计算：
            temp += P[j] * recursion_N(3, j, 0.01*i)
        C.append(temp)
    x = [i*0.01 for i in range(101)]
    x_p = [i*1/4 for i in range(5)]
    plt.plot(x, C)
    plt.plot(x_p, P, '.')
    plt.show()

