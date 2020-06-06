import matplotlib.pyplot as plt
import numpy as np


class cothiod:
    def __init__(self, k0, a, b, c):
        self.k0 = k0
        self.a = a
        self.b = b
        self.c = c

    def calc_k(self, s):
        return self.k0 + self.a * s + self.b * s ** 2 + self.c * s ** 3

    def delta_theta(self, s):
        return self.k0 * s + self.a * s ** 2 / 2 + self.b * s ** 3 / 3 + self.c * s ** 4 / 4

    def cal_dx(self, s, theta0=0, x0=0):
        res = 0
        for j in np.arange(0, s, 0.01):
            res += np.cos(theta0 + self.delta_theta(j)) * 0.01
        return x0 + res

    def cal_dy(self, s, theta0=0, y0=0):
        res = 0
        for j in np.arange(0, s, 0.01):
            res += np.sin(theta0 + self.delta_theta(j)) * 0.01
        return y0 + res

    def calc_x_y(self, s, theta0=0, xy=(0, 0)):
        x = []
        y = []
        for i in np.arange(0, 2, 0.01):
            x.append(coth.cal_dx(i, theta0, xy[0]))
            y.append(coth.cal_dy(i, theta0, xy[1]))
        return x, y


if __name__ == '__main__':/6
    coth = cothiod(0.5, 0.05, 1, 1)
    start = [3, 5]
    x, y = coth.calc_x_y(2, np.pi / 3, start)
    plt.plot(x, y, '-r')
    plt.axis('equal')
    plt.show()
