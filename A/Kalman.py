class Kalman(object):
    def __init__(self, LastP, Now_P, out, Kg, Q, R):
        self.LastP = LastP
        self.Now_P = Now_P
        self.out = out
        self.Kg = Kg
        self.Q = Q
        self.R = R
    def kalmanFilter(self, input):
        #预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
        self.Now_P = self.LastP + self.Q
        #卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
        self.Kg = self.Now_P / (self.Now_P + self.R)
        #更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
        self.out = self.out + self.Kg * (input -self.out)#因为这一次的预测值就是上一次的输出值
        #更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
        self.LastP = (1-self.Kg) * self.Now_P
        return self.out
