from gurobipy import *


class Car:

    def __init__(self, v, z, Vd, Zd, N, name, dt=0.1):
        self.name = name
        self.v = v
        self.z = z
        self.Vd = Vd
        self.Zd = Zd

        # ..............MPC data...............
        self.Q = 1  # * np.identity(1)
        self.R = 10  # * np.identity(1)
        self.N = N  # horizon
        self.dt = dt  # [s]
        self.Ds = 7  # Safety distance [m]
        self.Dl = 25  # lateral distance
        self.V_max = 80
        self.A_max = 30
        self.L = 6  # number of lanes
        self.Mmax = self.L - 1
        self.mmin = -self.L + 1

        # ..........historical data ............
        self.hist_vp1 = []
        self.hist_zp1 = []

        self.hist_vp2 = []
        self.hist_zp2 = []

        # % hold on
        self.hist_v = []
        self.hist_z = []
        self.hist_a = []
        self.hist_d = []

        self.hist_a1 = []
        self.hist_b1 = []
        self.hist_ab1 = []
        self.hist_n1 = []
        self.hist_th1 = []
        self.hist_f1 = []
        self.hist_g1 = []
        self.hist_h1 = []

        self.hist_a2 = []
        self.hist_b2 = []
        self.hist_ab2 = []
        self.hist_n2 = []
        self.hist_th2 = []
        self.hist_f2 = []
        self.hist_g2 = []
        self.hist_h2 = []

        self.hist_ll1 = []
        self.hist_lr1 = []
        self.hist_ll2 = []
        self.hist_lr2 = []

        self._time = []
        self.t_now = 0
        self.m = None
        # Dynamic constraints
        self.cnt_z = None
        self.cnt_v = None
        self.cnt_zd = None
        self.cnt_vd = None
        self.cnt_v2 = None
        self.cnt_z2 = None
        self.cnt_d = None
        self.a12 = None
        self.n12 = None
        self.th12 = None

    def create_model(self):
        self.m = Model(self.name)
        N = self.N
        # ------------desired states-----------
        Zd = self.m.addVar(lb=0, ub=self.L, vtype=GRB.INTEGER, name='Zd')  # carril deseado
        Vd = self.m.addVar(lb=0, ub=self.V_max, vtype=GRB.CONTINUOUS, name='Vd')  # velocidad deseada
        DS = self.m.addVars(N, lb=0, ub=self.V_max, vtype=GRB.CONTINUOUS, name='Ds')  # velocidad seguridad

        # -------------local vehicle---------------
        v = self.m.addVars(N + 1, lb=0, ub=self.V_max, vtype=GRB.CONTINUOUS, name='v')  # velocidad del vehiculo actual
        a = self.m.addVars(N, lb=-self.A_max, ub=self.A_max, vtype=GRB.CONTINUOUS,
                           name='a')  # acceleration of actual vehicle
        z = self.m.addVars(N + 1, lb=1, ub=self.L, vtype=GRB.INTEGER, name='z')  # carril actual
        ll = self.m.addVars(N, vtype=GRB.BINARY, name='ll')  # paso izquierda
        lr = self.m.addVars(N, vtype=GRB.BINARY, name='lr')  # paso derecha
        # -------------- neighbor ---------------
        lr2 = self.m.addVar(vtype=GRB.BINARY, name='lr2')  # paso derecha
        v_2 = self.m.addVar(vtype=GRB.CONTINUOUS, name='v_2')  # velocidad del otro vehculo
        z_2 = self.m.addVar(lb=1, ub=self.L, vtype=GRB.INTEGER, name='z_2')  # carril del vehiculo j
        # ------ distance between two vehicles ------
        dis12 = self.m.addVars(N + 1, lb=-10000, ub=10000, vtype=GRB.CONTINUOUS,
                               name='dis12')  # distancia entre vehiculo 1 y 2
        # %% binary variables

        a12 = self.m.addVars(N, vtype=GRB.BINARY, name="a12")
        b12 = self.m.addVars(N, vtype=GRB.BINARY, name="b12")
        ab12 = self.m.addVars(N, vtype=GRB.BINARY, name="ab12")
        n12 = self.m.addVars(N, vtype=GRB.BINARY, name="n12")
        th12 = self.m.addVars(N, vtype=GRB.BINARY, name="th12")
        f12 = self.m.addVars(N, vtype=GRB.CONTINUOUS, name="f12")
        g12 = self.m.addVars(N, vtype=GRB.CONTINUOUS, name="g12")
        h12 = self.m.addVars(N, vtype=GRB.CONTINUOUS, name="h12")

        Q = self.Q
        R = self.R
        # set objective
        self.m.setObjective(quicksum(Q * (v[k + 1] - Vd) ** 2 + R * (z[k + 1] - Zd) ** 2 for k in range(N)),
                            GRB.MINIMIZE)

        # create constraints
        self.m.addConstrs(z[k] - lr[k] <= z[k + 1] for k in range(N))
        self.m.addConstrs(z[k + 1] <= z[k] + ll[k] for k in range(N))
        self.m.addConstrs(ll[k] + lr[k] <= 1 for k in range(N))
        self.m.addConstrs((v[k + 1] == v[k] + self.dt * a[k] for k in range(N)), name='speed')
        # self.m.addConstrs( mmin  <= z_2-z[k+1] for k in range(N))
        # self.m.addConstrs(z_2-z[k+1]  <= Mmax for k in range(N))
        # %% neighbor constraints
        self.m.addConstrs((dis12[k + 1] == dis12[k] + self.dt * (v_2 - v[k]) for k in range(N)), name="distance")

        # Equation 12
        func = [z_2 - z[k] for k in range(N)]
        self.log_min(n12, func, 0)
        self.log_may(th12, func, 0)
        self.log_and(a12, n12, th12)
        # Equation 13
        self.log_may(b12, dis12, 0)
        # Equation 18
        self.log_and(ab12, a12, b12)
        # Equation 21
        # self.log_imp(f12, dis12, ab12)
        self.m.addConstrs(((ab12[k] == 1) >> (f12[k] - dis12[k] == 0)) for k in range(N))
        self.m.addConstrs(((a12[k] == 1) >> (g12[k] - DS[k] == 0)) for k in range(N))
        self.m.addConstrs(((a12[k] == 1) >> (h12[k] - dis12[k] == 0)) for k in range(N))
        self.m.addConstrs(-2 * f12[k] + g12[k] + h12[k] <= 0 for k in range(N))
        # # Equation 22
        # self.log_imp(g12, DS, a12)
        # # Equation 23
        # self.log_imp(h12, dis12, a12)

        # M = 10000
        # m = -M
        # eps = 10e-3
        # N = self.N
        # # equation 12
        # # log min
        # self.m.addConstrs((M) * n12[k] <= M - (z_2 - z[k]) for k in range(N))
        # self.m.addConstrs((-m + eps) * n12[k] >= eps - (z_2 - z[k]) for k in range(N))
        # # log_may
        # self.m.addConstrs(((- m) * th12[k] <= (z_2 - z[k]) - m) for k in range(N))
        # self.m.addConstrs(((M + eps) * th12[k] >= (z_2 - z[k]) + eps) for k in range(N))
        # # log_and
        # self.m.addConstrs(a12[k] <= n12[k] for k in range(N))
        # self.m.addConstrs(a12[k] <= th12[k] for k in range(N))
        # self.m.addConstrs(n12[k] + th12[k] - a12[k] <= 1 for k in range(N))
        #
        # # equation 13
        # # log_may
        # self.m.addConstrs(((- m) * b12[k] <= dis12[k] - m) for k in range(N))
        # self.m.addConstrs(((M + eps) * b12[k] >= dis12[k] + eps) for k in range(N))
        #
        # # equation 18
        # # log_and
        # self.m.addConstrs(ab12[k] <= a12[k] for k in range(N))
        # self.m.addConstrs(ab12[k] <= b12[k] for k in range(N))
        # self.m.addConstrs(a12[k] + b12[k] - ab12[k] <= 1 for k in range(N))
        #
        # # equation 21
        # # log_imp g, func, a
        # self.m.addConstrs(m * ab12[k] <= f12[k] for k in range(N))
        # self.m.addConstrs(f12[k] <= M * ab12[k] for k in range(N))
        # self.m.addConstrs(-M * (1 - ab12[k]) <= f12[k] - dis12[k] for k in range(N))
        # self.m.addConstrs(f12[k] - dis12[k] <= -m * (1 - ab12[k]) for k in range(N))

        self.m.addConstrs((DS[k] == self.Ds for k in range(N)), name="Ds")
        # dynamic constraints
        self.cnt_z = self.m.addConstr(z[0] == 1, name="Z[0]")
        self.cnt_v = self.m.addConstr(v[0] == 1, name="v[0]")
        self.cnt_zd = self.m.addConstr(Zd == 1, name="Zdes[0]")
        self.cnt_vd = self.m.addConstr(Vd == 1, name="Vdes[0]")
        self.cnt_v2 = self.m.addConstr(v_2 == 1, name="vel[1]")
        self.cnt_z2 = self.m.addConstr(z_2 == 1, name="zel[1]")
        self.cnt_d = self.m.addConstr(dis12[0] == 20, name="dis[0]")

    def update_cnt(self, zel, vel, zdes, vdes, v2, z2, dis):
        self.cnt_z.rhs = zel
        self.cnt_v.rhs = vel
        self.cnt_zd.rhs = zdes
        self.cnt_vd.rhs = vdes
        self.cnt_v2.rhs = v2
        self.cnt_z2.rhs = z2
        self.cnt_d.rhs = dis

    def get_var(self, N):
        vel = []
        acel = []
        lane = []
        trn_lef = []
        trn_right = []
        for i in range(N):
            vel.append(self.m.getVarByName('v[{}]'.format(i)).x)
            acel.append(self.m.getVarByName('a[{}]'.format(i)).x)
            lane.append(self.m.getVarByName('z[{}]'.format(i)).x)
            trn_lef.append(self.m.getVarByName('ll[{}]'.format(i)).x)
            trn_right.append(self.m.getVarByName('lr[{}]'.format(i)).x)

        self.hist_vp1.append(vel)
        self.hist_zp1.append(lane)
        self.hist_ll1.append(trn_lef)
        self.hist_lr1.append(trn_right)
        return acel[0], lane[1]

    def make_step(self, acel, z):
        """
        Run a simulation step by passing the current input.
        Returns the current measurement y.
        """
        self.v = self.v + self.dt * acel
        self.z = z
        self.hist_v.append(self.v)
        self.hist_z.append(self.z)
        self._time.append(self.t_now)
        self.t_now += self.dt

        return self.v, z

    # def log_min(self, bin, func1, func2, c):
    #     M = 10000
    #     m = -M
    #     eps = 10e-3
    #     N = self.N
    #     self.m.addConstrs((M - c) * bin[k] <= M - (func1 + func2[k]) for k in range(N))
    #     self.m.addConstrs((c - m + eps) * bin[k] >= eps + c - (func1 + func2[k]) for k in range(N))
    def log_min(self, bin, func, c=0, M=10000, eps=10e-3):
        m = -M
        N = self.N
        self.m.addConstrs((M - c) * bin[k] <= M - func[k] for k in range(N))
        self.m.addConstrs((c - m + eps) * bin[k] >= eps + c - func[k] for k in range(N))

    def log_may(self, bin, func, c=0, M=10000, eps=10e-3):
        M = 10000
        m = -M
        eps = 10e-3
        N = self.N
        self.m.addConstrs(((c - m) * bin[k] <= func[k] - m) for k in range(N))
        self.m.addConstrs(((M - c + eps) * bin[k] >= func[k] - c + eps) for k in range(N))

    def log_and(self, bin, a, b):
        N = self.N
        self.m.addConstrs(bin[k] <= a[k] for k in range(N))
        self.m.addConstrs(bin[k] <= b[k] for k in range(N))
        self.m.addConstrs(a[k] + b[k] - bin[k] <= 1 for k in range(N))

    def log_imp(self, g, func, a, M=10000):
        m = -M
        N = self.N
        for k in range(N):
            self.m.addConstr((a[k] == 1) >> (g[k] - func[k] == 0))
        #
        # self.m.addConstrs(m * a[k] <= g[k] for k in range(N))
        # self.m.addConstrs(g[k] <= M * a[k] for k in range(N))
        # self.m.addConstrs(-M * (1 - a[k]) <= g[k] - func[k] for k in range(N))
        # self.m.addConstrs(g[k] - func[k] <= -m * (1 - a[k]) for k in range(N))
