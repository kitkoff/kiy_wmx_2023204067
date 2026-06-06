/**
 * PID温度控制仿真系统 - 核心逻辑
 * v1.0 正式版
 */

// ==================== 全局状态 ====================
const AppState = {
    currentUser: null,
    currentRole: null,
    isRunning: false,
    isPaused: false,
    simulationId: null,
    startTime: null,
    dataHistory: [],
    curveVisibility: {
        sv: true,
        pv: true,
        u: true,
        error: true,
        interference: true
    }
};

// ==================== 用户管理 ====================
const UserManager = {
    // 初始化默认用户
    init() {
        if (!localStorage.getItem('pid_users')) {
            const defaultUsers = {
                'admin': { password: 'admin123', role: 'admin' },
                'user': { password: 'user123', role: 'user' }
            };
            localStorage.setItem('pid_users', JSON.stringify(defaultUsers));
        }
    },

    // 获取所有用户
    getUsers() {
        return JSON.parse(localStorage.getItem('pid_users') || '{}');
    },

    // 保存用户
    saveUsers(users) {
        localStorage.setItem('pid_users', JSON.stringify(users));
    },

    // 验证登录
    login(username, password) {
        const users = this.getUsers();
        if (users[username] && users[username].password === password) {
            return { success: true, role: users[username].role };
        }
        return { success: false };
    },

    // 修改密码
    changePassword(username, oldPassword, newPassword) {
        const users = this.getUsers();
        if (!users[username] || users[username].password !== oldPassword) {
            return { success: false, message: '当前密码错误' };
        }
        users[username].password = newPassword;
        this.saveUsers(users);
        return { success: true };
    },

    // 添加用户
    addUser(username, password, role) {
        const users = this.getUsers();
        if (users[username]) {
            return { success: false, message: '用户ID已存在' };
        }
        users[username] = { password, role };
        this.saveUsers(users);
        return { success: true };
    },

    // 删除用户
    deleteUser(username) {
        const users = this.getUsers();
        if (!users[username]) {
            return { success: false, message: '用户不存在' };
        }
        if (username === 'admin') {
            return { success: false, message: '不能删除默认管理员' };
        }
        delete users[username];
        this.saveUsers(users);
        return { success: true };
    }
};

// ==================== PID控制器 ====================
class PIDController {
    constructor(kp = 2.0, ti = 2.0, td = 0.5, outputLimit = null) {
        this.kp = kp;
        this.ti = ti;
        this.td = td;
        this.outputLimit = outputLimit; // [min, max] 或 null

        this.integral = 0;
        this.lastError = 0;
        this.lastOutput = 0;
        this.antiWindup = true;
    }

    // 重置
    reset() {
        this.integral = 0;
        this.lastError = 0;
        this.lastOutput = 0;
    }

    // 设置积分跟踪（用于手动/自动无扰切换）
    setIntegralTracking(output) {
        if (this.kp !== 0 && this.ti > 0) {
            this.integral = output / (this.kp / this.ti);
        }
    }

    // 计算控制量（位置式PID，带条件积分抗饱和）
    calculate(sp, pv, dt) {
        const error = sp - pv;
        const p = this.kp * error;

        // 条件积分抗饱和
        let shouldIntegrate = true;
        if (this.outputLimit && this.antiWindup) {
            const [uMin, uMax] = this.outputLimit;
            const outputBeforeI = p + (this.kp / this.ti) * this.integral;
            if ((outputBeforeI > uMax && error > 0) ||
                (outputBeforeI < uMin && error < 0)) {
                shouldIntegrate = false;
            }
        }

        if (shouldIntegrate) {
            this.integral += error * dt;
        }

        const i = (this.kp / this.ti) * this.integral;
        const d = this.kp * this.td * (error - this.lastError) / dt;

        let output = p + i + d;

        // 输出限幅
        if (this.outputLimit) {
            const [uMin, uMax] = this.outputLimit;
            output = Math.max(uMin, Math.min(uMax, output));
        }

        this.lastError = error;
        this.lastOutput = output;

        return output;
    }

    // 更新参数
    updateParams(kp, ti, td) {
        this.kp = kp;
        this.ti = ti;
        this.td = td;
    }
}

// ==================== 被控对象模型 ====================
class PlantModel {
    constructor(t1 = 1.0, t2 = 2.0, gain = 1.0) {
        this.t1 = t1;
        this.t2 = t2;
        this.gain = gain;

        // 状态变量（双惯性环节）
        this.state1 = 0; // 第一环节状态
        this.state2 = 0; // 第二环节状态
        this.output1 = 0; // 第一环节输出（中间变量，用于串级内环反馈）
        this.output2 = 0; // 第二环节输出（最终输出）
    }

    // 重置
    reset() {
        this.state1 = 0;
        this.state2 = 0;
        this.output1 = 0;
        this.output2 = 0;
    }

    // 更新参数
    updateParams(t1, t2, gain) {
        this.t1 = t1;
        this.t2 = t2;
        this.gain = gain;
    }

    // 更新模型（使用梯形法或简单欧拉法）
    update(input, dt) {
        // 第一惯性环节: T1 * dy1/dt + y1 = gain * u
        const k1 = (this.gain * input - this.output1) / this.t1;
        this.output1 += k1 * dt;

        // 第二惯性环节: T2 * dy2/dt + y2 = y1
        const k2 = (this.output1 - this.output2) / this.t2;
        this.output2 += k2 * dt;

        return {
            output: this.output2,    // 最终输出
            intermediate: this.output1  // 中间变量（用于串级内环）
        };
    }
}

// ==================== 反馈环节（一阶低通滤波器） ====================
class FeedbackFilter {
    constructor(tc = 0.1) {
        this.tc = tc;
        this.output = 0;
    }

    reset() {
        this.output = 0;
    }

    update(input, dt) {
        // H(s) = 1/(Tc*s + 1)
        const k = (input - this.output) / this.tc;
        this.output += k * dt;
        return this.output;
    }
}

// ==================== 仿真引擎 ====================
class SimulationEngine {
    constructor() {
        this.dt = 0.05; // 仿真步长 50ms
        this.time = 0;

        // 控制器
        this.pidMain = new PIDController(2.0, 2.0, 0.5, [-30, 30]);
        this.pidInner = new PIDController(1.0, 2.0, 0.0, [-30, 30]);

        // 被控对象
        this.plant = new PlantModel(1.0, 2.0, 1.0);

        // 反馈滤波器
        this.feedback = new FeedbackFilter(0.1);

        // 干扰
        this.interference = {
            active: false,
            amplitude: 0,
            duration: 0,
            startTime: 0,
            currentValue: 0
        };

        // 状态变量
        this.sv = 20;
        this.pv = 0;
        this.u = 0;
        this.manualU = 0;
        this.mode = 'auto'; // 'auto' 或 'manual'
        this.strategy = 'pid_single';

        // 前馈参数
        this.feedforwardGain = -0.5;

        // 历史数据
        this.history = [];
        this.maxHistoryPoints = 2000;

        // 图表数据
        this.chartData = {
            time: [],
            sv: [],
            pv: [],
            u: [],
            error: [],
            interference: []
        };
        this.maxChartPoints = 500;
    }

    // 重置仿真
    reset() {
        this.time = 0;
        this.pidMain.reset();
        this.pidInner.reset();
        this.plant.reset();
        this.feedback.reset();
        this.interference.active = false;
        this.interference.currentValue = 0;
        this.pv = 0;
        this.u = 0;
        this.history = [];
        this.chartData = {
            time: [],
            sv: [],
            pv: [],
            u: [],
            error: [],
            interference: []
        };
    }

    // 应用干扰
    applyInterference(amplitude, duration) {
        this.interference.active = true;
        this.interference.amplitude = amplitude;
        this.interference.duration = duration;
        this.interference.startTime = this.time;
    }

    // 更新干扰状态
    updateInterference() {
        if (this.interference.active) {
            if (this.time - this.interference.startTime < this.interference.duration) {
                this.interference.currentValue = this.interference.amplitude;
            } else {
                this.interference.active = false;
                this.interference.currentValue = 0;
            }
        } else {
            this.interference.currentValue = 0;
        }
    }

    // 更新PID参数
    updatePIDParams(kp, ti, td, kpInner, tiInner, tdInner) {
        const integralWas = this.pidMain.integral;
        this.pidMain.updateParams(kp, ti, td);
        this.pidMain.integral = integralWas; // 保持积分连续

        if (kpInner !== undefined) {
            const innerIntegralWas = this.pidInner.integral;
            this.pidInner.updateParams(kpInner, tiInner, tdInner);
            this.pidInner.integral = innerIntegralWas;
        }
    }

    // 更新被控对象参数
    updatePlantParams(t1, t2, gain) {
        this.plant.updateParams(t1, t2, gain);
    }

    // 设置控制模式
    setMode(mode) {
        if (this.mode !== mode) {
            this.mode = mode;
            if (mode === 'auto') {
                // 切换到自动时，积分跟踪当前手动输出
                this.pidMain.setIntegralTracking(this.manualU);
            }
        }
    }

    // 单步仿真
    step() {
        this.updateInterference();

        let uTotal = 0;

        switch (this.strategy) {
            case 'pid_unlimited':
                uTotal = this.simulatePIDUnlimited();
                break;
            case 'pid_single':
                uTotal = this.simulatePIDSingle();
                break;
            case 'feedforward':
                uTotal = this.simulateFeedforward();
                break;
            case 'cascade':
                uTotal = this.simulateCascade();
                break;
            case 'cascade_feedforward':
                uTotal = this.simulateCascadeFeedforward();
                break;
        }

        this.u = uTotal;

        // 更新被控对象
        const plantOutput = this.plant.update(uTotal, this.dt);

        // 被控对象输出 + 干扰
        const rawOutput = plantOutput.output + this.interference.currentValue;

        // 经过反馈环节得到PV
        this.pv = this.feedback.update(rawOutput, this.dt);

        // 记录数据
        this.recordData();

        this.time += this.dt;

        return {
            time: this.time,
            sv: this.sv,
            pv: this.pv,
            u: this.u,
            error: this.sv - this.pv,
            interference: this.interference.currentValue,
            intermediate: plantOutput.intermediate
        };
    }

    // 普通PID（无限幅）
    simulatePIDUnlimited() {
        if (this.mode === 'manual') {
            return this.manualU;
        }

        // 临时取消限幅
        const originalLimit = this.pidMain.outputLimit;
        this.pidMain.outputLimit = null;
        this.pidMain.antiWindup = false;

        const u = this.pidMain.calculate(this.sv, this.pv, this.dt);

        this.pidMain.outputLimit = originalLimit;
        this.pidMain.antiWindup = true;

        return u;
    }

    // 单回路PID（带抗饱和）
    simulatePIDSingle() {
        if (this.mode === 'manual') {
            // 手动模式时，控制器跟踪手动输出
            this.pidMain.setIntegralTracking(this.manualU);
            return this.manualU;
        }

        return this.pidMain.calculate(this.sv, this.pv, this.dt);
    }

    // 前馈+反馈控制
    simulateFeedforward() {
        if (this.mode === 'manual') {
            this.pidMain.setIntegralTracking(this.manualU);
            return this.manualU;
        }

        // 反馈部分
        const uFeedback = this.pidMain.calculate(this.sv, this.pv, this.dt);

        // 前馈补偿（假设干扰可测）
        const uFeedforward = this.feedforwardGain * this.interference.currentValue;

        let uTotal = uFeedback + uFeedforward;

        // 限幅
        uTotal = Math.max(-30, Math.min(30, uTotal));

        return uTotal;
    }

    // 串级PID控制
    simulateCascade() {
        if (this.mode === 'manual') {
            this.pidMain.setIntegralTracking(this.manualU);
            this.pidInner.setIntegralTracking(this.manualU);
            return this.manualU;
        }

        // 外环：控制最终输出
        const spInner = this.pidMain.calculate(this.sv, this.pv, this.dt);

        // 内环：控制中间变量（第一环节输出）
        const u = this.pidInner.calculate(spInner, this.plant.output1, this.dt);

        return u;
    }

    // 串级+前馈控制
    simulateCascadeFeedforward() {
        if (this.mode === 'manual') {
            this.pidMain.setIntegralTracking(this.manualU);
            this.pidInner.setIntegralTracking(this.manualU);
            return this.manualU;
        }

        // 外环
        const spInner = this.pidMain.calculate(this.sv, this.pv, this.dt);

        // 内环
        const uInner = this.pidInner.calculate(spInner, this.plant.output1, this.dt);

        // 前馈补偿（作用于内环输出）
        const uFeedforward = this.feedforwardGain * this.interference.currentValue;

        let uTotal = uInner + uFeedforward;
        uTotal = Math.max(-30, Math.min(30, uTotal));

        return uTotal;
    }

    // 记录数据
    recordData() {
        const error = this.sv - this.pv;

        this.chartData.time.push(this.time);
        this.chartData.sv.push(this.sv);
        this.chartData.pv.push(this.pv);
        this.chartData.u.push(this.u);
        this.chartData.error.push(error);
        this.chartData.interference.push(this.interference.currentValue);

        // 限制图表点数
        if (this.chartData.time.length > this.maxChartPoints) {
            this.chartData.time.shift();
            this.chartData.sv.shift();
            this.chartData.pv.shift();
            this.chartData.u.shift();
            this.chartData.error.shift();
            this.chartData.interference.shift();
        }

        // 记录完整历史（用于导出）
        this.history.push({
            time: this.time,
            sv: this.sv,
            pv: this.pv,
            u: this.u,
            error: error,
            interference: this.interference.currentValue
        });

        if (this.history.length > this.maxHistoryPoints) {
            this.history.shift();
        }
    }

    // 获取图表数据
    getChartData() {
        return this.chartData;
    }

    // 获取历史数据
    getHistory() {
        return this.history;
    }
}

// ==================== 图表绘制 ====================
class ChartRenderer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.resize();

        window.addEventListener('resize', () => this.resize());
    }

    resize() {
        const rect = this.canvas.parentElement.getBoundingClientRect();
        this.canvas.width = rect.width - 40;
        this.canvas.height = rect.height - 80;
    }

    draw(data, visibility) {
        const { width, height } = this.canvas;
        const ctx = this.ctx;

        // 清空画布
        ctx.clearRect(0, 0, width, height);

        if (!data.time || data.time.length === 0) return;

        // 计算数据范围
        let minY = Infinity, maxY = -Infinity;

        const checkRange = (arr, visible) => {
            if (!visible) return;
            arr.forEach(v => {
                if (v < minY) minY = v;
                if (v > maxY) maxY = v;
            });
        };

        checkRange(data.sv, visibility.sv);
        checkRange(data.pv, visibility.pv);
        checkRange(data.u, visibility.u);
        checkRange(data.error, visibility.error);
        checkRange(data.interference, visibility.interference);

        if (minY === Infinity) {
            minY = 0;
            maxY = 30;
        }

        // 添加边距
        const range = maxY - minY || 1;
        minY -= range * 0.1;
        maxY += range * 0.1;

        const padding = { left: 50, right: 20, top: 20, bottom: 40 };
        const chartWidth = width - padding.left - padding.right;
        const chartHeight = height - padding.top - padding.bottom;

        // 绘制网格
        ctx.strokeStyle = 'rgba(255,255,255,0.1)';
        ctx.lineWidth = 1;

        // 横向网格
        for (let i = 0; i <= 5; i++) {
            const y = padding.top + (chartHeight / 5) * i;
            ctx.beginPath();
            ctx.moveTo(padding.left, y);
            ctx.lineTo(width - padding.right, y);
            ctx.stroke();

            // Y轴标签
            ctx.fillStyle = '#888';
            ctx.font = '11px sans-serif';
            ctx.textAlign = 'right';
            const value = maxY - (maxY - minY) * (i / 5);
            ctx.fillText(value.toFixed(1), padding.left - 8, y + 4);
        }

        // 纵向网格
        const timeRange = data.time[data.time.length - 1] - data.time[0];
        for (let i = 0; i <= 5; i++) {
            const x = padding.left + (chartWidth / 5) * i;
            ctx.beginPath();
            ctx.moveTo(x, padding.top);
            ctx.lineTo(x, height - padding.bottom);
            ctx.stroke();

            // X轴标签（时间）
            if (timeRange > 0) {
                ctx.fillStyle = '#888';
                ctx.font = '11px sans-serif';
                ctx.textAlign = 'center';
                const time = data.time[0] + timeRange * (i / 5);
                ctx.fillText(time.toFixed(1) + 's', x, height - padding.bottom + 20);
            }
        }

        // 绘制曲线
        const drawCurve = (arr, color, visible, dashed = false) => {
            if (!visible || arr.length < 2) return;

            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            if (dashed) {
                ctx.setLineDash([5, 5]);
            } else {
                ctx.setLineDash([]);
            }

            ctx.beginPath();
            for (let i = 0; i < arr.length; i++) {
                const x = padding.left + (i / (arr.length - 1)) * chartWidth;
                const y = padding.top + (1 - (arr[i] - minY) / (maxY - minY)) * chartHeight;

                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            ctx.stroke();
            ctx.setLineDash([]);
        };

        drawCurve(data.sv, '#ff4444', visibility.sv);
        drawCurve(data.pv, '#00ff88', visibility.pv);
        drawCurve(data.u, '#4488ff', visibility.u);
        drawCurve(data.error, '#00ffff', visibility.error);
        drawCurve(data.interference, '#ff00ff', visibility.interference, true);

        // 绘制坐标轴标签
        ctx.fillStyle = '#aaa';
        ctx.font = '12px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('时间 (秒)', width / 2, height - 8);

        ctx.save();
        ctx.translate(15, height / 2);
        ctx.rotate(-Math.PI / 2);
        ctx.fillText('数值', 0, 0);
        ctx.restore();
    }
}

// ==================== UI控制器 ====================
const UIController = {
    engine: null,
    chart: null,
    historyChart: null,

    init() {
        UserManager.init();
        this.engine = new SimulationEngine();
        this.chart = new ChartRenderer('chartCanvas');
        this.historyChart = new ChartRenderer('historyCanvas');

        this.bindEvents();
        this.updateUI();
    },

    bindEvents() {
        // 登录表单
        document.getElementById('loginForm').addEventListener('submit', (e) => {
            e.preventDefault();
            this.handleLogin();
        });

        // 控制策略切换
        document.getElementById('controlStrategy').addEventListener('change', (e) => {
            this.engine.strategy = e.target.value;
            const cascadeParams = document.getElementById('cascadeParams');
            cascadeParams.style.display =
                (e.target.value === 'cascade' || e.target.value === 'cascade_feedforward')
                    ? 'block' : 'none';
        });

        // 手自动切换
        document.getElementById('autoManual').addEventListener('change', (e) => {
            this.engine.setMode(e.target.value);
            document.getElementById('manualU').disabled = e.target.value === 'auto';
        });

        // 参数变化时更新引擎
        const bindParam = (id, callback) => {
            document.getElementById(id).addEventListener('change', callback);
        };

        bindParam('svValue', (e) => { this.engine.sv = parseFloat(e.target.value); });
        bindParam('manualU', (e) => { this.engine.manualU = parseFloat(e.target.value); });

        bindParam('kpValue', () => this.updatePIDParams());
        bindParam('tiValue', () => this.updatePIDParams());
        bindParam('tdValue', () => this.updatePIDParams());
        bindParam('kpInner', () => this.updatePIDParams());
        bindParam('tiInner', () => this.updatePIDParams());
        bindParam('tdInner', () => this.updatePIDParams());

        bindParam('t1Value', () => this.updatePlantParams());
        bindParam('t2Value', () => this.updatePlantParams());
        bindParam('gainValue', () => this.updatePlantParams());
    },

    updatePIDParams() {
        const kp = parseFloat(document.getElementById('kpValue').value) || 0;
        const ti = parseFloat(document.getElementById('tiValue').value) || 1;
        const td = parseFloat(document.getElementById('tdValue').value) || 0;
        const kpInner = parseFloat(document.getElementById('kpInner').value) || 0;
        const tiInner = parseFloat(document.getElementById('tiInner').value) || 1;
        const tdInner = parseFloat(document.getElementById('tdInner').value) || 0;

        this.engine.updatePIDParams(kp, ti, td, kpInner, tiInner, tdInner);
    },

    updatePlantParams() {
        const t1 = parseFloat(document.getElementById('t1Value').value) || 1;
        const t2 = parseFloat(document.getElementById('t2Value').value) || 1;
        const gain = parseFloat(document.getElementById('gainValue').value) || 1;

        this.engine.updatePlantParams(t1, t2, gain);
    },

    handleLogin() {
        const username = document.getElementById('username').value.trim();
        const password = document.getElementById('password').value;
        const errorDiv = document.getElementById('loginError');

        if (!username || !password) {
            errorDiv.textContent = '请输入用户名和密码';
            return;
        }

        const result = UserManager.login(username, password);

        if (result.success) {
            AppState.currentUser = username;
            AppState.currentRole = result.role;
            this.showMainInterface();
            errorDiv.textContent = '';
        } else {
            errorDiv.textContent = '用户名或密码错误';
        }
    },

    showMainInterface() {
        document.getElementById('loginContainer').style.display = 'none';
        document.getElementById('mainContainer').style.display = 'block';

        document.getElementById('currentUserDisplay').textContent = AppState.currentUser;
        document.getElementById('currentRoleDisplay').textContent =
            AppState.currentRole === 'admin' ? '管理员' : '普通用户';

        // 管理员菜单项
        if (AppState.currentRole === 'admin') {
            document.querySelectorAll('.admin-only').forEach(el => {
                el.style.display = 'block';
            });
        }
    },

    updateUI() {
        if (AppState.isRunning && !AppState.isPaused) {
            const data = this.engine.step();

            // 更新显示
            document.getElementById('displaySV').textContent = data.sv.toFixed(1);
            document.getElementById('displayPV').textContent = data.pv.toFixed(1);
            document.getElementById('displayU').textContent = data.u.toFixed(1);
            document.getElementById('displayError').textContent = data.error.toFixed(1);
            document.getElementById('displayInterference').textContent = data.interference.toFixed(1);
            document.getElementById('uValue').value = data.u.toFixed(1);

            // 绘制图表
            const chartData = this.engine.getChartData();
            this.chart.draw(chartData, AppState.curveVisibility);

            // 检查干扰按钮状态
            if (!this.engine.interference.active) {
                document.getElementById('btnInterference').classList.remove('active');
                document.getElementById('btnInterference').textContent = '施加方波干扰';
            }
        }

        requestAnimationFrame(() => this.updateUI());
    }
};

// ==================== 全局函数 ====================

// 启动仿真
function startSimulation() {
    if (AppState.isRunning) return;

    AppState.isRunning = true;
    AppState.isPaused = false;

    document.getElementById('btnStart').style.display = 'none';
    document.getElementById('btnPause').style.display = 'block';
    document.getElementById('btnStop').style.display = 'block';

    showToast('仿真已启动', 'success');
}

// 暂停仿真
function pauseSimulation() {
    if (!AppState.isRunning) return;

    AppState.isPaused = !AppState.isPaused;

    const btn = document.getElementById('btnPause');
    if (AppState.isPaused) {
        btn.textContent = '继续仿真';
        showToast('仿真已暂停', 'info');
    } else {
        btn.textContent = '暂停仿真';
        showToast('仿真已继续', 'success');
    }
}

// 停止仿真
function stopSimulation() {
    AppState.isRunning = false;
    AppState.isPaused = false;

    UIController.engine.reset();

    document.getElementById('btnStart').style.display = 'block';
    document.getElementById('btnPause').style.display = 'none';
    document.getElementById('btnStop').style.display = 'none';
    document.getElementById('btnPause').textContent = '暂停仿真';

    // 重置显示
    document.getElementById('displayPV').textContent = '0.0';
    document.getElementById('displayU').textContent = '0.0';
    document.getElementById('displayError').textContent = document.getElementById('svValue').value;
    document.getElementById('displayInterference').textContent = '0.0';
    document.getElementById('uValue').value = '0';

    showToast('仿真已停止', 'info');
}

// 施加干扰
function applyInterference() {
    if (!AppState.isRunning) {
        showToast('请先启动仿真', 'error');
        return;
    }

    const amp = parseFloat(document.getElementById('interferenceAmp').value) || 0;
    const duration = parseFloat(document.getElementById('interferenceDuration').value) || 5;

    if (amp === 0) {
        showToast('干扰振幅不能为0', 'error');
        return;
    }

    UIController.engine.applyInterference(amp, duration);

    const btn = document.getElementById('btnInterference');
    btn.classList.add('active');
    btn.textContent = `干扰中 (${duration}秒)`;

    showToast(`已施加 ${amp} 干扰，持续 ${duration} 秒`, 'info');

    // 倒计时更新
    let remaining = duration;
    const countdown = setInterval(() => {
        remaining -= 1;
        if (remaining > 0 && UIController.engine.interference.active) {
            btn.textContent = `干扰中 (${remaining.toFixed(0)}秒)`;
        } else {
            clearInterval(countdown);
        }
    }, 1000);
}

// 切换曲线显示
function toggleCurve(curve) {
    AppState.curveVisibility[curve] = !AppState.curveVisibility[curve];

    // 更新图例样式
    const legendItems = document.querySelectorAll('.legend-item');
    const index = ['sv', 'pv', 'u', 'error', 'interference'].indexOf(curve);
    if (index >= 0 && legendItems[index]) {
        legendItems[index].classList.toggle('hidden', !AppState.curveVisibility[curve]);
    }
}

// 退出登录
function logout() {
    AppState.currentUser = null;
    AppState.currentRole = null;

    stopSimulation();

    document.getElementById('mainContainer').style.display = 'none';
    document.getElementById('loginContainer').style.display = 'flex';
    document.getElementById('username').value = '';
    document.getElementById('password').value = '';

    // 隐藏管理员菜单
    document.querySelectorAll('.admin-only').forEach(el => {
        el.style.display = 'none';
    });

    showToast('已退出登录', 'info');
}

// 退出程序
function exitApp() {
    if (confirm('确定要退出程序吗？')) {
        window.close();
        // 如果无法关闭窗口，显示提示
        showToast('请手动关闭浏览器标签页', 'info');
    }
}

// 显示修改密码弹窗
function showChangePassword() {
    document.getElementById('changePasswordModal').classList.add('active');
    document.getElementById('oldPassword').value = '';
    document.getElementById('newPassword').value = '';
    document.getElementById('confirmPassword').value = '';
}

// 修改密码
function changePassword() {
    const oldPwd = document.getElementById('oldPassword').value;
    const newPwd = document.getElementById('newPassword').value;
    const confirmPwd = document.getElementById('confirmPassword').value;

    if (!oldPwd || !newPwd || !confirmPwd) {
        showToast('请填写所有字段', 'error');
        return;
    }

    if (newPwd !== confirmPwd) {
        showToast('两次输入的新密码不一致', 'error');
        return;
    }

    const result = UserManager.changePassword(AppState.currentUser, oldPwd, newPwd);

    if (result.success) {
        closeModal('changePasswordModal');
        showToast('密码修改成功', 'success');
    } else {
        showToast(result.message, 'error');
    }
}

// 显示用户管理弹窗
function showUserManagement() {
    if (AppState.currentRole !== 'admin') {
        showToast('权限不足', 'error');
        return;
    }

    document.getElementById('userManagementModal').classList.add('active');
    refreshUserTable();
}

// 刷新用户表
function refreshUserTable() {
    const tbody = document.getElementById('userTableBody');
    const users = UserManager.getUsers();

    tbody.innerHTML = '';

    for (const [username, info] of Object.entries(users)) {
        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${username}</td>
            <td>${info.role === 'admin' ? '管理员' : '普通用户'}</td>
            <td>
                <button class="btn-small btn-delete" onclick="deleteUser('${username}')"
                    ${username === 'admin' ? 'disabled' : ''}>删除</button>
            </td>
        `;
        tbody.appendChild(row);
    }
}

// 显示添加用户弹窗
function showAddUser() {
    document.getElementById('addUserModal').classList.add('active');
    document.getElementById('newUserId').value = '';
    document.getElementById('newUserPassword').value = '';
    document.getElementById('newUserRole').value = 'user';
}

// 添加用户
function addUser() {
    const username = document.getElementById('newUserId').value.trim();
    const password = document.getElementById('newUserPassword').value;
    const role = document.getElementById('newUserRole').value;

    if (!username || !password) {
        showToast('请填写所有字段', 'error');
        return;
    }

    const result = UserManager.addUser(username, password, role);

    if (result.success) {
        closeModal('addUserModal');
        refreshUserTable();
        showToast('用户添加成功', 'success');
    } else {
        showToast(result.message, 'error');
    }
}

// 删除用户
function deleteUser(username) {
    if (!confirm(`确定要删除用户 "${username}" 吗？`)) return;

    const result = UserManager.deleteUser(username);

    if (result.success) {
        refreshUserTable();
        showToast('用户删除成功', 'success');
    } else {
        showToast(result.message, 'error');
    }
}

// 显示历史曲线
function showHistory() {
    document.getElementById('historyModal').classList.add('active');

    // 设置默认时间范围
    const now = new Date();
    const yesterday = new Date(now - 24 * 60 * 60 * 1000);

    document.getElementById('historyEnd').value = formatDateTimeLocal(now);
    document.getElementById('historyStart').value = formatDateTimeLocal(yesterday);

    queryHistory();
}

// 格式化日期时间
function formatDateTimeLocal(date) {
    const pad = (n) => n.toString().padStart(2, '0');
    return `${date.getFullYear()}-${pad(date.getMonth() + 1)}-${pad(date.getDate())}T${pad(date.getHours())}:${pad(date.getMinutes())}`;
}

// 查询历史
function queryHistory() {
    const history = UIController.engine.getHistory();

    if (history.length === 0) {
        showToast('暂无历史数据', 'info');
        return;
    }

    const data = {
        time: history.map(h => h.time),
        sv: history.map(h => h.sv),
        pv: history.map(h => h.pv),
        u: history.map(h => h.u),
        error: history.map(h => h.error),
        interference: history.map(h => h.interference)
    };

    UIController.historyChart.draw(data, AppState.curveVisibility);
}

// 导出历史
function exportHistory() {
    const history = UIController.engine.getHistory();

    if (history.length === 0) {
        showToast('暂无历史数据可导出', 'error');
        return;
    }

    // 创建CSV内容
    let csv = '时间,设定值(SV),过程值(PV),控制量(u),误差,干扰\n';

    history.forEach(h => {
        csv += `${h.time.toFixed(2)},${h.sv.toFixed(2)},${h.pv.toFixed(2)},${h.u.toFixed(2)},${h.error.toFixed(2)},${h.interference.toFixed(2)}\n`;
    });

    // 下载文件
    const blob = new Blob(['﻿' + csv], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `PID历史数据_${new Date().toLocaleString().replace(/[/:]/g, '-')}.csv`;
    link.click();

    showToast('历史数据已导出', 'success');
}

// 显示关于
function showAbout() {
    document.getElementById('aboutModal').classList.add('active');
}

// 关闭弹窗
function closeModal(modalId) {
    document.getElementById(modalId).classList.remove('active');
}

// 显示提示
function showToast(message, type = 'info') {
    const toast = document.getElementById('toast');
    toast.textContent = message;
    toast.className = `toast ${type} show`;

    setTimeout(() => {
        toast.classList.remove('show');
    }, 3000);
}

// ==================== 初始化 ====================
document.addEventListener('DOMContentLoaded', () => {
    UIController.init();
});

// 全局错误处理
window.onerror = function(msg, url, line) {
    console.error('错误:', msg, '行:', line);
    showToast('发生错误: ' + msg, 'error');
    return true;
};
