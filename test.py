import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Funções de filtro
def high_pass_filter(data, cutoff, fs):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(1, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)

def low_pass_filter(data, cutoff, fs):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(1, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# Seu array de entrada
data = np.array([-0.87, -0.45, -0.21, 1.75, -3.22, -0.98])

# Dividir os dados em acelerômetro e giroscópio
accel_data = data[:3]
gyro_data = data[3:]

# Defina o período de amostragem
sample_period = 1 / 256  # Defina seu período aqui

# Filtros nos dados do acelerômetro
filtered_accel = high_pass_filter(accel_data, 0.001, 1/sample_period)
filtered_accel = np.abs(filtered_accel)
filtered_accel = low_pass_filter(filtered_accel, 5, 1/sample_period)

# Detecção de períodos estacionários
threshold = 0.05
stationary = filtered_accel < threshold

# Simulação do AHRS (precisa ser substituído por um algoritmo real)
class AHRS:
    def __init__(self, sample_period, kp, kp_init):
        self.sample_period = sample_period
        self.kp = kp
        self.kp_init = kp_init
        self.quaternion = np.array([1, 0, 0, 0])  # Inicialização do quaternion

    def update_imu(self, gyro, accel):
        # Implementação simplificada para exemplo
        pass

ahrs_algorithm = AHRS(sample_period=1/256, kp=1, kp_init=1)
quat = []

# Simulação do processamento para cada amostra de tempo
for i in range(len(data) // 6):
    accel_sample = data[i*6:i*6+3]
    gyro_sample = data[i*6+3:i*6+6]

    ahrs_algorithm.update_imu(gyro_sample, accel_sample)
    quat.append(ahrs_algorithm.quaternion)

quat = np.array(quat)

# Cálculo das acelerações translacionais
def quatern_rotate(accel, quaternion):
    # Implementação da rotação com quaternions
    # Esta função deve realizar a rotação do vetor de aceleração pelo quaternion
    pass

# Rotacionando as acelerações do corpo para o referencial da Terra
rotated_accel = []
for i in range(len(quat)):
    rotated_accel.append(quatern_rotate(accel_data, quat[i]))

rotated_accel = np.array(rotated_accel)

# Convertendo as acelerações para m/s^2
accel = rotated_accel * 9.81

# Cálculo da velocidade
vel = np.zeros_like(accel)
for t in range(1, len(vel)):
    vel[t] = vel[t - 1] + accel[t] * sample_period
    if stationary[t]:
        vel[t] = np.array([0, 0, 0])  # Zerando a velocidade quando estiver estacionário

# Cálculo da posição
pos = np.zeros_like(vel)
for t in range(1, len(pos)):
    pos[t] = pos[t - 1] + vel[t] * sample_period  # Integração da velocidade para obter a posição

# Remoção da deriva integral
vel_drift = np.zeros_like(vel)
stationary_start = np.where(np.diff(stationary) == -1)[0]
stationary_end = np.where(np.diff(stationary) == 1)[0]

for i in range(len(stationary_end)):
    drift_rate = vel[stationary_end[i] - 1] / (stationary_end[i] - stationary_start[i])
    enum = np.arange(stationary_end[i] - stationary_start[i])
    drift = np.outer(enum, drift_rate)
    vel_drift[stationary_start[i]:stationary_end[i], :] = drift

# Removendo a deriva integral
vel = vel - vel_drift


# Visualização da trajetória 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot da trajetória
ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], label='Trajetória 3D')
ax.scatter(pos[0, 0], pos[0, 1], pos[0, 2], color='green', label='Início')
ax.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], color='red', label='Fim')

# Definindo rótulos dos eixos
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')

# Adicionando legenda
ax.legend()

# Mostrando o gráfico
plt.show()
