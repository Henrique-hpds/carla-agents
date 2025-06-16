import pandas as pd
from sklearn.manifold import TSNE
import matplotlib.pyplot as plt
import numpy as np

csv_assincrono = 'data/assincrono/exp_2025-05-26_00h-43m-28s/imu.csv'
csv_sincrono = 'data/sincrono/exp_2025-05-26_00h-56m-54s/imu.csv'

df_assincrono = pd.read_csv(csv_assincrono)
df_sincrono = pd.read_csv(csv_sincrono)

dados_assincrono = df_assincrono.drop(columns=['timestamp']).values
dados_sincrono = df_sincrono.drop(columns=['timestamp']).values

fft_assincrono = np.abs(np.fft.fft(dados_assincrono, axis=1))
fft_sincrono = np.abs(np.fft.fft(dados_sincrono, axis=1))

fft_assincrono = fft_assincrono[:, :fft_assincrono.shape[1] // 2]
fft_sincrono = fft_sincrono[:, :fft_sincrono.shape[1] // 2]

dados_freq = np.vstack([fft_assincrono, fft_sincrono])
labels = [0]*len(fft_assincrono) + [1]*len(fft_sincrono)

tsne = TSNE(n_components=2, random_state=42)
dados_embedded = tsne.fit_transform(dados_freq)

plt.figure(figsize=(8, 6))
plt.scatter(
    dados_embedded[:len(fft_assincrono), 0], dados_embedded[:len(fft_assincrono), 1],
    c='blue', label='Assíncrono', alpha=0.6
)
plt.scatter(
    dados_embedded[len(fft_assincrono):, 0], dados_embedded[len(fft_assincrono):, 1],
    c='red', label='Síncrono', alpha=0.6
)
plt.legend()
plt.title('t-SNE dos dados IMU (Domínio da Frequência)')
plt.xlabel('Dim 1')
plt.ylabel('Dim 2')
plt.savefig('tsne_imu_freq.png')