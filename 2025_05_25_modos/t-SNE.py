import pandas as pd
from sklearn.manifold import TSNE
import matplotlib.pyplot as plt

csv_assincrono = 'data/assincrono/exp_2025-05-26_00h-43m-28s/imu.csv'
csv_sincrono = 'data/sincrono/exp_2025-05-26_00h-56m-54s/imu.csv'

df_assincrono = pd.read_csv(csv_assincrono)
df_sincrono = pd.read_csv(csv_sincrono)

# pontos1 = df_assincrono.drop(columns=['timestamp']).values.tolist()
# pontos2 = df_sincrono.drop(columns=['timestamp']).values.tolist()

pontos1 = df_assincrono.values.tolist()
pontos2 = df_sincrono.values.tolist()

dados = pontos1 + pontos2
labels = [0]*len(pontos1) + [1]*len(pontos2)

tsne = TSNE(n_components=2, random_state=42)
dados_embedded = tsne.fit_transform(dados)

plt.figure(figsize=(8, 6))
plt.scatter(
    dados_embedded[:len(pontos1), 0], dados_embedded[:len(pontos1), 1],
    c='blue', label='Assíncrono', alpha=0.6
)
plt.scatter(
    dados_embedded[len(pontos1):, 0], dados_embedded[len(pontos1):, 1],
    c='red', label='Síncrono', alpha=0.6
)
plt.legend()
plt.title('t-SNE dos dados IMU')
plt.xlabel('Dim 1')
plt.ylabel('Dim 2')
plt.savefig('tsne_imu.png')