import pandas as pd

csv_assincrono = 'data/assincrono/exp_2025-05-26_00h-43m-28s/imu.csv'
csv_sincrono = 'data/sincrono/exp_2025-05-26_00h-56m-54s/imu.csv'

# Lê os dois arquivos CSV
df_assincrono = pd.read_csv(csv_assincrono)
df_sincrono = pd.read_csv(csv_sincrono)

# Remove a coluna de timestamp e converte cada linha em uma lista de floats
pontos1 = df_assincrono.drop(columns=['timestamp']).values.tolist()
pontos2 = df_sincrono.drop(columns=['timestamp']).values.tolist()

# Agora pontos1 e pontos2 são listas de pontos (cada ponto é uma lista de 6 valores)
print("Pontos do arquivo 1:", pontos1[:3])  # Exemplo: mostra os 3 primeiros pontos
print("Pontos do arquivo 2:", pontos2[:3])