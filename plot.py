import matplotlib.pyplot as plt
import numpy as np

colori = plt.cm.tab10.colors

# Dati
tests = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
delete = [4924.75, 6996.35, 14791.27, 0, 4901.47, 0, 6968.06]
delete_std = [145.36, 163.92, 608.04, 0, 138.63, 0, 183.81]
reservation = [4953.75, 6993.60, 14149.30 , 20315.10, 4947.20, 6994.15, 6999.45]
reservation_std = [174.38, 171.7, 195.77, 267.79, 171.38, 165.08, 159.29]

# larghezza delle barre
larghezza_barre = 0.35
larghezza_std = 0.2

# Posizione delle barre sull'asse x
posizioni = np.arange(len(tests))

# Creazione del grafico a barre
fig, ax = plt.subplots()
barre1 = ax.bar(posizioni - larghezza_barre/2, delete, larghezza_barre, label='DELETE', color = colori[0])
barre2 = ax.bar(posizioni + larghezza_barre/2, reservation, larghezza_barre, label='RESERVATION', color = colori[1])
barre3 = ax.bar(posizioni - larghezza_std/2, delete_std, larghezza_std, label='DELETE_STD', color=colori[2])
barre4 = ax.bar(posizioni + larghezza_std/2, reservation_std, larghezza_std, label='RESERVATION_STD', color=colori[3])
# Aggiunta delle etichette, titolo e legenda
ax.set_xlabel('Group of Tests')
ax.set_ylabel('Makespan')
ax.set_title('Grafico a barre con due serie di dati')
ax.set_xticks(posizioni)
ax.set_xticklabels(tests)
ax.legend()

def aggiungi_valori_sopra_barre(barre):
    for barra in barre:
        altezza = barra.get_height()
        ax.annotate('{}'.format(altezza),
                    xy=(barra.get_x() + barra.get_width() / 2, altezza),
                    xytext=(0, 3),
                    textcoords='offset points',
                    ha='center', va='bottom')

# Aggiunta dei valori sopra le barre
aggiungi_valori_sopra_barre(barre1)
aggiungi_valori_sopra_barre(barre2)
aggiungi_valori_sopra_barre(barre3)
aggiungi_valori_sopra_barre(barre4)






# Mostrare il grafico
plt.show()
