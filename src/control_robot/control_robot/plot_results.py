import pandas as pd
import matplotlib.pyplot as plt

def plot():
    try:
        data = pd.read_csv('tracking_results.csv')
    except FileNotFoundError:
        print("No se encontró el archivo CSV. ¿Corriste el nodo de tracking?")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Trayectoria
    ax1.plot(data['x'], data['y'], 'b-', label='Ruta del Robot')
    ax1.scatter(1.0, 0, color='green', marker='X', label='Meta (1.0, 0)')
    ax1.set_title('Trayectoria en el Plano XY')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.legend()
    ax1.grid(True)

    # Error
    ax2.plot(data['error'], 'r-', label='Error (m)')
    ax2.set_title('Evolución del Error')
    ax2.set_xlabel('Muestras (10Hz)')
    ax2.set_ylabel('Distancia a la meta')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig('grafica_desempeño.png')
    print("Gráfica guardada como 'grafica_desempeño.png'")
    plt.show()

if __name__ == "__main__":
    plot()
