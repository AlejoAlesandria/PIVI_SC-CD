import serial
import time
import csv
from datetime import datetime

# Configura el puerto serial y la velocidad de transmisión (baud rate)
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Cambia '/dev/ttyUSB0' al puerto correcto en tu sistema

def get_filename():
    # Genera un nombre de archivo basado en la fecha y hora actuales
    now = datetime.now()
    return f"data_{now.strftime('%Y%m%d_%H%M%S')}.csv"

def read_serial_and_save():
    filename = get_filename()
    with open(filename, "w", newline='') as file:  # Abrir archivo CSV
        writer = csv.writer(file)
        writer.writerow(["Time", "Sensor", "PWM"])  # Escribir encabezado
        print(f"Guardando datos en {filename}...")
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()  # Leer línea del puerto serial
                if line:
                    print(f"Recibido: {line}")
                    data = line.split(",")  # Separar los datos por comas
                    if len(data) == 3:  # Asegurarse de que la línea tenga tres valores
                        writer.writerow(data)  # Escribir datos en el archivo CSV
            except KeyboardInterrupt:
                print("Interrupción del usuario. Cerrando...")
                break
            except Exception as e:
                print(f"Error: {e}")
                break

if __name__ == "__main__":
    # Espera para asegurar que el puerto serial esté listo
    time.sleep(2)
    read_serial_and_save()
