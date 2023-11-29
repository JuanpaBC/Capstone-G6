import cv2
import os
import time

def capture_frames(output_folder, interval=0.1, duration=10):
    # Verificar si la carpeta de salida existe, de lo contrario, crearla
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Inicializar la cámara
    capture = cv2.VideoCapture(0)  # 0 para la cámara predeterminada, puedes ajustar esto si tienes múltiples cámaras

    # Inicializar el tiempo de inicio
    start_time = time.time()

    # Inicializar el tiempo de duración
    end_time = start_time + duration

    frame_count = 0

    while time.time() < end_time:
        # Leer el frame actual
        ret, frame = capture.read()

        # Verificar si la lectura del frame fue exitosa
        if not ret:
            print("Error al leer el frame")
            break

        # Obtener el tiempo transcurrido desde el inicio en segundos
        elapsed_time = time.time() - start_time

        # Verificar si ha pasado el intervalo de tiempo deseado
        if elapsed_time >= interval:
            # Crear el nombre del archivo de salida
            output_filename = f"frame_{frame_count}.png"

            # Guardar el frame como imagen en la carpeta de salida
            output_path = os.path.join(output_folder, output_filename)
            cv2.imwrite(output_path, frame)

            # Incrementar el contador de frames
            frame_count += 1

            # Reiniciar el tiempo de inicio
            start_time = time.time()

        # Mostrar el frame en una ventana (opcional)
        cv2.imshow("Capturando Frames", frame)

        # Salir del bucle cuando se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar los recursos
    capture.release()
    cv2.destroyAllWindows()

    print(f"Se han capturado {frame_count} frames en {duration} segundos.")

# Carpeta de salida para los frames
output_folder = "carpeta_de_salida_2"

# Intervalo de tiempo para capturar frames (en segundos)
interval = 0.1

# Duración de la captura en segundos
duration = 7

# Llamar a la función para capturar frames
capture_frames(output_folder, interval, duration)
