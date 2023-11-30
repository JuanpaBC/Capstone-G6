import cv2
import os

def extract_frames(video_path, output_folder, interval=0.1):
    # Verificar si la carpeta de salida existe, de lo contrario, crearla
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Abrir el video
    video_capture = cv2.VideoCapture(video_path)

    # Obtener la tasa de fotogramas del video
    fps = video_capture.get(cv2.CAP_PROP_FPS)

    # Calcular el número total de frames
    total_frames = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))

    # Calcular el número de frames que se deben extraer cada intervalo
    frames_to_extract = int(fps * interval)

    # Inicializar el contador de frames
    current_frame = 0

    while True:
        # Calcular el tiempo actual en segundos
        current_time = current_frame / fps

        # Leer el frame actual
        ret, frame = video_capture.read()

        # Verificar si se ha llegado al final del video
        if not ret:
            break

        # Verificar si el tiempo actual supera el siguiente intervalo
        if current_time >= interval:
            # Crear el nombre del archivo de salida
            output_filename = f"frame_{int(current_time * 10)}.png"

            # Guardar el frame como imagen en la carpeta de salida
            output_path = os.path.join(output_folder, output_filename)
            cv2.imwrite(output_path, frame)

            # Incrementar el intervalo
            interval += 0.1

        # Incrementar el contador de frames
        current_frame += 1

    # Liberar los recursos
    video_capture.release()

    print(f"Se han extraído {current_frame} frames de un total de {total_frames} frames.")

# Ruta del video de entrada
video_path = "video.mp4"

# Carpeta de salida para los frames
output_folder = "carpeta_de_salida"

# Intervalo de tiempo para extraer frames (en segundos)
interval = 0.1

# Llamar a la función para extraer frames
extract_frames(video_path, output_folder, interval)
