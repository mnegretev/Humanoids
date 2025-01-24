import cv2
import numpy as np

def nothing(x):
    pass

# Inicializa la cámara web
cap = cv2.VideoCapture(0)  # Usa 0 para la cámara predeterminada, o cambia el índice si tienes más cámaras

# Configura la ventana con deslizadores para HLS
cv2.namedWindow('Control HLS')
cv2.createTrackbar('H_min', 'Control HLS', 0, 179, nothing)
cv2.createTrackbar('H_max', 'Control HLS', 179, 179, nothing)
cv2.createTrackbar('L_min', 'Control HLS', 0, 255, nothing)
cv2.createTrackbar('L_max', 'Control HLS', 255, 255, nothing)
cv2.createTrackbar('S_min', 'Control HLS', 0, 255, nothing)
cv2.createTrackbar('S_max', 'Control HLS', 255, 255, nothing)

while True:
    # Captura un fotograma desde la cámara
    ret, frame = cap.read()
    if not ret:
        print("No se pudo acceder a la cámara.")
        break

    # Convierte el fotograma a HLS
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

    # Leer los valores de los trackbars
    h_min = cv2.getTrackbarPos('H_min', 'Control HLS')
    h_max = cv2.getTrackbarPos('H_max', 'Control HLS')
    l_min = cv2.getTrackbarPos('L_min', 'Control HLS')
    l_max = cv2.getTrackbarPos('L_max', 'Control HLS')
    s_min = cv2.getTrackbarPos('S_min', 'Control HLS')
    s_max = cv2.getTrackbarPos('S_max', 'Control HLS')

    # Crear los límites inferiores y superiores
    lower = np.array([h_min, l_min, s_min])
    upper = np.array([h_max, l_max, s_max])

    # Crear una máscara basada en los valores de los trackbars
    mask = cv2.inRange(hls, lower, upper)

    # Aplicar la máscara al fotograma original
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Mostrar los resultados
    cv2.imshow('Original', frame)
    cv2.imshow('Máscara', mask)
    cv2.imshow('Resultado', result)

    # Salir del bucle si se presiona 'Esc'
    if cv2.waitKey(1) & 0xFF == 27:  # Tecla 'Esc'
        break

# Libera la cámara y cierra las ventanas
cap.release()
cv2.destroyAllWindows()