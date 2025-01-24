import cv2
import numpy as np

def nothing(x):
    pass

# Inicializa la cámara web
cap = cv2.VideoCapture(0)  # Usa 0 para la cámara predeterminada, o cambia el índice si tienes más cámaras


# Crear una ventana con deslizadores
cv2.namedWindow('Control HSV')
cv2.createTrackbar('H_min', 'Control HSV', 0, 179, nothing)
cv2.createTrackbar('H_max', 'Control HSV', 179, 179, nothing)
cv2.createTrackbar('S_min', 'Control HSV', 0, 255, nothing)
cv2.createTrackbar('S_max', 'Control HSV', 255, 255, nothing)
cv2.createTrackbar('V_min', 'Control HSV', 0, 255, nothing)
cv2.createTrackbar('V_max', 'Control HSV', 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo acceder a la cámara.")
        break

    # Convierte el fotograma a HLS
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Leer los valores de los trackbars
    h_min = cv2.getTrackbarPos('H_min', 'Control HSV')
    h_max = cv2.getTrackbarPos('H_max', 'Control HSV')
    s_min = cv2.getTrackbarPos('S_min', 'Control HSV')
    s_max = cv2.getTrackbarPos('S_max', 'Control HSV')
    v_min = cv2.getTrackbarPos('V_min', 'Control HSV')
    v_max = cv2.getTrackbarPos('V_max', 'Control HSV')

    # Crear los límites inferiores y superiores
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Crear una máscara basada en los valores de los trackbars
    mask = cv2.inRange(hsv, lower, upper)

    # Aplicar la máscara a la imagen original
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