#!/usr/bin/env python3
import rospy
import pandas as pd
import ast
import matplotlib.pyplot as plt
from std_msgs.msg import String
from mpl_toolkits.mplot3d import Axes3D  # Importación necesaria

def load_csv(file_path):
    try:
        # Read CSV assuming a single column with space-separated values
        data = pd.read_csv(file_path, delim_whitespace=True, header=None)
        data.columns = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        rospy.loginfo(f"Loaded {file_path} successfully.")
        return data
    except Exception as e:
        rospy.logerr(f"Failed to load {file_path}: {e}")
        return None

def plot():
    rospy.loginfo("Plotting data from CSV files.")
    
    # Graficar cada conjunto de datos en una ventana diferente
    for idx, df in enumerate(dataframes):
        if df is not None:
            x_vals = df['x'].tolist()
            y_vals = df['y'].tolist()
            z_vals = df['z'].tolist()

            # Crear una nueva figura para cada archivo de datos
            fig = plt.figure(figsize=(10, 6))
            ax = fig.add_subplot(111, projection='3d')

            # Graficar la trayectoria
            ax.plot(x_vals, y_vals, z_vals, marker='o', label=f'Trayectoria {idx+1}')

            # Agregar etiquetas
            ax.set_xlabel('Eje X')
            ax.set_ylabel('Eje Y')
            ax.set_zlabel('Eje Z')

            # Agregar leyenda y mostrar la gráfica
            ax.legend()
            plt.show()

def main():
    global dataframes
    rospy.init_node('plot', anonymous=True)
        
    # Parameters for CSV files and topics
    csv_files = ast.literal_eval(rospy.get_param('~csv_files'))  # List of CSV file paths
    if not csv_files:
        rospy.logerr("No CSV files provided. Please set the '~csv_files' parameter.")
        rospy.signal_shutdown("No CSV files provided.")
    dataframes = [load_csv(file) for file in csv_files]
    while not rospy.is_shutdown():
        plot()
    rospy.spin()

if __name__ == '__main__':
    main()
