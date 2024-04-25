from flask import Flask, request, jsonify, send_from_directory
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

app = Flask(__name__)
PLOTS_DIR = 'plots'
os.makedirs(PLOTS_DIR, exist_ok=True)

lidar_data_store = []

def process_lidar_data(data):
    # Convert the list of dictionaries to two lists of angles and distances
    angles_degrees = [d['angle'] for d in data['angles']]
    distances_mm = [d['distance'] for d in data['angles']]

    # Convert angles from degrees to radians
    angles_radians = np.radians(angles_degrees)

    # Convert to Cartesian coordinates
    x_coords = [distance * np.sin(angle) for distance, angle in zip(distances_mm, angles_radians)]
    y_coords = [distance * -np.cos(angle) for distance, angle in zip(distances_mm, angles_radians)]

    # Find the min y-coordinate to determine the ground level
    y_surface_level = min(y_coords)
    
    # Calculate the depth of each point from the surface level
    depths = [y - y_surface_level for y in y_coords]

    # Generate and save the plot
    save_plot(x_coords, y_coords, depths, angles_degrees, data['finalDepth'])

def save_plot(x_coords, y_coords, depths, angles_degrees, final_depth):
    plt.figure(figsize=(12, 8))
    plt.scatter(x_coords, y_coords, label='LiDAR Points')
    plt.plot(x_coords, y_coords, 'b-', label='Ditch Profile')
    plt.scatter(0, 0, c='green', label='LiDAR Position')  # LIDAR at origin
    plt.title(f'LiDAR Data Visualization - Final Depth: {final_depth:.2f} mm')
    plt.xlabel('X Coordinate (mm)')
    plt.ylabel('Y Coordinate (mm)')
    plt.grid(True)
    plt.legend()
    plt.gca().invert_yaxis()  # Invert y-axis to show depth correctly

    # Annotate specific angles with depths
    selected_angles = list(range(270, 361, 10)) + list(range(0, 91, 10))
    for angle_deg in selected_angles:
        if angle_deg in angles_degrees:
            idx = angles_degrees.index(angle_deg)
            plt.annotate(f'{angle_deg}°\nDepth: {depths[idx]:.2f}mm',
                         (x_coords[idx], y_coords[idx]),
                         textcoords="offset points", xytext=(10, 10), ha='center')

    # Save the plot with a timestamp in the filename
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    filename = f'plot_{timestamp}.png'
    filepath = os.path.join(PLOTS_DIR, filename)
    plt.savefig(filepath)
    plt.close()

    # Save the path of the plot for later use (e.g., serving it via HTTP)
    lidar_data_store.append(filepath)

@app.route('/receive_data', methods=['POST'])
def receive_data():
    data = request.get_json()
    print('Data received:', data)
    if data and 'angles' in data:
        process_lidar_data(data)
        return jsonify({'message': 'Data received and processed'}), 200
    else:
        return jsonify({'message': 'Invalid data format'}), 400

@app.route('/plots/<filename>')
def get_plot(filename):
    return send_from_directory(PLOTS_DIR, filename)

@app.route('/plots')
def list_plots():
    plots = os.listdir(PLOTS_DIR)
    return jsonify(plots)

@app.route('/latest_plot', methods=['GET'])
def get_latest_plot():
    if lidar_data_store:
        latest_plot_path = lidar_data_store[-1]
        return send_from_directory(*os.path.split(latest_plot_path))
    else:
        return jsonify({'message': 'No plots available'}), 404

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=3000)


