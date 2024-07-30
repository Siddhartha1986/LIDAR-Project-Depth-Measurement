"""
- **Setup:**
  - Initialize Flask application.
  - Create a directory for storing plots.

- **Data Processing Function:**
  - `process_lidar_data`: Processes received LiDAR data.
    - Converts polar coordinates to Cartesian coordinates.
    - Rotates coordinates based on tilt angle.
    - Filters points within specified angle ranges.
    - Calculates depths and filters points above a threshold.
    - Finds the lowest point and adjusts depths.
    - Collects points around the lowest point for circle fitting.
    - Fits a circle to these points and evaluates the fit quality.
    - Saves the processed plot.

- **Plot Saving Function:**
  - `save_plot`: Generates and saves a plot of the processed LiDAR data.

- **API Endpoints:**
  - `/receive_data`: Receives and processes LiDAR data (POST).
  - `/plots/<filename>`: Serves a specific plot file.
  - `/plots`: Lists all available plot files.
  - `/latest_plot`: Serves the latest plot file.

- **Server Execution:**
  - Runs the Flask application on port 3000.
"""



from flask import Flask, request, jsonify, send_from_directory
import numpy as np
import plotly.graph_objects as go
import os
from datetime import datetime

app = Flask(__name__)
PLOTS_DIR = 'plots'
os.makedirs(PLOTS_DIR, exist_ok=True)

lidar_data_store = []

def process_lidar_data(data):
    angles_degrees = [d['angle'] for d in data['angles'] if d['distance'] > 0]
    distances_mm = [d['distance'] for d in data['angles'] if d['distance'] > 0]
    tilt = data['tiltAngle']

    angles_radians = np.radians(angles_degrees)
    x_coords = [distance * np.sin(angle) for distance, angle in zip(distances_mm, angles_radians)]
    y_coords = [distance * np.cos(angle) for distance, angle in zip(distances_mm, angles_radians)]

    # Rotate the coordinates by the tilt angle
    rotation_angle = np.radians(tilt)
    cos_angle = np.cos(rotation_angle)
    sin_angle = np.sin(rotation_angle)
    x_coords_rotated = [x * cos_angle - y * sin_angle for x, y in zip(x_coords, y_coords)]
    y_coords_rotated = [x * sin_angle + y * cos_angle for x, y in zip(x_coords, y_coords)]

    y_surface_level_rotated = max(y_coords_rotated)
    depths_rotated = [y_surface_level_rotated - y for y in y_coords_rotated]

    # Define angle ranges with tilt angle adjustment and wrapping logic
    startAngleRange1 = (290 + tilt) % 360
    endAngleRange1 = (359 + tilt) % 360
    startAngleRange2 = (0 + tilt) % 360
    endAngleRange2 = (70 + tilt) % 360

    # Ensure angles are within 0-360 degrees
    if startAngleRange1 < 0: startAngleRange1 += 360
    if endAngleRange1 < 0: endAngleRange1 += 360
    if startAngleRange2 < 0: startAngleRange2 += 360
    if endAngleRange2 < 0: endAngleRange2 += 360

    # Filter points within the specified angle ranges
    filtered_x_coords_rotated = []
    filtered_y_coords_rotated = []
    filtered_depths_rotated = []
    filtered_angles_degrees = []

    for angle, x, y, depth in zip(angles_degrees, x_coords_rotated, y_coords_rotated, depths_rotated):
        if (startAngleRange1 <= endAngleRange1 and startAngleRange1 <= angle <= endAngleRange1) or \
           (startAngleRange1 > endAngleRange1 and (angle >= startAngleRange1 or angle <= endAngleRange1)) or \
           (startAngleRange2 <= endAngleRange2 and startAngleRange2 <= angle <= endAngleRange2) or \
           (startAngleRange2 > endAngleRange2 and (angle >= startAngleRange2 or angle <= endAngleRange2)):
            filtered_x_coords_rotated.append(x)
            filtered_y_coords_rotated.append(y)
            filtered_depths_rotated.append(depth)
            filtered_angles_degrees.append(angle)

    # Collect depths for specified angle ranges
    depths_range1 = [depth for angle, depth in zip(filtered_angles_degrees, filtered_depths_rotated) if \
                     (startAngleRange1 <= angle <= endAngleRange1) or \
                     (startAngleRange1 > endAngleRange1 and (angle >= startAngleRange1 or angle <= endAngleRange1))]
    depths_range2 = [depth for angle, depth in zip(filtered_angles_degrees, filtered_depths_rotated) if \
                     (startAngleRange2 <= angle <= endAngleRange2) or \
                     (startAngleRange2 > endAngleRange2 and (angle >= startAngleRange2 or angle <= endAngleRange2))]

    # Sort the depths and take the top 10 highest values
    depths_range1.sort(reverse=True)
    depths_range2.sort(reverse=True)

    top_10_depths_range1 = depths_range1[:10]
    top_10_depths_range2 = depths_range2[:10]

    # Take the lowest value out of those 10 values on both sides
    min_top_10_depth_range1 = min(top_10_depths_range1) if top_10_depths_range1 else float('inf')
    min_top_10_depth_range2 = min(top_10_depths_range2) if top_10_depths_range2 else float('inf')

    # Determine the final depth value
    final_depth = min(min_top_10_depth_range1, min_top_10_depth_range2)

    # Calculate the threshold as 5% of the final depth value
    threshold_value = 0.05 * final_depth

    # Filter points above the threshold value
    threshold_indices = [i for i, y in enumerate(y_coords_rotated) if y >= y_surface_level_rotated - threshold_value]
    threshold_angles = [angles_degrees[i] for i in threshold_indices]

    # Check if there are points above the threshold
    if not threshold_indices:
        print("No points above the threshold value.")
        new_final_depth = final_depth
        detection_status = "No Pipe Detected"
        xc, yc, R, max_residual, mean_residual, radius_limit, diameter_limit = 0, 0, 0, 0, 0, 0, 0
        is_good_fit = False
    else:
        # Determine the angle range for the points above the threshold
        threshold_angles.sort()
        if threshold_angles[-1] - threshold_angles[0] <= 180:
            angle_range_start = threshold_angles[0]
            angle_range_end = threshold_angles[-1]
        else:
            for i in range(1, len(threshold_angles)):
                if threshold_angles[i] - threshold_angles[i-1] > 180:
                    angle_range_start = threshold_angles[i]
                    angle_range_end = threshold_angles[i-1] + 360
                    break

        if angle_range_end < angle_range_start:
            angle_range_end += 360

        # Collect all points within this new angle range
        filtered_points = [(x, y, d, a) for x, y, d, a in zip(x_coords_rotated, y_coords_rotated, depths_rotated, angles_degrees)
                           if angle_range_start <= a <= angle_range_end or (angle_range_start <= a + 360 <= angle_range_end) or (angle_range_start <= a - 360 <= angle_range_end)]

        # Ensure the wrap-around angle logic is correct
        adjusted_angles = [a if angle_range_start <= a <= angle_range_end else (a + 360 if a + 360 <= angle_range_end else a - 360) for _, _, _, a in filtered_points]
        filtered_points = [(x, y, d, a) for (x, y, d, a), adj_a in zip(filtered_points, adjusted_angles) if angle_range_start <= adj_a <= angle_range_end]

        # Sort the filtered points by their y-coordinate in ascending order
        filtered_points_sorted_by_y = sorted(filtered_points, key=lambda p: p[1])

        # Initialize lowest_point as None
        lowest_point = None

        # Find the lowest y-coordinate point that is also less than the final depth
        for point in filtered_points_sorted_by_y:
            if point[2] < final_depth:
                lowest_point = point
                break

        # Ensure we found a valid point
        if lowest_point:
            lowest_angle_index = angles_degrees.index(lowest_point[3])

            # Adjust depths relative to the new lowest point
            new_depths = [(lowest_point[1] - y) for y in y_coords_rotated]

            # Filter points for plotting within the specified angle ranges
            filtered_new_depths = [(lowest_point[1] - y) for x, y in zip(filtered_x_coords_rotated, filtered_y_coords_rotated)]
            filtered_hover_texts = [f'Angle: {angle}°<br>X: {x:.2f}mm<br>Y: {y:.2f}mm<br>Initial Depth: {depth:.2f}mm<br>New Depth: {new_depth:.2f}mm'
                                    for angle, x, y, depth, new_depth in zip(filtered_angles_degrees, filtered_x_coords_rotated, filtered_y_coords_rotated, filtered_depths_rotated, filtered_new_depths)]

            # Calculate the new R limit based on your logic
            pipe = max(y_coords_rotated) - lowest_point[1]
            diameter_limit = pipe + 0.005 * pipe
            radius_limit = diameter_limit / 2

            # Define the maximum allowable Euclidean distance gap to consider points as continuous
            MAX_EUCLIDEAN_DISTANCE_GAP = np.sqrt(2) * radius_limit

            # Ensure proper index wrapping around the list
            lowest_angle_index = angles_degrees.index(lowest_point[3])

            # Initialize variables for bump points collection
            bump_indices = []
            left_collected = 0
            right_collected = 0
            i = 1

            bump_indices.append(lowest_angle_index)

            # Function to calculate Euclidean distance between two points
            def euclidean_distance(x1, y1, x2, y2):
                return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            # Collect points to the left and right of the lowest point
            while left_collected < 5 or right_collected < 5:
                if left_collected < 5:
                    left_index = (lowest_angle_index - i + len(angles_degrees)) % len(angles_degrees)
                    if euclidean_distance(x_coords_rotated[lowest_angle_index], y_coords_rotated[lowest_angle_index],
                                          x_coords_rotated[left_index], y_coords_rotated[left_index]) <= MAX_EUCLIDEAN_DISTANCE_GAP:
                        bump_indices.append(left_index)
                        left_collected += 1
                    else:
                        left_collected = 5
                if right_collected < 5:
                    right_index = (lowest_angle_index + i) % len(angles_degrees)
                    if euclidean_distance(x_coords_rotated[lowest_angle_index], y_coords_rotated[lowest_angle_index],
                                          x_coords_rotated[right_index], y_coords_rotated[right_index]) <= MAX_EUCLIDEAN_DISTANCE_GAP:
                        bump_indices.append(right_index)
                        right_collected += 1
                    else:
                        right_collected = 5
                i += 1
                if i > len(angles_degrees):
                    break

            bump_points = [(x_coords_rotated[i], y_coords_rotated[i], depths_rotated[i], new_depths[i], angles_degrees[i]) for i in bump_indices]
            x_bump, y_bump = zip(*[(bp[0], bp[1]) for bp in bump_points])
            xc = data['xc']
            yc = data['yc']
            R = data['R']
            max_residual = data['max_residual']
            mean_residual = data['mean_residual']
            radius_limit = data['radius_limit']
            is_good_fit = data['is_good_fit']
            diameter_limit = data['diameter_limit']

            # Print residual values for verification
            print(f"Max Residual: {max_residual:.2f} mm")
            print(f"Mean Residual: {mean_residual:.2f} mm")

            # Verify if the mean residual is being calculated correctly
            calculated_residuals = [np.sqrt((x - xc)**2 + (y - yc)**2) - R for x, y in zip(x_bump, y_bump)]
            calculated_max_residual = np.max(np.abs(calculated_residuals))
            calculated_mean_residual = np.mean(np.abs(calculated_residuals))
            print(f"Calculated Max Residual: {calculated_max_residual:.2f} mm")
            print(f"Calculated Mean Residual: {calculated_mean_residual:.2f} mm")

            new_final_depth = abs(final_depth - (max(y_coords_rotated) - lowest_point[1]))
            detection_status = "Pipe Detected" if is_good_fit else "No Pipe Detected"

            if is_good_fit:
                pipe_diameter = 2 * R
                print(f"Server Final Depth: {final_depth:.2f} mm")
                print(f"Server New Final Depth: {new_final_depth:.2f} mm")
                print(f"Server Detection Status: {detection_status}")
                print(f"Server Pipe Diameter: {pipe_diameter:.2f} mm")
            else:
                print(f"Server Final Depth: {final_depth:.2f} mm")
                print(f"Server New Final Depth: {new_final_depth:.2f} mm")
                print(f"Server Detection Status: {detection_status}")

            # Call the save_plot function to create and save the plot
            save_plot(filtered_x_coords_rotated, filtered_y_coords_rotated, filtered_depths_rotated, filtered_angles_degrees, final_depth, tilt, data, new_final_depth, detection_status, is_good_fit, xc, yc, R, bump_points, filtered_new_depths, lowest_point, diameter_limit)
        else:
            print("No valid point found for the bump top.")
            new_final_depth = final_depth
            detection_status = "No Pipe Detected"
            xc, yc, R, max_residual, mean_residual, radius_limit, diameter_limit = 0, 0, 0, 0, 0, 0, 0
            is_good_fit = False
            print(f"Final depth value for the ditch is: {final_depth:.2f} mm")
            print(f"New final depth value: {new_final_depth:.2f} mm")

def save_plot(x_coords, y_coords, depths, angles_degrees, final_depth, tilt_angle, data, new_final_depth, detection_status, is_good_fit, xc, yc, R, bump_points, new_depths, lowest_point, diameter_limit):
    fig = go.Figure()

    # Create hover texts for the filtered points
    hover_texts = [f'Angle: {angle}°<br>X: {x:.2f}mm<br>Y: {y:.2f}mm<br>Initial Depth: {depth:.2f}mm<br>New Depth: {new_depth:.2f}mm' 
                   for angle, x, y, depth, new_depth in zip(angles_degrees, x_coords, y_coords, depths, new_depths)]
    fig.add_trace(go.Scatter(x=x_coords, y=y_coords, mode='markers', marker=dict(size=5, color=new_depths, colorscale='Viridis', colorbar=dict(title='New Depth (mm)')), text=hover_texts, hoverinfo='text', name='LiDAR Points'))

    # Add the LIDAR position to the plot
    fig.add_trace(go.Scatter(x=[0], y=[0], mode='markers', marker=dict(size=10, color='green'), name='LIDAR Position'))

    y_surface_level_rotated = max(y_coords)
    y_final_depth = y_surface_level_rotated - final_depth

    # Draw the initial trench bottom line
    fig.add_shape(type='line',
                  x0=min(x_coords), y0=y_surface_level_rotated,
                  x1=max(x_coords), y1=y_surface_level_rotated,
                  line=dict(color='red', width=2),
                  name='Initial Trench Bottom')
    fig.add_annotation(x=max(x_coords), y=y_surface_level_rotated, text="Initial Trench Bottom", showarrow=False, yshift=10, font=dict(size=12, color="red"))

    # Draw the new trench bottom line
    fig.add_shape(type='line',
                  x0=min(x_coords), y0=lowest_point[1],
                  x1=max(x_coords), y1=lowest_point[1],
                  line=dict(color='blue', width=2, dash='dash'),
                  name='New Trench Bottom')
    fig.add_annotation(x=max(x_coords), y=lowest_point[1], text="New Trench Bottom", showarrow=False, yshift=10, font=dict(size=12, color="blue"))

    # Draw the earth surface line
    fig.add_shape(type='line',
                  x0=min(x_coords), y0=y_final_depth,
                  x1=max(x_coords), y1=y_final_depth,
                  line=dict(color='green', width=2, dash='dash'),
                  name='Earth Surface')
    fig.add_annotation(x=max(x_coords), y=y_final_depth, text="Earth Surface", showarrow=False, yshift=10, font=dict(size=12, color="green"))

    # Add annotations for the ESP32 calculated depths and detection status
    fig.add_annotation(xref='paper', yref='paper', x=0, y=1.0, text=f"ESP32 Final Depth: {data['finalDepth']:.2f} mm", showarrow=False, font=dict(size=12, color="blue"))
    fig.add_annotation(xref='paper', yref='paper', x=0, y=0.95, text=f"ESP32 New Final Depth: {data['newFinalDepth']:.2f} mm", showarrow=False, font=dict(size=12, color="green"))
    fig.add_annotation(xref='paper', yref='paper', x=0, y=0.9, text=f"ESP32 Detection Status: {data['detectionStatus']}", showarrow=False, font=dict(size=12, color="purple"))

    # Add annotations for the server calculated depths and detection status
    fig.add_annotation(xref='paper', yref='paper', x=0.3, y=1.0, text=f"Server Final Depth: {final_depth:.2f} mm", showarrow=False, font=dict(size=12, color="red"))
    fig.add_annotation(xref='paper', yref='paper', x=0.3, y=0.95, text=f"Server New Final Depth: {new_final_depth:.2f} mm", showarrow=False, font=dict(size=12, color="red"))
    fig.add_annotation(xref='paper', yref='paper', x=0.3, y=0.9, text=f"Server Detection Status: {detection_status}", showarrow=False, font=dict(size=12, color="red"))

    # Plot the fitted circle if it's a good fit
    if is_good_fit:
        theta = np.linspace(0, 2 * np.pi, 100)
        x_circle = xc + R * np.cos(theta)
        y_circle = yc - R * np.sin(theta)
        fig.add_trace(go.Scatter(x=x_circle, y=y_circle, mode='lines', line=dict(color='red'), name='Fitted Circle'))

        pipe_diameter = 2 * R
        fig.add_annotation(xref='paper', yref='paper', x=0.6, y=1.0, text=f"Circle Diameter: {pipe_diameter:.2f} mm", showarrow=False, font=dict(size=12, color="orange"))
        # Annotate the diameter limit
        fig.add_annotation(xref='paper', yref='paper', x=0.6, y=0.95, text=f"Max Diameter of Pipe: {diameter_limit:.2f} mm", showarrow=False, font=dict(size=12, color="orange"))

    # Plot the bump points with updated hover text
    bump_hover_texts = [f'Angle: {bp[4]}°<br>X: {bp[0]:.2f}mm<br>Y: {bp[1]:.2f}mm<br>Depth: {bp[2]:.2f}mm<br>New Depth: {bp[3]:.2f}mm' for bp in bump_points]
    bump_points_np = np.array([(bp[0], bp[1]) for bp in bump_points])
    fig.add_trace(go.Scatter(x=bump_points_np[:, 0], y=bump_points_np[:, 1], mode='markers', marker=dict(size=5, color='red'), text=bump_hover_texts, hoverinfo='text', name='Bump Points'))

    # Update layout of the plot
    fig.update_layout(title=f'LiDAR Data Visualization with Circle Fitting | Final Depth: {final_depth:.2f} mm | New Final Depth: {new_final_depth:.2f} mm | Tilt: {tilt_angle}°',
                      xaxis_title='X Coordinate (mm)',
                      yaxis_title='Y Coordinate (mm)',
                      legend=dict(x=0.7, y=1.1, orientation='h'),
                      margin=dict(l=0, r=0, t=40, b=0),
                      yaxis=dict(autorange='reversed'))

    # Save the plot to an HTML file
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    filename = f'plot_{timestamp}.html'
    filepath = os.path.join(PLOTS_DIR, filename)
    fig.write_html(filepath)
    
    print(f'Saved plot to {filepath}')

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
    print(f'Serving plot file: {filename}')
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
    app.run(debug=True, host='0.0.0.0', port=3000, threaded=True)
