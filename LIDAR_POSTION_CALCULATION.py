""" Explanation of Code

Calculatiang the Lidar postion that is at 1m distance from the ground point attacahed to the geo stick.

Explanation
Vertical Distance:

The vertical component of the LiDAR height when the stick is tilted at an angle (theta).
Calculated as vertical_distance = h_lidar * cos(theta_rad).
Horizontal Distance:

The horizontal component of the LiDAR height due to the tilt.
Calculated as horizontal_distance = h_lidar * sin(theta_rad).
Azimuth (phi):

Determines the direction of the horizontal displacement around the ground point.
Used to compute the new latitude and longitude based on the horizontal distance.

********************************************************************************

Direct Method:

Adds the LiDAR height directly to the geodetic height (h_ground + h_lidar).
This method does not account for any horizontal displacement due to tilt and azimuth.
Offset Method:

Converts tilt and azimuth to radians.
Calculates the horizontal and vertical distances based on the tilt angle.
Uses geod.fwd to compute the new latitude and longitude based on the horizontal distance and azimuth.
Adjusts the height based on the vertical distance.
Converts the new geodetic coordinates to ECEF for verification.
Visualizing with a Tilt and Azimuth Example
If theta = 10 degrees and phi = 45 degrees:

********************************************************************************
Horizontal Distance: 1 meter * sin(10 degrees) ≈ 0.1736 meters
Vertical Distance: 1 meter * cos(10 degrees) ≈ 0.9848 meters
The LiDAR moves 0.1736 meters in the direction of 45 degrees azimuth (northeast direction) from the ground point and vertically 0.9848 meters.
By using the offset method, we accurately account for the horizontal displacement caused by the tilt and azimuth,
resulting in different latitude, longitude, and height values compared to the direct method. 
This ensures the LiDAR position is calculated correctly for any tilt and azimuth.
************************************************************************************ """






from pyproj import Transformer, Geod
import math

# Define WGS84 ellipsoid and ECEF using Transformer
transformer_wgs84_to_ecef = Transformer.from_crs("epsg:4326", "epsg:4978", always_xy=True)
transformer_ecef_to_wgs84 = Transformer.from_crs("epsg:4978", "epsg:4326", always_xy=True)
geod = Geod(ellps="WGS84")

# Ground point in latitude, longitude, height
lat_ground = 40.0
lon_ground = -105.0
h_ground = 300

# Convert ground point to ECEF
X_ground, Y_ground, Z_ground = transformer_wgs84_to_ecef.transform(lon_ground, lat_ground, h_ground)
print(f"Ground ECEF: X = {X_ground}, Y = {Y_ground}, Z = {Z_ground}")

# Height of LiDAR and tilt, azimuth in degrees
h_lidar = 1
theta = 10  # tilt angle in degrees
phi = 45    # azimuth angle in degrees

# Method 1: Direct Height Addition in Geodetic System (only works for theta = 0 and phi = 0)
h_new = h_ground + h_lidar

# Convert the new geodetic coordinates to ECEF
X_lidar_direct, Y_lidar_direct, Z_lidar_direct = transformer_wgs84_to_ecef.transform(lon_ground, lat_ground, h_new)
# Convert back to lat, lon, height for verification
lon_lidar_direct, lat_lidar_direct, h_lidar_direct = transformer_ecef_to_wgs84.transform(X_lidar_direct, Y_lidar_direct, Z_lidar_direct)
print(f"Direct Method - LiDAR Position: Latitude = {lat_lidar_direct}, Longitude = {lon_lidar_direct}, Height = {h_lidar_direct}")

# Method 2: Applying Tilt and Azimuth
# Convert tilt and azimuth to radians
theta_rad = math.radians(theta)
phi_rad = math.radians(phi)

# Calculate the horizontal distance from the tilt
horizontal_distance = h_lidar * math.sin(theta_rad)
vertical_distance = h_lidar * math.cos(theta_rad)

# Calculate new lat, lon from the ground point with tilt and azimuth
lon_lidar_offset, lat_lidar_offset, back_azimuth = geod.fwd(lon_ground, lat_ground, phi, horizontal_distance)
h_lidar_offset = h_ground + vertical_distance

# Convert the new geodetic coordinates to ECEF for verification
X_lidar_offset, Y_lidar_offset, Z_lidar_offset = transformer_wgs84_to_ecef.transform(lon_lidar_offset, lat_lidar_offset, h_lidar_offset)
# Convert back to lat, lon, height for verification
lon_lidar_offset_ver, lat_lidar_offset_ver, h_lidar_offset_ver = transformer_ecef_to_wgs84.transform(X_lidar_offset, Y_lidar_offset, Z_lidar_offset)
print(f"Offset Method - LiDAR Position: Latitude = {lat_lidar_offset_ver}, Longitude = {lon_lidar_offset_ver}, Height = {h_lidar_offset_ver}")

# Additional prints to compare intermediate steps
print(f"Direct Method ECEF: X = {X_lidar_direct}, Y = {Y_lidar_direct}, Z = {Z_lidar_direct}")
print(f"Offset Method ECEF: X = {X_lidar_offset}, Y = {Y_lidar_offset}, Z = {Z_lidar_offset}")
print(f"Direct Method Geodetic: Latitude = {lat_lidar_direct}, Longitude = {lon_lidar_direct}, Height = {h_lidar_direct}")
print(f"Offset Method Geodetic: Latitude = {lat_lidar_offset_ver}, Longitude = {lon_lidar_offset_ver}, Height = {h_lidar_offset_ver}")
