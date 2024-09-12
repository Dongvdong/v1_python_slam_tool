import numpy as np
import math
#pip install geographiclib
from geographiclib.geodesic import Geodesic

# Constants
WGS84_A = 6378137.0
WGS84_B = 6356752.314245
WGS84_E = 0.0818191908

# Helper functions


def D2R(deg):
    return deg * np.pi / 180.0


def Square(x):
    return x * x


def RAD2DEG(rad):
    return rad * 180.0 / np.pi

# Function to convert ECEF to LLA


def ECEF2LLA(x, y, z):
    b = np.sqrt(WGS84_A * WGS84_A * (1 - WGS84_E * WGS84_E))
    ep = np.sqrt((WGS84_A * WGS84_A - b * b) / (b * b))
    p = np.hypot(x, y)
    th = np.arctan2(WGS84_A * z, b * p)
    lon = np.arctan2(y, x)
    lat = np.arctan2((z + ep * ep * b * np.power(np.sin(th), 3)),
                     (p - WGS84_E * WGS84_E * WGS84_A * np.power(np.cos(th), 3)))
    N = WGS84_A / np.sqrt(1 - WGS84_E * WGS84_E * np.sin(lat) * np.sin(lat))
    alt = p / np.cos(lat) - N

    return [RAD2DEG(lat), RAD2DEG(lon), alt]

# Function to convert LLA to ECEF


def LLA2ECEF(lat, lon, alt):
    clat = np.cos(D2R(lat))
    slat = np.sin(D2R(lat))
    clon = np.cos(D2R(lon))
    slon = np.sin(D2R(lon))

    a2 = Square(WGS84_A)
    b2 = Square(WGS84_B)

    L = 1.0 / np.sqrt(a2 * Square(clat) + b2 * Square(slat))
    x = (a2 * L + alt) * clat * clon
    y = (a2 * L + alt) * clat * slon
    z = (b2 * L + alt) * slat

    return [x, y, z]

#在经纬度变化很小的情况下，可以假设地球是平的。
# 然而，当经纬度变化较大时，这种假设可能导致较大的误差。对于大范围的转换，地球的曲率和椭球形状都会影响计算精度。
def GPS2NED(init_lat, init_lon, init_h, t_lat, t_lon, t_h):
    # EARTH_RADIUS = 6371000
    EARTH_RADIUS = 6378137 #WGS84 椭球体的赤道半径
    delta_lon = t_lon - init_lon
    delta_lat = t_lat - init_lat
    x_in_NED = EARTH_RADIUS * math.radians(delta_lat)
    y_in_NED = EARTH_RADIUS * \
        math.cos(math.radians(t_lat)) * math.radians(delta_lon)
    z_in_NED = init_h - t_h
    return x_in_NED, y_in_NED, z_in_NED


def NED2GPS(init_lat, init_lon, init_h, x_in_NED, y_in_NED, z_in_NED):
    # EARTH_RADIUS = 6371000
    EARTH_RADIUS = 6378137
    y_rad = x_in_NED / EARTH_RADIUS
    t_lat = (init_lat + math.degrees(y_rad))
    x_rad = y_in_NED / EARTH_RADIUS / math.cos(math.radians(t_lat))
    t_lon = init_lon + math.degrees(x_rad)
    t_h = init_h - z_in_NED
    return t_lat, t_lon, t_h


def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
    """
    Convert geodetic coordinates to ENU coordinates.
    """
    # WGS 84
    geod = Geodesic.WGS84

    # Calculate azimuth, back azimuth, and distance from the reference point
    g = geod.Inverse(lat_ref, lon_ref, lat, lon)

    # Distance from reference point (meters)
    dist = g['s12']

    # Azimuth from reference point to current point (degrees)
    azi = g['azi1']

    # Elevation angle (rad)
    elev = np.arctan2(h - h_ref, dist)

    # Distance from reference point projected to ground (meters)
    proj_dist = dist * np.cos(elev)

    # Calculate ENU coordinates
    e = proj_dist * np.sin(np.radians(azi))
    n = proj_dist * np.cos(np.radians(azi))
    u = h - h_ref

    return e, n, u

# 地球模型 - WGS84 椭球体 https://zhuanlan.zhihu.com/p/366781817
# O_lat, O_lon, O_alt 原点的经纬度和海拔
# ?????貌似不是NED,也不是ENU


def GPS2NED2(M_lat, M_lon, M_alt, O_lat, O_lon, O_alt):
    Ea = 6378137   # 赤道半径
    Eb = 6356725   # 极半径
    M_lat = math.radians(M_lat)
    M_lon = math.radians(M_lon)
    O_lat = math.radians(O_lat)
    O_lon = math.radians(O_lon)
    Ec = Ea*(1-(Ea-Eb)/Ea*((math.sin(M_lat))**2)) + M_alt
    Ed = Ec * math.cos(M_lat)
    d_lat = M_lat - O_lat
    d_lon = M_lon - O_lon
    x = d_lat * Ec
    y = d_lon * Ed
    z = M_alt - O_alt
    return x, y, z


def ecef_to_enu(ecef_coords, ref_ecef_coords):
    """
    Convert ECEF (Earth-Centered, Earth-Fixed) coordinates to ENU (East-North-Up) coordinates.

    Args:
    - ecef_coords: The coordinates in ECEF (x, y, z) format.
    - ref_ecef_coords: The reference ECEF coordinates (origin) in (x, y, z) format.

    Returns:
    - enu_coords: The converted ENU coordinates in (E, N, U) format.
    """
    # Extract ECEF coordinates
    x, y, z = ecef_coords[0], ecef_coords[1], ecef_coords[2]

    # Extract reference ECEF coordinates
    ref_x, ref_y, ref_z = ref_ecef_coords[0], ref_ecef_coords[1], ref_ecef_coords[2]

    # Calculate the differences between ECEF and reference ECEF coordinates
    dx = x - ref_x
    dy = y - ref_y
    dz = z - ref_z

    # Conversion matrix from ECEF to ENU
    rotation_matrix = np.array([
        [-np.sin(np.radians(ref_x)), np.cos(np.radians(ref_x)), 0],
        [-np.sin(np.radians(ref_y)) * np.cos(np.radians(ref_x)), -np.sin(np.radians(ref_y))
         * np.sin(np.radians(ref_x)), np.cos(np.radians(ref_y))],
        [np.cos(np.radians(ref_y)) * np.cos(np.radians(ref_x)), np.cos(np.radians(ref_y))
         * np.sin(np.radians(ref_x)), np.sin(np.radians(ref_y))]
    ])

    # Calculate ENU coordinates
    enu_coords = np.dot(rotation_matrix, np.array([dx, dy, dz]))

    return enu_coords
