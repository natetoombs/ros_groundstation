import numpy as np

def ecef2lla(ecef):
    A = 6378137.0
    B = 6356752.314245
    F = (A - B) / A
    E2 = F * (2 - F)
    
    ecef_x = ecef[0]
    ecef_y = ecef[1]
    ecef_z = ecef[2]

    r2 = ecef_x**2 + ecef_y**2
    z = ecef_z

    while True:
        zk = z
        sinp = z / np.sqrt(r2 + z**2)
        v = A / np.sqrt(1 - E2*sinp**2)
        z = ecef_z + v*E2*sinp
        if not np.abs(z - zk) >= 1e-4:
            break

    lla_x = np.arctan(z / np.sqrt(r2)) if r2 > 1e-12 else (np.pi/2 if ecef_z > 0.0 else -np.pi/2)
    lla_y = np.arctan2(ecef_y, ecef_x) if r2 > 1e-12 else 0.0
    lla_z = np.sqrt(r2 + z**2) - v

    rad2deg = 180/np.pi

    lla = [lla_x*rad2deg, lla_y*rad2deg, lla_z*rad2deg]
    print(lla)
    return lla