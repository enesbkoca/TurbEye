
g = 9.80665  # gravitational acceleration

components = {
    "Fuelcell": {
        "name": "A-2000",
        "mass": 3.0,
        "power": 0
    },
    "Brackets": {
        "name": "Brackets of the fuel cell",
        "mass": 0.080,
        "power": 0
    },
    "Pressure regulator": {
        "name": "Ultralight H2 Gas Pressure Regulator",
        "mass": 0.200,
        "power": 0
    },
    "DC converter": {
        "name": "XLSemi XL4015",
        "mass": 0.020,
        "power": 0
    },
    "Backup battery": {
        "name": "Voltaplex LiPo 1800 mAh x10",
        "mass": 0.470,
        "power": 0
    },
    "Battery enclosing": {
        "name": "Enclosing of the backup battery",
        "mass": 0.100,
        "power": 0
    },
    "LIDAR": {
        "name": "Livox Mid-360",
        "mass": 0.265,
        "power": 6.5
    },
    "Barometer": {
        "name": "BMP390",
        "mass": 0,
        "power": 11.52E-6
    },
    "IMU": {
        "name": "MICROSTRAIN 3DM-CV7-AHRS",
        "mass": 0.0083,
        "power": 280E-3
    },
    "Flight Computer": {
        "name": "NUCLEO-H7A3ZI-Q",
        "mass": 0.117,
        "power": 0.620 * 3.6
    },
    "GPS": {
        "name": "HGLRC M100 MINI GPS",
        "mass": 0.0027 * 2,
        "power": 0.2 * 2
    },
    "Transmitter": {
        "name": "SIYI FM30",
        "mass": 0.036,
        "power": 0.7
    },
    "Receiver": {
        "name": "SIYI FR Mini Receiver",
        "mass": 0.015,
        "power": 0.7
    },
    "3D Scanner": {
        "name": "MotionCam-3D Color L+",
        "mass": 1.150,
        "power": 60
    },
    "Camera": {
        "name": "DJI Zenmuse H20T",
        "mass": 0.828,
        "power": 27
    },
    "SD Card": {
        "name": "Sony SF-G Series TOUGH UHS-II",
        "mass": 0,
        "power": 0
    },
    "Cables": {
        "name": "Custom Cable",
        "mass": 0.0493,
        "power": 0
    },
    "Chassis": {
        "name": "Custom Chassis",
        "mass": 1.5,
        "power": 10
    }
}

mass_components = 0
power_components = 0

for component, parameters in components.items():
    mass_components += parameters["mass"]
    power_components += parameters["power"]

I_components = power_components / 50

if __name__ == "__main__":
    print("Mass of components: ", mass_components)
    print("Power of components: ", power_components)
    print("Current draw of components: ", I_components)
