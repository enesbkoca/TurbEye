import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def get_data(input_file):
    # get data from file
    dataset = pd.read_csv(input_file)
    data = pd.DataFrame(
        dataset[["Rotation speed (rpm)", "Thrust (kgf)", "Torque (N⋅m)"]]
    )

    # convert to SI
    temp = data["Thrust (kgf)"].multiply(9.80665)
    data["Thrust (kgf)"] = temp
    temp = data["Rotation speed (rpm)"].mul(0.1047198)
    data["Rotation speed (rpm)"] = temp

    # rename columns and return data
    data = data.rename(
        columns={
            "Rotation speed (rpm)": "rpm",
            "Thrust (kgf)": "thrust",
            "Torque (N⋅m)": "torque",
        }
    )
    return data


def get_thrust_coeffs(data: pd.DataFrame):
    rpm = data["rpm"].to_numpy()[1::]
    thrust = data["thrust"].to_numpy()[1::]
    coeffs = np.polyfit(x=rpm, y=thrust, deg=2)
    return coeffs


def get_torque_coeffs(data: pd.DataFrame):
    rpm = data["rpm"].to_numpy()[1::]
    thrust = data["torque"].to_numpy()[1::]
    coeffs = np.polyfit(x=rpm, y=thrust, deg=2)
    return coeffs


def plot_estimation(coeffs):
    x = np.arange(600)
    y = x**2 * coeffs[0] + x * coeffs[1] + coeffs[2]
    plt.plot(x, y)
    plt.show()


input_file = "test_data/T-Motor Antigravity 6007II-160 x T-Motor P26 8S - Rick.csv"
data = get_data(input_file)

coeffs = get_torque_coeffs(data)
print(coeffs)
