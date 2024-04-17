import pandas as pd
import numpy as np
from io import StringIO
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LinearRegression

np.random.seed(0)

speed = 400
tau = 20
max_speed_in_tau = tau * speed / 1000

file_name = f"../workspace/data/elevation_motor/u{tau}/data{speed}0000.csv"
# the following targets are provided while generating the data
targets = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 200, 300, 400, 500, 1000]


def upward_curve(Xt: np.array, yt: np.array) -> "tuple[float, float, float]":
    """Curve fit on upward line in speed curve

    Args:
        Xt (np.array): time
        yt (np.array): smoother speed (x2 - x0)/2

    Returns:
        m (float): slope
        b (float): intercept
        total_score (float): total score
    """
    consider_till = yt.argmax() + 1
    X1 = np.array(Xt[:consider_till]).reshape(-1, 1)
    y1 = np.array(yt[:consider_till]).reshape(-1, 1)
    regr = LinearRegression()
    regr.fit(X1, y1)
    total_score = regr.score(X1, y1)
    return regr.coef_[0], regr.intercept_[0], total_score


def downward_curve(Xt: np.array, yt: np.array) -> "tuple[float, float, float]":
    """Curve fit on downward line in speed curve

    Args:
        Xt (np.array): time
        yt (np.array): smoother speed (x2 - x0)/2

    Returns:
        m (float): slope
        b (float): intercept
        total_score (float): total score
    """
    consider_from = len(yt) - yt[::-1].argmax() - 1
    X2 = np.array(Xt[consider_from:]).reshape(-1, 1)
    y2 = np.array(yt[consider_from:]).reshape(-1, 1)
    regr = LinearRegression()
    regr.fit(X2, y2)
    total_score = regr.score(X2, y2)
    return regr.coef_[0], regr.intercept_[0], total_score


def given_time_curve_fit(Xt: np.array, yt: np.array, m1, m2):
    global tau
    global max_speed_in_tau
    consider_till = yt.argmax() + 1
    consider_from = len(yt) - yt[::-1].argmax() - 1
    X1 = np.array(Xt[:consider_till]).reshape(-1, 1)
    y1 = np.array(yt[:consider_till]).reshape(-1, 1)
    X2 = np.array(Xt[consider_from:]).reshape(-1, 1)
    y2 = np.array(yt[consider_from:]).reshape(-1, 1)
    b2 = -m2 * (X2[-1] + tau)
    slope_count_pred = (np.round(m1 * X1 * 2) / 2).sum() + (
        np.round((m2 * X2 + b2) * 2) / 2
    ).sum()
    slope_count_real = y1.sum() + y2.sum()
    target = yt.sum()
    flat_count = target - slope_count_pred
    flat_time = 0
    if flat_count > 0:
        flat_time = np.round(flat_count / max_speed_in_tau)
    total_pred_count = flat_time * max_speed_in_tau + slope_count_pred
    return target, total_pred_count


def make_tau_multiple(time):
    global tau
    return np.round(time / tau) * tau


def position_to_smoother_derivative(positions: np.array):
    speeds = [y - x for x, y in zip(positions, positions[1:])]
    speeds = [(x + y) / 2 for x, y in zip(speeds + [0], [0] + speeds)]
    return speeds


def smoother_derivative_to_position(speeds: np.array, start_position: int):
    n = len(speeds)
    transform = -0.5 * np.roll(np.eye(n), 2).T + 0.5 * np.eye(n)
    transform[0][0] = 1
    transform[0][-1] = 0
    inv_transform = np.linalg.inv(transform)
    speeds = np.concatenate([[start_position], speeds[:-1]])
    return inv_transform @ speeds


def curve_fit(m1, m2, start_speed, target, end_time):
    """Get the curve that fits target, while meeting the time
    restriction, given the initial speed and the time after which the target needs to be reached

    Args:
        m1 (float): upward slope
        m2 (float): downward slope
        start_speed (float): this speed in current setup must be two step difference divided by 2
        target (int): number of steps to cover
        end_time (int): number of milliseconds
    """
    global max_speed_in_tau
    start_time = make_tau_multiple(start_speed / m1)
    saturate_time = make_tau_multiple((max_speed_in_tau + 0.5) / m1)
    end_time += start_time
    if end_time < saturate_time:
        pass
    else:
        pass


if __name__ == "__main__":
    with open(file_name, "r") as f:
        data = f.read()

    split_data = data.split("\n\n")[:-1]

    dfs = []
    for i, split in enumerate(split_data):
        df = pd.read_csv(StringIO(split), sep=",", header=None)
        times = df[0].to_numpy()
        locations = df[1].to_numpy()
        speeds = position_to_smoother_derivative(locations)
        df[2] = speeds
        accelerations = position_to_smoother_derivative(speeds)
        df[3] = accelerations
        df[0] = df[0] - df[0][0]
        dfs.append(df)
    m1 = []
    m2 = []
    total_scores1 = []
    total_scores2 = []
    for df in dfs:
        m, b, total_score = upward_curve(df[0], df[2])
        m1.append(m[0])
        total_scores1.append(total_score)
        m, b, total_score = downward_curve(df[0], df[2])
        m2.append(m[0])
        total_scores2.append(total_score)
    df = pd.DataFrame(
        {
            "target": targets,
            "m1": m1,
            "m2": m2,
            "total1": total_scores1,
            "total2": total_scores2,
        }
    )
    print(df)
    print(df["m1"].mean(), df["m2"].mean())
