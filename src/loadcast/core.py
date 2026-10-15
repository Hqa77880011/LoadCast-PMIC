import numpy as np
from scipy.linalg import expm
from scipy.optimize import minimize


METHODS = ["gated", "pid", "static_ff", "no_gate", "observable_gate", "oracle_mpc"]


def discretize(vin, inductance, capacitance, period):
    matrix = np.array([[0.0, -1 / inductance], [1 / capacitance, 0.0]])
    inputs = np.array([[vin / inductance, 0.0], [0.0, -1 / capacitance]])
    augmented = np.zeros((4, 4))
    augmented[:2, :2] = matrix
    augmented[:2, 2:] = inputs
    discrete = expm(augmented * period)
    return discrete[:2, :2], discrete[:2, 2:]


def generate(rng, count):
    for index in range(count):
        load = []
        for _ in range(12):
            load.extend([rng.uniform(0.15, 3.2)] * rng.randint(20, 60))
        forecast = np.asarray(load) + np.asarray([rng.gauss(0, 0.15) for _ in load])
        if index % 3 == 0:
            forecast[len(load) // 3 : len(load) // 2] += 1.0
        yield {
            "id": str(index),
            "family": f"workload-{index}",
            "load": load,
            "forecast": forecast.tolist(),
            "noise": [rng.gauss(0, 0.001) for _ in load],
            "vin": 3.3,
            "reference": 0.9,
            "L": 0.47e-6,
            "C": 47e-6,
            "esr": 0.008,
            "frequency": 2e6,
            "duty_min": 0.05,
            "duty_max": 0.95,
            "kp": 0.5,
            "ki": 0.002,
            "kd": 0.1,
            "kff": 0.015,
            "sigma_max": 0.3,
            "settling_band": 0.005,
        }


def run_case(case, method):
    load = np.asarray(case["load"])
    forecast = np.asarray(case["forecast"])
    if len(load) != len(forecast) or len(load) != len(case["noise"]):
        raise ValueError("Load, forecast and noise traces must have equal length")
    period = 1 / case["frequency"]
    A, B = discretize(case["vin"], case["L"], case["C"], period)
    state = np.array([load[0], case["reference"]])
    voltages, currents, duties, gates = [], [], [], []
    integral, previous, previous_v = 0.0, 0.0, case["reference"]
    residuals = []
    for t in range(len(load)):
        voltage = state[1] + case["esr"] * (state[0] - load[t])
        sensed = voltage + case["noise"][t]
        error = case["reference"] - sensed
        integral_trial = integral + error
        feedback = (
            case["kp"] * error + case["ki"] * integral_trial + case["kd"] * (error - previous)
        )
        increment = forecast[min(t + 1, len(load) - 1)] - forecast[t]
        gate = 0.0 if method == "pid" else 1.0
        if method in ("gated", "observable_gate"):
            sigma = np.std(residuals[-16:]) if len(residuals) >= 2 else case["sigma_max"]
            gate = float(np.clip(1 - sigma / case["sigma_max"], 0, 1))
        correction = case["kff"] * increment
        if method == "static_ff":
            correction = case["kff"] * (forecast[t] - load[0])
        raw = case["reference"] / case["vin"] + feedback + gate * correction
        duty = float(np.clip(raw, case["duty_min"], case["duty_max"]))
        if (
            duty == raw
            or (raw > case["duty_max"] and error < 0)
            or (raw < case["duty_min"] and error > 0)
        ):
            integral = integral_trial
        if method == "oracle_mpc":
            future = load[t : min(t + 4, len(load))]

            def objective(commands):
                z = state.copy()
                cost = 0.0
                for command, demand in zip(commands, future):
                    z = A @ z + B @ [command, demand]
                    cost += 1e3 * (z[1] - case["reference"]) ** 2 + 0.01 * (command - duty) ** 2
                return cost

            solution = minimize(
                objective,
                np.full(len(future), duty),
                bounds=[(case["duty_min"], case["duty_max"])] * len(future),
                method="L-BFGS-B",
            )
            if not solution.success:
                raise RuntimeError("Oracle MPC did not converge")
            duty = float(solution.x[0])
        voltages.append(float(voltage))
        currents.append(float(state[0]))
        duties.append(duty)
        gates.append(gate)
        observed_load = state[0] - case["C"] * (sensed - previous_v) / period
        residuals.append((observed_load if method == "observable_gate" else load[t]) - forecast[t])
        state = A @ state + B @ [duty, load[t]]
        previous, previous_v = error, sensed
    return {"voltage": voltages, "current": currents, "duty": duties, "gate": gates}


def score(case, result):
    voltage, load, current = (
        np.asarray(result["voltage"]),
        np.asarray(case["load"]),
        np.asarray(result["current"]),
    )
    error = voltage - case["reference"]
    changes = np.flatnonzero(np.abs(np.diff(load)) > 1e-9) + 1
    settling = []
    failures = 0
    for j, start in enumerate(changes):
        stop = changes[j + 1] if j + 1 < len(changes) else len(load)
        segment = np.abs(error[start:stop]) > case["settling_band"]
        if segment.any() and np.flatnonzero(segment)[-1] == len(segment) - 1:
            failures += 1
        else:
            settling.append(
                (np.flatnonzero(segment)[-1] + 1 if segment.any() else 0) / case["frequency"]
            )
    output_power = float(np.mean(np.maximum(voltage, 0) * load))
    esr_loss = float(case["esr"] * np.mean((current - load) ** 2))
    return {
        "worst_droop_mv": float(max(0, -error.min()) * 1000),
        "worst_overshoot_mv": float(max(0, error.max()) * 1000),
        "rms_error_mv": float(np.sqrt(np.mean(error**2)) * 1000),
        "ripple_peak_to_peak_mv": float(np.ptp(voltage) * 1000),
        "settling_seconds": float(np.mean(settling)) if settling else None,
        "unsettled_transitions": failures,
        "rail_violation": int(np.any(voltage < 0.84)),
        "estimated_efficiency": output_power / (output_power + esr_loss)
        if output_power + esr_loss
        else None,
        "legal_effects": len(voltage),
    }
