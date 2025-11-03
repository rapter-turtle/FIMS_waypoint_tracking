import numpy as np
import matplotlib.pyplot as plt

# Example data (replace with your actual data)
speed = np.array([2.0, 2.4, 2.8, 3.3, 3.9, 4.4, 5.0])
thrust = np.array([10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0])

# Construct regression matrix [speed, speed^2]
A = np.column_stack((speed, speed**2))

# Solve least squares: thrust ≈ A @ [Xu, Xuu]
coeffs, residuals, rank, s = np.linalg.lstsq(A, thrust, rcond=None)
Xu, Xuu = coeffs

print(f"Xu  = {Xu:.6f}")
print(f"Xuu = {Xuu:.6f}")


speed_fit = np.linspace(min(speed), max(speed), 200)
thrust_fit = Xu * speed_fit + Xuu * speed_fit**2

# ===========================
# 4️⃣  Plotting
# ===========================
plt.figure(figsize=(7,5))
plt.scatter(speed, thrust, color='blue', label='Measured data', s=60)
plt.plot(speed_fit, thrust_fit, 'r-', linewidth=2.5, label='Fitted model')
plt.xlabel('Speed u [m/s]', fontsize=12)
plt.ylabel('Thrust τ [N]', fontsize=12)
plt.title('Quadratic Fit: τ = Xu·u + Xuu·u²', fontsize=13)
plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(fontsize=11)
plt.tight_layout()
plt.show()