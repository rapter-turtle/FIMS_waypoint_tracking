import numpy as np
import matplotlib.pyplot as plt

# Example data (replace with your actual data)
# speed = np.array([2.0, 2.4, 2.8, 3.3, 3.9, 4.4, 5.0]) #m/s
# thrust = np.array([10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0]) # % 
# speed = np.array([1.7, 2.1, 2.5, 3.2, 3.8, 4.8]) #m/s
# thrust = np.array([30.0, 40.0, 50.0, 60.0, 70.0, 80.0]) # %
# speed = np.array([1.2, 2.1, 3.1, 4.4, 5.6]) #m/s
# thrust = np.array([45.0, 50.0, 55.0, 60.0, 65.0]) - 40.5 # %
speed = np.array([ 3.1, 5.0, 6.4, 7.2, 9.5, 13.1, 14.5]) #m/s
thrust_pwm = np.array([1659, 1696, 1718, 1725, 1746,1771, 1787]) # %
# speed = np.array([ 3.1, 5.0]) #m/s
# thrust_pwm = np.array([1659, 1696]) # %


thrust = (thrust_pwm - 1550)/3.9 - 25# convert pwm to Newton
# thrust = (thrust_pwm - 1550)/3.9# convert pwm to Newton

# Construct regression matrix [speed, speed^2]
A = np.column_stack((speed, speed**2))

# Solve least squares: thrust ≈ A @ [Xu, Xuu]
coeffs, residuals, rank, s = np.linalg.lstsq(A, thrust, rcond=None)
Xu, Xuu = coeffs

print(f"Xu  = {Xu:.6f}")
print(f"Xuu = {Xuu:.6f}")


speed_fit = np.linspace(min(speed), max(speed), 200)
thrust_fit = Xu * speed_fit + Xuu * speed_fit**2
# Xu = 4.41
# C = 40.5 
# thrust_fit = Xu * speed_fit + C

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