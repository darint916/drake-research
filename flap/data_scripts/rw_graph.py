import pandas as pd
import matplotlib.pyplot as plt

# Load the data from CSV
df = pd.read_csv("../data/rw_angles.csv")

# Plotting
plt.figure(figsize=(10, 6))  # Set the figure size (optional)
plt.plot(df['time'], df['passive_pitch_angles'], label='Passive Pitch Angle', color='blue')
plt.plot(df['time'], df['flap_input_angles'], label='Flap Input Angle', color='red')

# Adding titles and labels
plt.title('Passive Pitch Angles and Flap Input Angles Over Time')
plt.xlabel('Time')
plt.ylabel('Angle (degrees)')
plt.legend()  # Show legend

# Save the figure
plt.savefig('rw_joint.png')

# Display the plot
plt.show()
