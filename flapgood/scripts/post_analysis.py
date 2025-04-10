import json
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
from mpl_toolkits.mplot3d import Axes3D

current_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(current_dir, '..', 'opt_data', 'alpha_4')
# Load the optimization results
with open(os.path.join(data_dir, 'opt_result.json'), 'r') as f:
    results = json.load(f)

# 1. Print basic optimization information
print(f"Optimal solution: {results['x']}")
print(f"Optimal function value: {results['fun']}")
print(f"Number of function evaluations: {results['nfev']}")
print(f"Number of iterations: {results['nit']}")
print(f"Convergence measure: {results['convergence']}")

# 2. Visualize parameter values
param_names = [f"Parameter {i+1}" for i in range(len(results['x']))]
plt.figure(figsize=(10, 6))
plt.bar(param_names, results['x'])
plt.title('Optimal Parameter Values')
plt.ylabel('Value')
plt.xticks(rotation=45)
plt.tight_layout()
plt.savefig('optimal_parameters.png')
plt.show()

# 3. Visualize population diversity
population = np.array(results['population'])
population_energies = np.array(results['population_energies'])

# Calculate statistics for each parameter across population
param_stats = {
    'mean': np.mean(population, axis=0),
    'std': np.std(population, axis=0),
    'min': np.min(population, axis=0),
    'max': np.max(population, axis=0)
}

plt.figure(figsize=(12, 8))
for i in range(len(results['x'])):
    plt.subplot(2, 3, i+1)
    plt.hist(population[:, i], bins=15)
    plt.axvline(results['x'][i], color='r', linestyle='dashed', linewidth=2)
    plt.title(f'Parameter {i+1} Distribution')
plt.tight_layout()
plt.savefig('parameter_distributions.png')
plt.show()

# 4. Visualize parameter correlations (for pairs of parameters)
plt.figure(figsize=(12, 10))
sns.heatmap(np.corrcoef(population.T), annot=True, cmap='coolwarm', 
            xticklabels=param_names, yticklabels=param_names)
plt.title('Parameter Correlations in Final Population')
plt.tight_layout()
plt.savefig('parameter_correlations.png')
plt.show()

# 5. 3D visualization for three selected parameters (adjust indices as needed)
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Select three parameters to visualize (e.g., first three)
param_indices = [0, 1, 2]

# Color points by their energy (function value)
scatter = ax.scatter(
    population[:, param_indices[0]],
    population[:, param_indices[1]],
    population[:, param_indices[2]],
    c=population_energies,
    cmap='viridis',
    alpha=0.7
)

# Highlight the optimal solution
ax.scatter(
    [results['x'][param_indices[0]]],
    [results['x'][param_indices[1]]],
    [results['x'][param_indices[2]]],
    color='red',
    s=100,
    marker='*'
)

ax.set_xlabel(param_names[param_indices[0]])
ax.set_ylabel(param_names[param_indices[1]])
ax.set_zlabel(param_names[param_indices[2]])
plt.colorbar(scatter, label='Function Value')
plt.title('3D Visualization of Solution Space')
plt.savefig('3d_solution_space.png')
plt.show()