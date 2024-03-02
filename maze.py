import numpy as np
import math

alpha = (0,1,0)

beta = (0,-1,0)

# Calculate the dot product of alpha and beta
dot_product = np.dot(alpha, beta)

# Calculate the magnitudes of alpha and beta
magnitude_alpha = np.linalg.norm(alpha)
magnitude_beta = np.linalg.norm(beta)

# Calculate the cosine of the angle between the vectors
cos_theta = dot_product / (magnitude_alpha * magnitude_beta)

# Calculate the angle in radians
theta = math.acos(cos_theta)

print(math.degrees(theta))
print(theta)
