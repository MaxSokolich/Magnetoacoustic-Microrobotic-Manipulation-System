<<<<<<< HEAD
from itertools import combinations

def is_prime(num):
    """Returns True if num is a prime number, else False."""
    if num < 2:
        return False
    for i in range(2, int(num ** 0.5) + 1):
        if num % i == 0:
            return False
    return True

def find_prime_combinations(numbers):
    """Finds and prints two-number combinations whose sum is prime."""
    prime_combinations = []
    for subset in combinations(numbers, 2):  # Only two-number combinations
        subset_sum = sum(subset)
        if is_prime(subset_sum):
            prime_combinations.append((subset, subset_sum))
    return prime_combinations

# Numbers in the range of one-digit numbers (1 to 6)
numbers = list(range(1, 7))
prime_combinations = find_prime_combinations(numbers)

# Print the combinations and their sums
if prime_combinations:
    print("One-digit combinations (1 to 6) with prime sums:")
    for combo, sum_value in prime_combinations:
        print(f"Combination: {combo}, Sum: {sum_value}")
else:
    print("No combinations found with prime sums.")
=======
import math

def calculate_ricochet_angle(incident_angle):
    # Ensure that the incident angle is between 0 and 360 degrees
    incident_angle = incident_angle % 360
    # Calculate the ricochet angle
    return (2 * 180 - incident_angle) % 360

# Calculate ricochet angles for angles from 0 to 359 degrees
for angle in range(360):
    ricochet_angle = calculate_ricochet_angle(angle)
    print(f"Angle of incidence: {angle} degrees, Ricochet angle: {ricochet_angle} degrees")
>>>>>>> 9e8c59663ff3bedc7f3b6aa0d5d186cac71fd8d7
