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
