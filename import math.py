import math

number = int(input("Enter the number of terms for the series: "))
angle = float(input("Enter the angle in radians: "))

sin_approx = 0
for i in range(number):
    term = ((-1)**i) * (angle**(2*i+1)) / math.factorial(2*i+1)
    sin_approx += term
    

print(f"Approximated sin({angle}) using {number} terms: {sin_approx}")
print(f"Actual sin({angle}) from math.sin: {math.sin(angle)}")
    