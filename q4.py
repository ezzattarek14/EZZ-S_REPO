import math

# Calculate the factorial of a number
number =int(input("Enter a number < =12  "))
if number <=12:
    result = math.factorial(number)
    print(f"The factorial of {number} is {result}")
else :
    print(f"number should be  < =12")
 