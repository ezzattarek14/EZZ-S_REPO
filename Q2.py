import math

a = int(input("Enter a parameter: "))
b = int(input("Enter b parameter: "))
c = int(input("Enter c parameter: "))
d = int(input("Enter d parameter: "))
e = int(input("Enter e parameter: "))
f = int(input("Enter f parameter: "))
g = int(input("Enter g parameter: "))
sum_1=(a+b/c)/(d+e/(f+g))
print(f"SUM: {sum_1}")


X = int(input("Enter X in radian: "))
Y = int(input("Enter Y in radian: "))
result = (math.sin(X+Y))**2
print(f"RESULT = {result}")