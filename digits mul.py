def func():
    
    n = int(input("integer n: "))
    if n==0 or n>1000:
      print("invalid num of digit")
      return func()
    x = int(input("integer x contains n digits:"))
    if  x!=0 and x!=1 and n!=1 :
      if x<(10**(n-1)) or x>((10**(n))-1) :
        print("You entered invalid numbers of digits")
        return func()  # Recursively call the function again
    
    m = int(input("integer m: "))
    if m==0 or m>1000:
      print("invalid num of digit")
      return func()
    y = int(input("integer y contains m digits:"))

    if  y!=0 and y!=1 and m!=1 :
      if y<(10**(m-1)) or y>((10**(m))-1) :
        print("You entered invalid numbers of digits")
        return func()  # Recursively call the function again

    result=x*y
    print(f"x*y=: {result}")
   
func()