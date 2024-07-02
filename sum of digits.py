
def func(sum,n):    
 
    digit=n%10
    n-=digit
    n/=10
    sum+=digit
    if n==0:
        print(f"sum of digits is :{sum}")
        return sum
    else:
        func(sum,n)


n = int(input("integer n: "))
sum=func(0,n)
