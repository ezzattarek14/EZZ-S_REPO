##Recursive: The recursive version has exponential time complexity O(2^n) due to repeated calculations of the same terms, making it very slow for large n.
##Iterative: The iterative version has linear time complexity O(n), making it much more efficient for large n.


def fibonacci_recursive(n):
    if n <= 0:
        return 0
    elif n == 1:
        return 1
    else:
        return fibonacci_recursive(n-1) + fibonacci_recursive(n-2)

def print_fibonacci_recursive(n):
    for i in range(n):
        print(fibonacci_recursive(i))
    print()

# Example usage:
n = int(input("Enter the number of terms for the Fibonacci series (recursive): "))
print_fibonacci_recursive(n)
