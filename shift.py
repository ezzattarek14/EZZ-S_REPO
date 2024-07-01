num_integers = int(input("Enter the number of integers: "))

num_shifts = int(input("Enter the number of shifts: "))

integers = []
print(f"Enter {num_integers} integers:")

def shift_right(lst):
    if not lst:
        return lst
    last_element = lst[-1]
    for i in range(len(lst) - 1, 0, -1):
        lst[i] = lst[i - 1]
 
    lst[0] = last_element

for _ in range(num_integers):
    integers.append(int(input()))

for i in range(num_shifts):
    shift_right(integers)
print(integers)