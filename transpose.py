def reverse_2d_array(arr):
    if not arr:
        return arr
    
    rows = len(arr)
    cols = len(arr[0])
    
    reversed_arr = [[0] * cols for _ in range(rows)]

    for i in range(rows):
        for j in range(cols):
            reversed_arr[j][i] = arr[i][j]
    
    return reversed_arr

rows = int(input("Enter the number of rows: "))
cols = int(input("Enter the number of columns: "))

arr = []

print("Enter the elements row by row:")

for i in range(rows):
    row = []
    for j in range(cols):
        row.append(int(input(f"Enter element for row {i+1}, column {j+1}: ")))
    arr.append(row)  # Append the entire row after completing the inner loop

print("Original array:")
for row in arr:
    print(row)

reversed_arr = reverse_2d_array(arr)

print("\nReversed array (rows and columns swapped):")
for row in reversed_arr:
    print(row)
