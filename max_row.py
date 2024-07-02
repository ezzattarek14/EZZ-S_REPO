def func():
    rows = int(input("Enter rows (1 < rows <= 20): "))
    print(f"You entered: {rows}")
    columns = int(input("Enter columns (1 < columns <= 20): "))
    print(f"You entered: {columns}")

    if rows > 20 or rows <= 1 or columns > 20 or columns <= 1:
        print("You entered invalid numbers")
        return func()  # Recursively call the function again
    else:
        two_dim_array = []
        for i in range(rows):
            row = []
            for j in range(columns):
                value = int(input(f"Enter the value for element ({i},{j}): "))
                row.append(value)
            two_dim_array.append(row)

        # Sum the elements of each row
        row_sums = [sum(row) for row in two_dim_array]

        # Find the row with the maximum sum
        max_sum = max(row_sums)
        max_row = row_sums.index(max_sum)

        print(f"The row with the maximum sum is: {max_row} with a sum of {max_sum}")
        two_dim_array[0], two_dim_array[max_row] = two_dim_array[max_row], two_dim_array[0]
        for row in two_dim_array:
            print(row)
func()
