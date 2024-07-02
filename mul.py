def enter_array():
    rows = int(input("Enter rows: "))
    print(f"You entered: {rows}")
    columns = int(input("Enter columns: "))
    print(f"You entered: {columns}")

    two_dim_array = []
    for i in range(rows):
        row = []
        for j in range(columns):
            value = int(input(f"Enter the value for element ({i},{j}): "))
            row.append(value)
        two_dim_array.append(row)
    return two_dim_array

def mul_func(arr_1, arr_2):
    rows_1 = len(arr_1)
    cols_1 = len(arr_1[0])
    rows_2 = len(arr_2)
    cols_2 = len(arr_2[0])

    if cols_1 != rows_2:
        print("Matrix multiplication is not possible with the given dimensions.")
        return None

    # Initialize the result array with zeros
    result = [[0 for _ in range(cols_2)] for _ in range(rows_1)]

    for i in range(rows_1):
        for j in range(cols_2):
            for k in range(cols_1):
                result[i][j] += arr_1[i][k] * arr_2[k][j]   
                

    return result

arr_1 = enter_array()
arr_2 = enter_array()
mul_arr = mul_func(arr_1, arr_2)

if mul_arr:
    print("Resultant Matrix after Multiplication:")
    for row in mul_arr:
        print(row)
