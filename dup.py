def remove_duplicates_1(arr):
    new_arr = []
    for num in arr:
        if num not in new_arr:
            new_arr.append(num)
    return new_arr

def remove_duplicates_2(arr1,arr2):
   
    for num in arr2:
        if num not in arr1:
            arr1.append(num)
    return arr1

arr1 = list(map(int, input("Enter elements separated by space: ").split()))
filtered_arr_1 = remove_duplicates_1(arr1)
print("Filtered Array:", filtered_arr_1)
arr2 = list(map(int, input("Enter elements separated by space: ").split()))
filtered_arr_2 = remove_duplicates_1(arr2)
print("Filtered Array:", filtered_arr_2)
filtered_arr_3=remove_duplicates_2(filtered_arr_1,filtered_arr_2)
print("Filtered Array:", filtered_arr_3)

