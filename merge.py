
def clear_similar(arr):
    new_arr=[]
    for i in range(len(arr)):
        var=arr[i]
        if var in new_arr:
            continue
        else:
            new_arr.append(var)
    return new_arr
        




arr_1=[1,2,3,4,5,6,77,88]
arr_2=[3,4,5,6,7,9]
if arr_1[0]<arr_2[0]:
    arr_1.extend(arr_2)
   # print(arr_1) 
    new_array=clear_similar(arr_1)
    print(new_array)

else:
    arr_2.extend(arr_1)
   # print(arr_2)
    new_array=clear_similar(arr_2)
    print(new_array)

