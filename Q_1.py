user_input = int(input("Enter an integer: "))
print(f"You entered: {user_input}")
number_of_dozens=(user_input-(user_input % 12))/12
left_over=user_input % 12
print(f"number_of_dozens: {number_of_dozens}")
print(f"left_ove: {left_over}")