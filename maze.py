def quick_sort(arr):
    if len(arr) <= 1:
        return arr
    pivot = arr[len(arr) // 2]
    left = [x for x in arr if x < pivot]
    middle = [x for x in arr if x == pivot]
    right = [x for x in arr if x > pivot]
    return quick_sort(left) + middle + quick_sort(right)

def sort_three_numbers(a, b, c):
    sorted_numbers = quick_sort([a, b, c])
    return sorted_numbers

# 테스트
dest_position = '1230.860.100'
x = dest_position.split('.')
print(x)

num1 = int(x[0])
num2 = int(x[1])
num3 = int(x[2])

sorted_nums = sort_three_numbers(num1, num2, num3)
print("Sorted Numbers:", sorted_nums)
