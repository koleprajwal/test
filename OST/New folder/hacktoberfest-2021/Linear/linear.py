#Python Code for Linear Search

list = []

n = int(input("Enter the number of elements: "))

for i in range(0, n):
    element = int(input("Enter element: "))
    list.append(element)

print("The list is:", list)

key = int(input("Enter the element to search: "))

for i in range (0, len(list)):
    if list[i] == key:
        print("Element found at index:", i)
        break
else:
    print("Element not found")