import threading

mode = 0
def take_input():
    global mode
    while(True):
        print("6")
        n = input()
        print("15")
        mode = int(n)

t1 = threading.Thread(target=take_input)
t1.daemon = True
t1.start()

while(True):
    print(mode)
    if(mode == 2):
        

        break
print("broke")