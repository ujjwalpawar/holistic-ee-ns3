import os
import time
if os.path.exists("fifo2"):
        os.unlink("fifo2")

os.mkfifo("fifo2")
fifo=os.open("fifo1",os.O_RDONLY)
fifo2=os.open("fifo2", os.O_WRONLY)

print("Opening FIFO...")
def generate_log():
    time.sleep(1)


while True:
    data=os.read(fifo,1)
    data=data.decode("utf-8")

    if(not data):
        continue
    print("Received: ",data)
    if(data=='0'):
        print("Do something \n")
        generate_log()
        os.write(fifo2,"1".encode("utf-8"))

    
