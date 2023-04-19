import serial

myserial = serial.Serial()


def port_open_recv(Com,Bud):
    myserial.port = Com
    myserial.baudrate = Bud
    myserial.bytesize = 8
    myserial.stopbits = 1
    myserial.parity = "N"
    myserial.open()
    if(myserial.isOpen()):
        print("串口打开成功！")
    else:
        print("串口打开失败！")


def send(send_data):
    if(myserial.isOpen()):
        a = 2
        # myserial.write(bytes.fromhex(send_data))
        myserial.write(send_data)
        print("发送成功", send_data)
    else:
        print("发送失败！")


if __name__ == '__main__':
    port_open_recv("com9",115200)
    height = 1
    width = 300
    send_buffer = [0xaa, 0, 0, 0, 0, 0xbb]
    send_buffer[1] = (height >> 8) & 0xff
    send_buffer[2] = (height) & 0xff
    send_buffer[3] = (width >> 8) & 0xff
    send_buffer[4] = (width) & 0xff

    while True:
        a = input("发送!")
        send(send_buffer)
        # for i in range(6):
        #     print(send_buffer[i])
        #     # send(binascii.unhexlify(send_buffer[i]).decode('utf-8'))
        #     send(send_buffer[i])
