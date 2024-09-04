import serial
import time


ser = serial.Serial('COM5', 9600, timeout=1)

def send_data_to_microcontroller(data, data_type):
    if data_type == 'int8':
        ser.write(b'I')  
        ser.write(bytes([data]))  
        time.sleep(0.5)  
        received_data = ser.readline().decode().strip()  
        if received_data:
            print(f"Modified data received from microcontroller: {received_data}")
    
    elif data_type == 'int16':
        ser.write(b'L')  
        ser.write(data.to_bytes(2, byteorder='little'))  
        time.sleep(0.5)  
        received_data = ser.readline().decode().strip()  
        if received_data:
            print(f"16-bit integer received from microcontroller: {received_data}")

    elif data_type == 'char':
        ser.write(b'C')  
        ser.write(data.encode())  
        time.sleep(0.5)  
        received_data = ser.readline().decode().strip()  
        if received_data:
            print(f"Character received from microcontroller: {received_data}")

    elif data_type == 'string':
        ser.write(b'S')  
        ser.write(data.encode())  
        time.sleep(0.5)  
        received_data = ser.readline().decode().strip()  
        if received_data:
            print(f"String received from microcontroller: {received_data}")

    elif data_type == 'array':
        ser.write(b'A')  
        for num in data:
            ser.write(bytes([num]))  
        time.sleep(0.5)  
        received_data = ser.readline().decode().strip()  
        if received_data:
            print(f"Array received from microcontroller: {received_data}")

def main():
    try:
        while True:
            user_choice = input("Choose input type (int8/int16/char/string/array): ").strip().lower()
            
            if user_choice == 'int8':
                user_input = int(input("Enter an 8-bit integer (0-255): "))
                send_data_to_microcontroller(user_input, 'int8')
                
            elif user_choice == 'int16':
                user_input = int(input("Enter a 16-bit integer (0-65535): "))
                send_data_to_microcontroller(user_input, 'int16')

            elif user_choice == 'char':
                user_input = input("Enter a character (e.g., 'A', 'Z'): ")
                if len(user_input) == 1:
                    send_data_to_microcontroller(user_input, 'char')
                else:
                    print("Please enter a single character.")
                    
            elif user_choice == 'string':
                user_input = input("Enter a string: ")
                send_data_to_microcontroller(user_input, 'string')
            
            elif user_choice == 'array':
                user_input = input("Enter array inputs (e.g., 100, 60, 123, 30): ")
                array = [int(x.strip()) for x in user_input.split(',')]
                send_data_to_microcontroller(array, 'array')
            
            else:
                print("Invalid choice. Please choose either 'int8', 'int16', 'char', 'string', or 'array'.")
                
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
