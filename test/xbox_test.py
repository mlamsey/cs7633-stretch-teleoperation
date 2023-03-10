from inputs import devices

if __name__ == '__main__':
    for device in devices:
        print(device)

    print(' ')
    for device in devices.gamepads:
        print(device)