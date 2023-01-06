from pynput.keyboard import Listener, Key


def keyboardListener(on_press):
    with Listener(on_press=on_press) as listener:
        listener.join()


def on_press(key):
    print("enter：{0}".format(key))


if __name__ == '__main__':
    keyboardListener(on_press)