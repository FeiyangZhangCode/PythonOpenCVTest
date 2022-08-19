


if __name__ == '__main__':
    ver_left = [[0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0]]

    ver_left[0][0] = 1.1
    ver_left[0][1] = 2.1
    ver_left[0][2] = 1.3
    ver_left[0][3] = 4.1
    ver_left[0][4] = 1.5
    ver_left[0][5] = 1
    ver_left[0][6] = 2
    ver_left[0][7] = 3
    ver_left[0][8] = 4
    temp_list = [2.0, 3.1, 2.2, 3.3, 1.4, 4, 3, 2, 1]
    ver_left.append(temp_list)
    for ver_list in ver_left:
        for a, b, d_a, w, d_m, x1, y1, x2, y2 in ver_list:
            print(a, b, d_a, w, d_m, x1, y1, x2, y2)


