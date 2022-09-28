import xlwt


def txt2xls(filename, xlsname):
    f = open(filename)  # 打开txt文本进行读取
    x = 0  # 在excel开始写的位置（y）
    y = 0  # 在excel开始写的位置（x）
    num_line = 0
    xls = xlwt.Workbook()
    sheet = xls.add_sheet('sheet1', cell_overwrite_ok=True)
    while True:  # 循环，读取文本里面的所有内容
        line = f.readline()  # 一行一行读取
        if not line:  # 如果没有内容，则退出循环
            break
        num_line += 1
        for value in line.split(';'):  # 读取出相应的内容写到x
            item = value
            # if y == 0:
            #     item = value
            # else:
            #     item = value[3:]
    #         #            item=i.strip().decode('utf8')
    #         item = i.strip()
            sheet.write(x, y, item)
            # print(x, y, item)
            y += 1  # 另起一列
        x += 1  # 另起一行
        y = 0  # 初始成第一列
    f.close()
    xls.save(xlsname)  # 保存
    print(num_line)


filename = './TestData/MPU-20220927.txt'
xlsname = './TestData/MPU-20220927.xls'
txt2xls(filename, xlsname)
