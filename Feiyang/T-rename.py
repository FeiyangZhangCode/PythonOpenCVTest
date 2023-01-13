import os

str_fileHome = './TestData/C/'
title_li = os.listdir(str_fileHome)
title_li = sorted(title_li, key=lambda x: os.path.getmtime(os.path.join(str_fileHome, x)), reverse=False)
loop_num = 0
for title in title_li:
    old_name = title
    c_num = old_name[0:2]
    strlist = old_name.split('.')
    new_name = strlist[0][3:]
    new_name = new_name + '-' + c_num + '.' + strlist[1]
    # print(new_name)
    os.rename(str_fileHome + old_name, str_fileHome + new_name)
