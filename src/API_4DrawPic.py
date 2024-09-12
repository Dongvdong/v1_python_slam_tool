
#!/usr/bin/python
# -*- coding: UTF-8 -*-
import math


from pyecharts import Line3D


from pyecharts import Line

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np

# matplotlib画一条单独的3D轨迹
def Draw_trace1(x,y,z):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    ax.set_title("3D_Curve")
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_zlabel("z(m)")
    figure = ax.plot(x, y, z, c='r')
    
    plt.show()


# matplotlib将多条3D轨迹画在同一个图里
def API_txt_to_Draw3D(list_name_xyz):
   
    
    x_list=[]
    y_list=[]
    z_list=[]
    for data_i in list_name_xyz:
        nam_i=data_i[0]
        x_i=float(data_i[1])
        y_i=float(data_i[2])
        z_i=float(data_i[3])
        x_list.append(x_i)
        y_list.append(y_i)
        z_list.append(z_i)
    return x_list,y_list,z_list


def Draw3D_trace_more(tracelist):

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    #ax = fig.add_subplot(projection='3d')

    #ax.set_zlim3d(zmin=-2, zmax=2) #纵轴范围
        
    #plt.xlim(-1500,100)
    #plt.ylim(-50,50)

    #ax.set_title("3D_Curve")
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_zlabel("z(m)")

    
    ax.set_aspect("auto")#设置x,y z轴等比例

    color=["blue","green","red","orange","purple","pink","yellow"]
    linestyle=["-","--","-.","-"] #":"


    for i in range(0,len(tracelist)):
        #print(tracelist[i])

        xi_list,yi_list,zi_list=API_txt_to_Draw3D(tracelist[i]) 

        color_i=color[i]
        figure = ax.plot(xi_list, yi_list, zi_list, c=color_i,linestyle=linestyle[i])

    plt.grid()#网格线
    plt.show()

#画单个2维平面轨迹
def Draw2D_trace_gpsvreal(x1,y1,x2,y2):
    
    #绘画轨迹
    print("画轨迹图")
    plt.title('Error mapped onto trajectory')
    plt.ylabel('Y(m)')
    plt.xlabel('X(m)')
    #plt.annotate('blue ', xy=(2,5), xytext=(2, 10),arrowprops=dict(facecolor='black', shrink=0.01),)
    #x=[1, 2, 3, 4,3,2,1]
    #y=[1, 4, 9, 16,3,5,6]
    plt.plot(x1,y1,color='b',linestyle='dashed')

    '''
    color：线条颜色，值r表示红色（red）
    marker：点的形状，值o表示点为圆圈标记（circle marker）
    linestyle：线条的形状，值dashed表示用虚线连接各点
    '''
    #x=[3, 4, 5, 6]
    #y=[1, 4, 9, 16]

    #plt.plot(x2, y2, color='r',marker='o',linestyle='dashed')
    plt.plot(x2, y2, color='r')

    '''
    axis：坐标轴范围
    语法为axis[xmin, xmax, ymin, ymax]，
    也就是axis[x轴最小值, x轴最大值, y轴最小值, y轴最大值]
    '''

    #plt.axis([0, 6, 0, 20])
    plt.grid()#网格线
    #plt.grid(axis='x',color = 'r', linestyle = '--', linewidth = 0.5)## 设置 y 就在轴方向显示网格线
    plt.show()   

#画3个对对比 2维平面轨迹
"""
输入 data_list 多个轨迹的enu列表
     要画的x轴 xlabel='X(m)' "Y(m)" "Z(m)"
     要画的y轴 ylabeln='X(m)' "Y(m)" "Z(m)"


"""


def Draw2D_trace_gpsvreal_list(data_list,xlabel='X(m)',ylabeln='Y(m)'):
    
    color=["blue","green","red","orange","purple","pink","yellow"]
    '''
    '-'       solid    line style
    '--'      dashed   line style
    '-.'      dash-dot line style
    ':'       dotted   line style
    '''
    linestyle=["-","--","-.","-"] #:
    xmax=0
    xmin=0
    ymax=0
    ymin=0
    for i in range(0,len(data_list)):

      
        linexylist=data_list[i]
        linexlist=[]
        lineylist=[]
        #根据轴 决定画的是侧视图还是俯视图
        if xlabel=="X(m)":
            linexlist=linexylist[0]
        elif xlabel=="Y(m)":
            linexlist=linexylist[1]
        elif xlabel=="Z(m)":
            linexlist=linexylist[2]    



        if ylabeln=="X(m)":
            lineylist=linexylist[0]
        elif ylabeln=="Y(m)":
            lineylist=linexylist[1]
        elif ylabeln=="Z(m)":
            lineylist=linexylist[2]

        threah=0
        #if xlabel=="X(m)" and ylabeln=="Y(m)":
            #threah=100
       

        # xmax=int(max(linexlist)+threah)
        # xmin=int(min(linexlist)-threah)
        # ymax=int(max(lineylist)+threah)
        # ymin=int(min(lineylist)-threah)
    
        #绘画轨迹
        #print("画轨迹图")
        #plt.title('Error mapped onto trajectory')
        
        plt.xlabel(xlabel)
        plt.ylabel(ylabeln)
        #plt.annotate('blue ', xy=(2,5), xytext=(2, 10),arrowprops=dict(facecolor='black', shrink=0.01),)
        #x=[1, 2, 3, 4,3,2,1]
        #y=[1, 4, 9, 16,3,5,6]
        color_i=color[i]
        line_i=linestyle[i]
        

        plt.plot(linexlist,lineylist,color=color_i,linestyle=line_i)

        #plt.xlim(xmin,xmax)
        #plt.ylim(ymin,ymax) 



    plt.grid()#网格线
    plt.show()   
       
'''
        color：线条颜色，值r表示红色（red）

            cnames = {
            'aliceblue':            '#F0F8FF',
            'antiquewhite':         '#FAEBD7',
            'aqua':                 '#00FFFF',
            'aquamarine':           '#7FFFD4',
            'azure':                '#F0FFFF',
            'beige':                '#F5F5DC',
            'bisque':               '#FFE4C4',
            'black':                '#000000',
            'blanchedalmond':       '#FFEBCD',
            'blue':                 '#0000FF',
            'blueviolet':           '#8A2BE2',
            'brown':                '#A52A2A',
            'burlywood':            '#DEB887',
            'cadetblue':            '#5F9EA0',
            'chartreuse':           '#7FFF00',
            'chocolate':            '#D2691E',
            'coral':                '#FF7F50',
            'cornflowerblue':       '#6495ED',
            'cornsilk':             '#FFF8DC',
            'crimson':              '#DC143C',
            'cyan':                 '#00FFFF',
            'darkblue':             '#00008B',
            'darkcyan':             '#008B8B',
            'darkgoldenrod':        '#B8860B',
            'darkgray':             '#A9A9A9',
            'darkgreen':            '#006400',
            'darkkhaki':            '#BDB76B',
            'darkmagenta':          '#8B008B',
            'darkolivegreen':       '#556B2F',
            'darkorange':           '#FF8C00',
            'darkorchid':           '#9932CC',
            'darkred':              '#8B0000',
            'darksalmon':           '#E9967A',
            'darkseagreen':         '#8FBC8F',
            'darkslateblue':        '#483D8B',
            'darkslategray':        '#2F4F4F',
            'darkturquoise':        '#00CED1',
            'darkviolet':           '#9400D3',
            'deeppink':             '#FF1493',
            'deepskyblue':          '#00BFFF',
            'dimgray':              '#696969',
            'dodgerblue':           '#1E90FF',
            'firebrick':            '#B22222',
            'floralwhite':          '#FFFAF0',
            'forestgreen':          '#228B22',
            'fuchsia':              '#FF00FF',
            'gainsboro':            '#DCDCDC',
            'ghostwhite':           '#F8F8FF',
            'gold':                 '#FFD700',
            'goldenrod':            '#DAA520',
            'gray':                 '#808080',
            'green':                '#008000',
            'greenyellow':          '#ADFF2F',
            'honeydew':             '#F0FFF0',
            'hotpink':              '#FF69B4',
            'indianred':            '#CD5C5C',
            'indigo':               '#4B0082',
            'ivory':                '#FFFFF0',
            'khaki':                '#F0E68C',
            'lavender':             '#E6E6FA',
            'lavenderblush':        '#FFF0F5',
            'lawngreen':            '#7CFC00',
            'lemonchiffon':         '#FFFACD',
            'lightblue':            '#ADD8E6',
            'lightcoral':           '#F08080',
            'lightcyan':            '#E0FFFF',
            'lightgoldenrodyellow': '#FAFAD2',
            'lightgreen':           '#90EE90',
            'lightgray':            '#D3D3D3',
            'lightpink':            '#FFB6C1',
            'lightsalmon':          '#FFA07A',
            'lightseagreen':        '#20B2AA',
            'lightskyblue':         '#87CEFA',
            'lightslategray':       '#778899',
            'lightsteelblue':       '#B0C4DE',
            'lightyellow':          '#FFFFE0',
            'lime':                 '#00FF00',
            'limegreen':            '#32CD32',
            'linen':                '#FAF0E6',
            'magenta':              '#FF00FF',
            'maroon':               '#800000',
            'mediumaquamarine':     '#66CDAA',
            'mediumblue':           '#0000CD',
            'mediumorchid':         '#BA55D3',
            'mediumpurple':         '#9370DB',
            'mediumseagreen':       '#3CB371',
            'mediumslateblue':      '#7B68EE',
            'mediumspringgreen':    '#00FA9A',
            'mediumturquoise':      '#48D1CC',
            'mediumvioletred':      '#C71585',
            'midnightblue':         '#191970',
            'mintcream':            '#F5FFFA',
            'mistyrose':            '#FFE4E1',
            'moccasin':             '#FFE4B5',
            'navajowhite':          '#FFDEAD',
            'navy':                 '#000080',
            'oldlace':              '#FDF5E6',
            'olive':                '#808000',
            'olivedrab':            '#6B8E23',
            'orange':               '#FFA500',
            'orangered':            '#FF4500',
            'orchid':               '#DA70D6',
            'palegoldenrod':        '#EEE8AA',
            'palegreen':            '#98FB98',
            'paleturquoise':        '#AFEEEE',
            'palevioletred':        '#DB7093',
            'papayawhip':           '#FFEFD5',
            'peachpuff':            '#FFDAB9',
            'peru':                 '#CD853F',
            'pink':                 '#FFC0CB',
            'plum':                 '#DDA0DD',
            'powderblue':           '#B0E0E6',
            'purple':               '#800080',
            'red':                  '#FF0000',
            'rosybrown':            '#BC8F8F',
            'royalblue':            '#4169E1',
            'saddlebrown':          '#8B4513',
            'salmon':               '#FA8072',
            'sandybrown':           '#FAA460',
            'seagreen':             '#2E8B57',
            'seashell':             '#FFF5EE',
            'sienna':               '#A0522D',
            'silver':               '#C0C0C0',
            'skyblue':              '#87CEEB',
            'slateblue':            '#6A5ACD',
            'slategray':            '#708090',
            'snow':                 '#FFFAFA',
            'springgreen':          '#00FF7F',
            'steelblue':            '#4682B4',
            'tan':                  '#D2B48C',
            'teal':                 '#008080',
            'thistle':              '#D8BFD8',
            'tomato':               '#FF6347',
            'turquoise':            '#40E0D0',
            'violet':               '#EE82EE',
            'wheat':                '#F5DEB3',
            'white':                '#FFFFFF',
            'whitesmoke':           '#F5F5F5',
            'yellow':               '#FFFF00',
            'yellowgreen':          '#9ACD32'}



        marker：点的形状，值o表示点为圆圈标记（circle marker）
                '.'       point marker
                ','       pixel marker
                'o'       circle marker
                'v'       triangle_down marker
                '^'       triangle_up marker
                '<'       triangle_left marker
                '>'       triangle_right marker
                '1'       tri_down marker
                '2'       tri_up marker
                '3'       tri_left marker
                '4'       tri_right marker
                's'       square marker
                'p'       pentagon marker
                '*'       star marker
                'h'       hexagon1 marker
                'H'       hexagon2 marker
                '+'       plus marker
                'x'       x marker
                'D'       diamond marker
                'd'       thin_diamond marker
                '|'       vline marker
                '_'       hline marker


        linestyle：线条的形状，值dashed表示用虚线连接各点
                '-'       solid line style
                '--'      dashed line style
                '-.'      dash-dot line style
                ':'       dotted line style
'''

   



#画单个折线图 水平误差折线 
def Draw2D(realgps_enu_listx,realgps_enu_listy,slamgps_enu_listx,slamgps_enu_listy):

    line = Line("真值和定位轨迹") 
    v_x = [5, 20, 36, 10, 10, 100]
    v_y = [55, 60, 16, 20, 15, 80]
    line.add("真值",
        realgps_enu_listx, 
        realgps_enu_listy, 
        #mark_point=["average"] 
    )

    line.add( 
        "SLAM",
        slamgps_enu_listx,
        slamgps_enu_listy,
        #mark_line=[“average”] 表示补充一条平均值线
        mark_line=["average", "max", "min"],
        #mark_point=["average", "max", "min"],
        #mark_point_symbol="diamond",
        #mark_point_textcolor="#40ff27",
        is_smooth=True,
    )

    line.render()

#画多个折线图 在同一个图里 水平误差折线 
def Draw2D_error_onePic(slam_enu_error_list_xyz):
    # 第一个为真值


    # len(gps_data_list)和 len(slam_data_list)数量应该是对等的
   
    error_list=[]

    hengzuobiao=[]
    fps=20.0
    time_interbval=1.0/fps
    

    for i in range(0,len(slam_enu_error_list_xyz)):

        slam_enu_error_i=slam_enu_error_list_xyz[i]

        #line = Line("Positioning  Error (m) "+ str(i)) 


        error_x_list=slam_enu_error_i[0]
        error_y_list=slam_enu_error_i[1]
        error_z_list=slam_enu_error_i[2]

        for j in range(0,len(error_x_list)):
            hengzuobiao.append((j*time_interbval))


        plt.figure(figsize=(20, 10), dpi=100)
        
        plt.plot(hengzuobiao, error_x_list, c='red', label="x")
        plt.plot(hengzuobiao, error_y_list, c='green', linestyle='--', label="y")
        plt.plot(hengzuobiao, error_z_list, c='blue', linestyle='-.', label="z")
        plt.scatter(hengzuobiao, error_x_list, c='red')
        plt.scatter(hengzuobiao, error_y_list, c='green')
        plt.scatter(hengzuobiao, error_z_list, c='blue')
        plt.legend(loc='best')
        #plt.yticks(range(0, 50, 5))
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.xlabel("Time (s)", fontdict={'size': 16})
        plt.ylabel("Error (m)", fontdict={'size': 16})
        plt.title("Position Error", fontdict={'size': 20})
        plt.show()


#画单组 3个折线图 各自单独一个图里 水平误差折线 
def Draw2D_error_MorePic(slam_enu_error_list_xyz):
    # 第一个为真值
    # len(gps_data_list)和 len(slam_data_list)数量应该是对等的
   
    fps=10.0
    time_interbval=1.0/fps
    
    plt.figure()
    
    '''
    num = None,               # 设定figure名称。系统默认按数字升序命名的figure_num（透视表输出窗口）e.g. “figure1”。可自行设定figure名称，名称或是INT，或是str类型；
    figsize=None,             # 设定figure尺寸。系统默认命令是rcParams["figure.fig.size"] = [6.4, 4.8]，即figure长宽为6.4 * 4.8；
    dpi=None,                 # 设定figure像素密度。系统默命令是rcParams["sigure.dpi"] = 100；
    facecolor=None,           # 设定figure背景色。系统默认命令是rcParams["figure.facecolor"] = 'w'，即白色white；
    edgecolor=None, frameon=True,    # 设定要不要绘制轮廓&轮廓颜色。系统默认绘制轮廓，轮廓染色rcParams["figure.edgecolor"]='w',即白色white；
    FigureClass=<class 'matplotlib.figure.Figure'>,   # 设定使不使用一个figure模板。系统默认不使用；
    '''
    #fig, axs = plt.subplots(nrows=3, ncols=1)#3行1列  figsize=(20, 6) dpi=100 像素密度
    colors = ['red', 'green', 'blue']
    line_style = ['-', '--', '-.']
    y_labels = ["x Error (m)", "y Error (m)", "z Error (m)"] #Error (m)
    
    
    
     
    slam_enu_error_i=slam_enu_error_list_xyz


    error_x_list=slam_enu_error_i[0]
    error_y_list=slam_enu_error_i[1]


    
    error_z_list=slam_enu_error_i[2]
    x_i=[]
    for j in range(0,len(error_x_list)):
        x_i.append(j)

    x_data=x_i
    y_data = [error_x_list, error_y_list, error_z_list]

    # xmin=int(min(x_data))
    # xmax=int(max(x_data))
    # ymin=int(min(y_data))
    # ymax=int(max(y_data))
    # yth=2

    for i in range(1,4):
        plt.subplot(3,1,i)


        
        #plt.plot([0,1],[0,1])
        i=i-1
        # axs[i].plot(x_data, y_data[i], c=colors[i], label=y_labels[i], linestyle=line_style[i],linewidth=1,marker='.'
    # ,markeredgecolor=colors[i],markersize='1',markeredgewidth=1) #label='total' alpha=0.5
        plt.plot(x_data, y_data[i], c=colors[i], label=y_labels[i], linestyle=line_style[i],linewidth=1,marker='.',markersize=0.1)
        #plt.scatter(x_data, y_data[i], c=colors[i])
        plt.xlabel("ID")
        plt.ylabel(y_labels[i])
        #axs[i].legend(loc='best') #默认: 对于Axes， ‘best’, 对于Figure, 'upper right'
        
        #axs[i].set_aspect(aspect=0.1)#y轴的单位刻度显示长度 与 x轴的单位刻度显示长度 的比例
        #axs[i].axis('equal')#设置y轴和x轴的单位刻度显示长度相同
        
        #axs[i].set_yticks(range(ymin,ymax, 2)) # y轴的间距和显示范围
        
         
        plt.grid(True, linestyle='--', alpha=0.5)#背景网格线
        #plt.set_xlabel("Time (s)", fontdict={'size': 10})#x轴坐标名字 和 字号
        #plt.set_ylabel(y_labels[i], fontdict={'size': 10}, rotation=90) #y轴坐标名字 和 字号
        #plt.set_title("Position {}".format(y_labels[i]), fontdict={'size': 12}) #标题

    #fig.autofmt_xdate()
    plt.show()

#画单组 3个折线图 各自单独一个图里 水平误差折线 
def Draw2D_error_MorePicv2(slam_enu_error_list_xyz):
    # 第一个为真值
    # len(gps_data_list)和 len(slam_data_list)数量应该是对等的
   
    fps=10.0
    time_interbval=1.0/fps
    
    '''
    num = None,               # 设定figure名称。系统默认按数字升序命名的figure_num（透视表输出窗口）e.g. “figure1”。可自行设定figure名称，名称或是INT，或是str类型；
    figsize=None,             # 设定figure尺寸。系统默认命令是rcParams["figure.fig.size"] = [6.4, 4.8]，即figure长宽为6.4 * 4.8；
    dpi=None,                 # 设定figure像素密度。系统默命令是rcParams["sigure.dpi"] = 100；
    facecolor=None,           # 设定figure背景色。系统默认命令是rcParams["figure.facecolor"] = 'w'，即白色white；
    edgecolor=None, frameon=True,    # 设定要不要绘制轮廓&轮廓颜色。系统默认绘制轮廓，轮廓染色rcParams["figure.edgecolor"]='w',即白色white；
    FigureClass=<class 'matplotlib.figure.Figure'>,   # 设定使不使用一个figure模板。系统默认不使用；
    '''
    fig, axs = plt.subplots(nrows=3, ncols=1)#3行1列  figsize=(20, 6) dpi=100 像素密度
    colors = ['red', 'green', 'blue']
    line_style = ['-', '--', '-.']
    y_labels = ["x Error (m)", "y Error (m)", "z Error (m)"] #Error (m)
    
    
    
     
    slam_enu_error_i=slam_enu_error_list_xyz


    error_x_list=slam_enu_error_i[0]
    error_y_list=slam_enu_error_i[1]


    
    error_z_list=slam_enu_error_i[2]
    x_i=[]
    for j in range(0,len(error_x_list)):
        x_i.append((j*time_interbval))

    x_data=x_i
    y_data = [error_x_list, error_y_list, error_z_list]

    # xmin=int(min(x_data))
    # xmax=int(max(x_data))
    # ymin=int(min(y_data))
    # ymax=int(max(y_data))
    # yth=2

    for i in range(3):
        # axs[i].plot(x_data, y_data[i], c=colors[i], label=y_labels[i], linestyle=line_style[i],linewidth=1,marker='.'
    # ,markeredgecolor=colors[i],markersize='1',markeredgewidth=1) #label='total' alpha=0.5
        axs[i].plot(x_data, y_data[i], c=colors[i], label=y_labels[i], linestyle=line_style[i],linewidth=1,marker='.',markersize=0.1)
        #axs[i].scatter(x_data, y_data[i], c=colors[i])
        #axs[i].legend(loc='best') #默认: 对于Axes， ‘best’, 对于Figure, 'upper right'
        
        #axs[i].set_aspect(aspect=0.1)#y轴的单位刻度显示长度 与 x轴的单位刻度显示长度 的比例
        #axs[i].axis('equal')#设置y轴和x轴的单位刻度显示长度相同
        
        #axs[i].set_yticks(range(ymin,ymax, 2)) # y轴的间距和显示范围
        
         
        axs[i].grid(True, linestyle='--', alpha=0.5)#背景网格线
        axs[i].set_xlabel("Time (s)", fontdict={'size': 10})#x轴坐标名字 和 字号
        axs[i].set_ylabel(y_labels[i], fontdict={'size': 10}, rotation=90) #y轴坐标名字 和 字号
        axs[i].set_title("Position {}".format(y_labels[i]), fontdict={'size': 12}) #标题

    #fig.autofmt_xdate()
    plt.show()

     


#画多组 多个折线图 各自单独一个图里 水平误差折线 
def Draw2D_error_More33Pic(slam_enu_error_list_xyz):


    fps=10.0
    time_interbval=1.0/fps
    
    '''
    num = None,               # 设定figure名称。系统默认按数字升序命名的figure_num（透视表输出窗口）e.g. “figure1”。可自行设定figure名称，名称或是INT，或是str类型；
    figsize=None,             # 设定figure尺寸。系统默认命令是rcParams["figure.fig.size"] = [6.4, 4.8]，即figure长宽为6.4 * 4.8；
    dpi=None,                 # 设定figure像素密度。系统默命令是rcParams["sigure.dpi"] = 100；
    facecolor=None,           # 设定figure背景色。系统默认命令是rcParams["figure.facecolor"] = 'w'，即白色white；
    edgecolor=None, frameon=True,    # 设定要不要绘制轮廓&轮廓颜色。系统默认绘制轮廓，轮廓染色rcParams["figure.edgecolor"]='w',即白色white；
    FigureClass=<class 'matplotlib.figure.Figure'>,   # 设定使不使用一个figure模板。系统默认不使用；
    '''
    fig, axs = plt.subplots(nrows=3, ncols=1,dpi=100)#3行1列  figsize=(20, 6) dpi=100 像素密度
    #colors = ['blue','green','red']
    colors = ['blue','green','red']
    line_style = ['-', '--', '-.']
    #datalabls=["nogps-nogps","gps-nogps","gps-gps"]
    datalabls=["true","vio + gps","vio"]
    y_labels = ["x Error (m)", "y Error (m)", "z Error (m)"] #Error (m)


    
    for i in range(0,len(slam_enu_error_list_xyz)):

     
        slam_enu_error_i=slam_enu_error_list_xyz[i]

        error_x_list=slam_enu_error_i[0]
        error_y_list=slam_enu_error_i[1]
        error_z_list=slam_enu_error_i[2]
        x_i=[]
        for j in range(0,len(error_x_list)):
            
            x_i.append((j*time_interbval))

        x_data=x_i
        y_data = [error_x_list, error_y_list, error_z_list]

        print("数据点总共",len(x_i))

        
        for ii in range(0,3):
           # axs[i].plot(x_data, y_data[i], c=colors[i], label=y_labels[i], linestyle=line_style[i],linewidth=1,marker='.'
        # ,markeredgecolor=colors[i],markersize='1',markeredgewidth=1) #label='total' alpha=0.5
            #添加lable标签显示
            #axs[ii].plot(x_data, y_data[ii], c=colors[i], label=datalabls[i], linestyle=line_style[i],linewidth=1,marker='.',markersize=0.1)
            axs[ii].plot(x_data, y_data[ii], c=colors[i], linestyle=line_style[i],linewidth=1,marker='.',markersize=0.1)


            #axs[ii].scatter(x_data, y_data[ii], c=colors[i])
            axs[ii].legend(loc='best')
            #axs[ii].gca().set_box_aspect((3, 5, 2))  # 当x、y、z轴范围之比为3:5:2时。
            #axs[ii].set_yticks(range(-4, 4, 2))

            #if ii==2:
                #axs[ii].set_yticks(range(-4, 8, 2))
            # if ii==0:
            #     axs[ii].set_yticks(range(-60, 30, 10))
            # elif ii==1:
            #     axs[ii].set_yticks(range(-30, 70, 20)) # y轴的间距和显示范围
            # else:
            #     axs[ii].set_yticks(range(-20, 30, 10)) 
            
            axs[ii].grid(True, linestyle='--', alpha=0.5)#背景网格线
            axs[ii].set_xlabel("Time (s)", fontdict={'size': 10})#x轴坐标名字 和 字号
            axs[ii].set_ylabel(y_labels[ii], fontdict={'size': 10}, rotation=90) #y轴坐标名字 和 字号  Y轴说明是否旋转90度
            axs[ii].set_title("Position {}".format(y_labels[ii]), fontdict={'size': 12}) #标题

    fig.autofmt_xdate()
    plt.show()


def API_Cal_wucha(errorlist_txyz0_error):
        # 0 累计误差 计算总体转化均方根误差
        error_all=0
        for i in range(0,len(errorlist_txyz0_error)):
            #print(errorlist_txyz0_error[i])

            #distance=sqrt((errorlist_txyz0_error[i][0])**2+(errorlist_txyz0_error[i][1])**2)
            #
            distance=sqrt((errorlist_txyz0_error[i][0])**2+(errorlist_txyz0_error[i][1])**2+(errorlist_txyz0_error[i][2])**2)
            #print(i,time_realgps_slamgps_list[i])
            error_all=error_all+distance
        error_averrange=sqrt(error_all/len(errorlist_txyz0_error))
        return error_averrange



if __name__ == "__main__":

    


    from API_0File_Read_Write import *

    data_list=[] #文件名
    data_list.append("data/1GNSS_from_img.txt")
    #data_list.append("data/2ENU_from_GNSS.txt") 
    data_list.append("data/3GNSS_From_ENU.txt")

    draw_tracelist=[]# 存放多组要绘画的3D图数据 分列存放

    for txt_name_i in data_list:
       
        list_name_xyz = API_read2txt(txt_name_i)     
        draw_tracelist.append(list_name_xyz)
    
    Draw3D_trace_more(draw_tracelist)


'''  
        #1 gps真值
        #enu_realgps_txt_savename0=root_path+"真实gps完整.txt"
        enu_realgps_txt_savename0=root_path+"带gps建图带gps定位"+"/匹配enu_real"+slamgps_txt_name
        #2 不同状态的定位
        enu_slamgps_txt_savename2=root_path+"不带gps建图不带gps定位"+"/匹配enu_slam"+slamgps_txt_name
        enu_slamgps_txt_savename1=root_path+"带gps建图不带gps定位"+"/匹配enu_slam"+slamgps_txt_name
        enu_slamgps_txt_savename3=root_path+"带gps建图带gps定位"+"/匹配enu_slam"+slamgps_txt_name
        
        data_list=[] #文件名
        data_list.append(enu_realgps_txt_savename0)
        data_list.append(enu_slamgps_txt_savename1)
        data_list.append(enu_slamgps_txt_savename2)
        data_list.append(enu_slamgps_txt_savename3)
    
        
        
        draw_tracelist=[]# enu数据列表
        for i in range(0,len(data_list)):

            list_txyz0,list_x0,list_y0,list_z0 = readtxt_ENU(data_list[i])
            draw_tracelist.append([list_x0,list_y0,list_z0])


        #==========1 画多个3D轨迹在同一个3D图 对比轨迹
        Draw3D_trace_more(draw_tracelist)
        
        #==========2 画多个3D轨迹的2D侧视图
        Draw2D_trace_gpsvreal_list(draw_tracelist,"X(m)","Y(m)") #-俯视图xy
        Draw2D_trace_gpsvreal_list(draw_tracelist,"X(m)","Z(m)") #-左视图xz
        Draw2D_trace_gpsvreal_list(draw_tracelist,"Y(m)","Z(m)") #-右视图yz


        #============3 画2D折线图误差

        #3-1 加载数据 折线误差slam值

        SlamEnuError_txtNamelist=[]  #保存slam enu error轨迹 文件名
        #SlamEnuError_txtNamelist.append(enu_realgps_txt_savename0)
        SlamEnuError_txtNamelist.append(enu_slamgps_txt_savename3)
        SlamEnuError_txtNamelist.append(enu_slamgps_txt_savename1)
        SlamEnuError_txtNamelist.append(enu_slamgps_txt_savename2)

        draw_slamEnutraceError_list=[] #保存slam enu轨迹 数据

        for i in range(0,len(SlamEnuError_txtNamelist)):

            errorlist_txyz0_error,list_x0_error,list_y0_error,list_z0_error = readtxt_ENU_Error(SlamEnuError_txtNamelist[i])

            errorlist_txyx0_error_skip=[]
            errorlist_txyy0_error_skip=[]
            errorlist_txyz0_error_skip=[]

            # #步长 3
            # for i in range(0,len(errorlist_txyz0_error),100):
            #     errorlist_txyx0_error_skip.append(list_x0_error[i])
            #     errorlist_txyy0_error_skip.append(list_y0_error[i])
            #     errorlist_txyz0_error_skip.append(list_z0_error[i])

            draw_slamEnutraceError_list.append([list_x0_error,list_y0_error,list_z0_error])
            #draw_slamEnutraceError_list.append([errorlist_txyx0_error_skip,errorlist_txyy0_error_skip,errorlist_txyz0_error_skip])
        
        #=============3-2 画数据 2D折线图画在多个行里
        Draw2D_error_More33Pic(draw_slamEnutraceError_list)
'''





        
    