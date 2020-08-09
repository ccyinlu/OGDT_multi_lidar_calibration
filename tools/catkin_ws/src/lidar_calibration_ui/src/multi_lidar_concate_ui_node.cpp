//
// Created by qzkj on 19-5-20.
//

#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_concate_ui");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    QApplication app(argc, argv);
    MainWindow window(private_nh);
    window.show();
    return app.exec();
}
