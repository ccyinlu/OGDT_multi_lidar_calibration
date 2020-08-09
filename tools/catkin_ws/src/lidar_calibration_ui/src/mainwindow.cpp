//
// Created by qzkj on 19-5-20.
//
#include <std_msgs/String.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(ros::NodeHandle node, QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    pub_trans_vec_ = node.advertise<std_msgs::String>("/adjust_order",1);
    pub_save_align_ = node.advertise<std_msgs::String>("/save_order",1);
    pub_car_ = node.advertise<std_msgs::String>("car_order",1);
	  node.param("lidar_child_yaml_file", lidar_child_yaml_file_, std::string(""));
    
	  ROS_INFO("lidar_child_yaml_file: %s", lidar_child_yaml_file_.c_str());

    connect(ui->doubleSpinBox_set_x,SIGNAL(valueChanged(double)),this,SLOT(trans_changed(double)));
    connect(ui->doubleSpinBox_set_y,SIGNAL(valueChanged(double)),this,SLOT(trans_changed(double)));
    connect(ui->doubleSpinBox_set_z,SIGNAL(valueChanged(double)),this,SLOT(trans_changed(double)));
    connect(ui->doubleSpinBox_set_roll,SIGNAL(valueChanged(double)),this,SLOT(trans_changed(double)));
    connect(ui->doubleSpinBox_set_pitch,SIGNAL(valueChanged(double)),this,SLOT(trans_changed(double)));
    connect(ui->doubleSpinBox_set_yaw,SIGNAL(valueChanged(double)),this,SLOT(trans_changed(double)));

    // connect(ui->doubleSpinBox_set_car_length,SIGNAL(valueChanged(double)),this,SLOT(car_changed(double)));
    // connect(ui->doubleSpinBox_set_car_width,SIGNAL(valueChanged(double)),this,SLOT(car_changed(double)));
    // connect(ui->doubleSpinBox_set_car_height,SIGNAL(valueChanged(double)),this,SLOT(car_changed(double)));
}

void MainWindow::on_pushButton_load_adjust_clicked(){
    YAML::Node lidar_child_yaml_config = YAML::LoadFile(lidar_child_yaml_file_);
    ui->doubleSpinBox_set_x->setValue(lidar_child_yaml_config["x"].as<double>());
    ui->doubleSpinBox_set_y->setValue(lidar_child_yaml_config["y"].as<double>());
    ui->doubleSpinBox_set_z->setValue(lidar_child_yaml_config["z"].as<double>());
    ui->doubleSpinBox_set_roll->setValue(lidar_child_yaml_config["roll"].as<double>());
    ui->doubleSpinBox_set_pitch->setValue(lidar_child_yaml_config["pitch"].as<double>());
    ui->doubleSpinBox_set_yaw->setValue(lidar_child_yaml_config["yaw"].as<double>());
}

void MainWindow::trans_changed(const double& trans_value){
    std::string str_msg;

    str_msg += num2str(ui->doubleSpinBox_set_x->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_y->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_z->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_roll->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_pitch->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_yaw->value());

    std_msgs::String msg;
    msg.data = str_msg.c_str();
    pub_trans_vec_.publish(msg);
}

void MainWindow::car_changed(const double& car){
    std::string str_msg;

    str_msg += num2str(ui->doubleSpinBox_set_car_length->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_car_width->value());
    str_msg += " ";
    str_msg += num2str(ui->doubleSpinBox_set_car_height->value());

    std_msgs::String msg;
    msg.data = str_msg.c_str();
    pub_car_.publish(msg);
}

void MainWindow::on_pushButton_save_adjust_clicked(){
    std_msgs::String tmp_str;
    pub_save_align_.publish(tmp_str);
}


MainWindow::~MainWindow()
{
    delete ui;
}