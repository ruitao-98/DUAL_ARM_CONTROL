#include <chrono>
#include "real_robot_control/right_robot_control.h"
#include "real_robot_control/screwing_tool.h"
using namespace std;
using namespace Eigen;
endeffector ef;
RobotAdmittanceControl::RobotAdmittanceControl(){
    selection_vector.resize(6);
    selection_vector<<1, 1, 1, 0, 0, 0;
    // 质量，刚度，阻尼
    adm_m.resize(6);
    adm_k.resize(6);
    adm_d.resize(6);
    wish_force.resize(6);
    wish_force << 0 ,0, -5, 0, 0, 0;

    adm_m << 3, 3, 4, 2, 2, 2;
    adm_k << 500.0, 500.0, 1400.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 5 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[1] = 3 * sqrt(adm_m[1] * adm_k[1]);
    adm_d[5] = 2 * sqrt(adm_m[1] * adm_k[1]);
    // adm_d[2] = 300;
    cout << "k" << adm_k[0] <<endl;
    cout << "d" << adm_d[0]<<endl;

    // 初始化机器人
    robot.login_in("192.168.3.201"); //right_arm
    robot.power_on();
    robot.enable_robot();
    robot.set_tool_id(0);
    // reset();
    // go_to_pose();
    robot.servo_move_use_carte_NLF(20, 50, 400, 20, 20, 40);
    // robot.servo_speed_foresight(15, 0.03);
    robot.servo_move_enable(TRUE);
    robot.set_torque_sensor_mode(1);
    robot.set_compliant_type(1, 0);
    sleep(1);
    robot.set_compliant_type(0,0);
    char ver[100];
    robot.get_sdk_version(ver);
    std::cout << " SDK version is :" << ver << std::endl;
    object_length << 0, 0, 0.022;

    eef_offset << -0.0785, 0, 0.1169;
    eef_offset_to_sensor << -0.0785, 0, 0.0774;
    eef_offset = eef_offset + object_length;
    cout << "eef_offset" << eef_offset << endl;
    eef_offset_to_sensor = eef_offset_to_sensor + object_length;
    eef_offset_rotm_to_sensor = Eigen::AngleAxisd(-PI, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(-PI, Eigen::Vector3d::UnitX());

    eef_offset_rotm = Eigen::AngleAxisd(-PI, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(-PI, Eigen::Vector3d::UnitX());
}

RobotAdmittanceControl::~RobotAdmittanceControl(){
    // if (excution_thread.joinable()) {
    //     excution_thread.join();
    // }
    // if (sensor_thread.joinable()) {
    //     sensor_thread.join();
    // }
}


std::vector<Eigen::Matrix3d> RobotAdmittanceControl::calculateRotationMatrices(int N, double theta) {
    std::vector<Eigen::Matrix3d> rotationMatrices;
    for (int i = 0; i < N; ++i) {
        double phi = 2 * M_PI * i / N;
        Eigen::Vector3d axis(std::sin(phi), - std::cos(phi), 0);
        Eigen::AngleAxisd angleAxis(theta, axis.normalized());
        Eigen::Matrix3d rotationMatrix = angleAxis.toRotationMatrix();
        rotationMatrices.push_back(rotationMatrix);
    }
    return rotationMatrices;
}

void RobotAdmittanceControl::get_eef_pose(){

        link6_pos[0] = status.cartesiantran_position[0]/1000;
        link6_pos[1] = status.cartesiantran_position[1]/1000;
        link6_pos[2] = status.cartesiantran_position[2]/1000; //转化为米单位
        // 姿态欧拉角赋值
        current_rpy.rx = status.cartesiantran_position[3];
        current_rpy.ry = status.cartesiantran_position[4];
        current_rpy.rz = status.cartesiantran_position[5];
        // cout << current_rpy.rx << " " << current_rpy.ry << " "<< current_rpy.rz << endl;
        Rpy rpy;
        rpy.rx = current_rpy.rx; rpy.ry = current_rpy.ry; rpy.rz = current_rpy.rz;
        RotMatrix rot_matrix;
        // cout << "*******rpy_to_rot_matrix***********" << endl;

        robot.rpy_to_rot_matrix(&rpy, &rot_matrix);
        link6_rotm << rot_matrix.x.x, rot_matrix.x.y, rot_matrix.x.z,
                    rot_matrix.y.x, rot_matrix.y.y, rot_matrix.y.z,
                    rot_matrix.z.x, rot_matrix.z.y, rot_matrix.z.z;
        
        // cout << link6_rotm << endl;

        // cout << "*******eigen***********" << endl;
        // link6_rotm = Eigen::AngleAxisd(current_rpy.rz, Eigen::Vector3d::UnitZ()) *
        //             Eigen::AngleAxisd(current_rpy.ry, Eigen::Vector3d::UnitY()) *
        //             Eigen::AngleAxisd(current_rpy.rx, Eigen::Vector3d::UnitX());

        // cout << link6_rotm << endl;
        eef_rotm = link6_rotm * eef_offset_rotm;
        eef_pos = link6_pos + eef_rotm * (eef_offset_rotm.transpose() * eef_offset);
        // cout << "eef_pos"<< eef_pos << endl;
        // cout <<"eef_rotm"<< eef_rotm << endl;
        eigen_rpy = eef_rotm.eulerAngles(2,1,0);
        // cout << "eigen_rpy"<<eigen_rpy << endl;

}
 
void RobotAdmittanceControl::get_new_link6_pose(const Eigen::Vector3d& new_linear_eef, const Eigen::Matrix3d& new_angular_eef){
        // left_link6_pos = left_eef_pos - left_eef_rotm @ (self.left_eef_offset_rotm.T @ self.left_eef_offset)
        // left_link6_rotm = left_eef_rotm @ self.left_eef_offset_rotm.T
        // new_linear = new_linear_eef - new_angular_eef * (eef_offset_rotm.transpose() * eef_offset);
        // cout <<"new_angular_eef"<< new_angular_eef<<endl;
        // cout <<"new_linear_eef"<< new_linear_eef<<endl;
        new_angular = new_angular_eef * eef_offset_rotm.transpose();
        new_linear = new_linear_eef - (new_angular * eef_offset);
}

void RobotAdmittanceControl::updata_rotation(const Eigen::Matrix3d& current_rotm, const Eigen::Vector3d& angluar_disp, Eigen::Matrix3d& new_orientation){


    Eigen::AngleAxisd delta_rotation(angluar_disp.norm(), angluar_disp.normalized());
    // 更新当前旋转矩阵
    new_orientation = delta_rotation.toRotationMatrix() *  current_rotm;
    // cout<< current_rotm <<endl;
    // cout<< new_orientation <<endl;
}


void RobotAdmittanceControl::ros_init(int argc, char** argv){
    ros::init(argc, argv, "real_robot_control");

    nh = std::make_shared<ros::NodeHandle>();
    for_pub = nh->advertise<real_robot_control::force_pub>("robot_force", 10);

}

void RobotAdmittanceControl::update_robot_state(){
    robot.get_robot_status(&status);
}

void RobotAdmittanceControl::get_world_force(){
    for (int i=0; i<6; i++){
        local_force[i] = status.torq_sensor_monitor_data.actTorque[i];
    }
    Eigen::VectorXd local_force_compensate(6);
    local_force_compensate.head<3>() = local_force.head<3>();
    local_force_compensate.tail<3>() = local_force.tail<3>() - eef_offset_to_sensor.cross(local_force_compensate.head<3>());
    
    // 位置赋值
    link6_pos[0] = status.cartesiantran_position[0]/1000;
    link6_pos[1] = status.cartesiantran_position[1]/1000;
    link6_pos[2] = status.cartesiantran_position[2]/1000; //转化为米单位
    // 姿态欧拉角赋值
    current_rpy.rx = status.cartesiantran_position[3];
    current_rpy.ry = status.cartesiantran_position[4];
    current_rpy.rz = status.cartesiantran_position[5];
    // 欧拉角转旋转矩阵，赋值
    link6_rotm = Eigen::AngleAxisd(current_rpy.rz, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(current_rpy.ry, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(current_rpy.rx, Eigen::Vector3d::UnitX());
    
    world_force.head<3>() = link6_rotm * local_force_compensate.head<3>();
    world_force.tail<3>() = link6_rotm * local_force_compensate.tail<3>();

    f.X = world_force[0];
    f.Y = world_force[1];
    f.Z = world_force[2];
    f.MX = world_force[3];
    f.MY = world_force[4];
    f.MZ = world_force[5];
    for_pub.publish(f);
}
void RobotAdmittanceControl::get_tcp_force(){
    for (int i=0; i<6; i++){
        local_force[i] = status.torq_sensor_monitor_data.actTorque[i];
    }
    // cout << eef_offset_rotm_to_sensor << endl;
    tcp_force.head<3>() = eef_offset_rotm_to_sensor.transpose() * local_force.head<3>();
    tcp_force.tail<3>() = eef_offset_to_sensor.cross(tcp_force.head<3>()) + eef_offset_rotm_to_sensor.transpose() * local_force.tail<3>();


    f.X = tcp_force[0];
    f.Y = tcp_force[1];
    f.Z = tcp_force[2];
    f.MX = tcp_force[3];
    f.MY = tcp_force[4];
    f.MZ = tcp_force[5];
    for_pub.publish(f);
}

void RobotAdmittanceControl::tcp_admittance_control(){
 
        
        // 使用Eigen数组操作进行clip
        clipped_tcp_force = tcp_force.array().min(upper).max(lower);
        clipped_tcp_force = wish_force + clipped_tcp_force;
        e.head<3>() = eef_rotm.transpose() * (eef_pos - eef_pos_d); //将基坐标系下的位置偏移转化为tcp坐标系下的偏移，后续所有计算都是在当前时刻的tcp坐标系下，计算下一时刻的数值
        Eigen::Matrix3d e_rotm = eef_rotm.transpose() * eef_rotm_d;

        Eigen::AngleAxisd angle_axis(e_rotm);
        // 获取旋转向量（旋转轴 * 旋转角度）
        Eigen::Vector3d rotation_vector = angle_axis.angle() * angle_axis.axis();
        e.tail<3>() = rotation_vector;
        e_dot = eef_vel;
        Eigen::VectorXd MA = clipped_tcp_force - adm_k.cwiseProduct(e) - adm_d.cwiseProduct(e_dot);
        Eigen::VectorXd adm_acc = MA.cwiseQuotient(adm_m);
        Eigen::VectorXd adm_vel = eef_vel + adm_acc * T;
        linear_disp = adm_vel.head(3) * T;
        angular_disp = adm_vel.tail(3) * T;
        linear_disp = selection_vector.head<3>().cwiseProduct(linear_disp);
        angular_disp = selection_vector.tail<3>().cwiseProduct(angular_disp);
        eef_vel = adm_vel;
}

void RobotAdmittanceControl::admittance_control(){
 
        get_eef_pose();
        // 使用Eigen数组操作进行clip
        clipped_world_force = world_force.array().min(upper).max(lower);

        clipped_world_force = wish_force + clipped_world_force;
        e.head<3>() = eef_pos - eef_pos_d;
        Eigen::Matrix3d e_rotm = eef_rotm * eef_rotm_d.transpose();

        Eigen::AngleAxisd angle_axis(e_rotm);
        // 获取旋转向量（旋转轴 * 旋转角度）
        Eigen::Vector3d rotation_vector = angle_axis.angle() * angle_axis.axis();
        e.tail<3>() = rotation_vector;
        e_dot = eef_vel;
        Eigen::VectorXd MA = clipped_world_force - adm_k.cwiseProduct(e) - adm_d.cwiseProduct(e_dot);
        Eigen::VectorXd adm_acc = MA.cwiseQuotient(adm_m);
        Eigen::VectorXd adm_vel = eef_vel + adm_acc * T;
        linear_disp = adm_vel.head(3) * T;
        angular_disp = adm_vel.tail(3) * T;
        linear_disp = selection_vector.head<3>().cwiseProduct(linear_disp);
        angular_disp = selection_vector.tail<3>().cwiseProduct(angular_disp);
        //trick 因为x方向旋转的反的
        // angular_disp[1] = - angular_disp[1];
        eef_vel = adm_vel;

}

void RobotAdmittanceControl::calculation_loop(){
    for(int j =0; j<1000000; j++){
        auto start_time = std::chrono::high_resolution_clock::now();
        update_robot_state();
        get_world_force();
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 正确的延迟方法
        // sleep(0.1);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout<< "the number is:"<< j << "    "<<"calculation time is"<< duration.count()<<"ms"<<endl;
        
    }
}

void RobotAdmittanceControl::excution_loop(){
    for(int step=0; step<10000; step++){

        auto start_time = std::chrono::high_resolution_clock::now();
        cart.tran.x = 0; cart.tran.y = -0.02; cart.tran.z = 0;
        cart.rpy.rx = 0; cart.rpy.ry = 0; cart.rpy.rz = 0;
        // std::lock_guard<std::mutex> lock(robot_mutex); // 加锁，进入临界区
        // robot.linear_move(&cart, INCR, TRUE, 1);
        // {
            // std::lock_guard<std::mutex> lock(robot_mutex); 
        robot.servo_p(&cart, INCR, loop_rate);
        // } // 离开作用域，锁自动释放
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout<<  "the number is:"<< step << "    "<<"excution time is"<< duration.count()<<"ms"<<endl;
    }
}

void RobotAdmittanceControl::excution_calculation_loop(){
    update_robot_state();
    get_world_force();
    admittance_control();

    linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
    angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01);
    new_linear_eef = eef_pos + linear_disp_clipped;
    cout << new_linear_eef << endl;
    //fixed rotation
    new_rotm_eef = eef_rotm;

    //no-fixed rotation

    // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);


    get_new_link6_pose(new_linear_eef, new_rotm_eef);
    // eigen_rpy = new_angular.eulerAngles(2,1,0);
    // cout <<"rpy" << eigen_rpy << endl;
    // new_rpy.rx = eigen_rpy[2]; new_rpy.ry = eigen_rpy[1]; new_rpy.rz = eigen_rpy[0]; 
    new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(0,1); new_rotm.z.x = new_angular(0,2);
    new_rotm.x.y = new_angular(1,0); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(1,2);
    new_rotm.x.z = new_angular(2,0); new_rotm.y.z = new_angular(2,1); new_rotm.z.z = new_angular(2,2);
    robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
    new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
    new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
    // sleep(0.1);
    robot.servo_p(&new_pos, ABS, loop_rate);
}

void RobotAdmittanceControl::tcp_admittance_run(){
    robot.servo_move_enable(true);
    //直线搜索***********************************************************************************
    wish_force << 0, 0, 0, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 1; //选择向量

    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    int item = 0;

    // ruled-based rotation
    int N = 6;
    double theta = 0 * PI / 180;
    std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(N, theta);
    // cout << rotationMatrices[1] << endl;
    eef_rotm_d =  eef_rotm * rotationMatrices[0]; //期望的位置是当前位置，期望的位姿是经过旋转变换以后的位姿

    // eef_rotm_d = eef_rotm;
    get_new_link6_pose(eef_pos_d, eef_rotm_d); // 转化期望的位姿到link6
    // eigen_rpy = new_angular.eulerAngles(2,1,0);
    // new_rpy.rx = eigen_rpy[2]; new_rpy.ry = eigen_rpy[1]; new_rpy.rz = eigen_rpy[0]; 
    // cout <<"eigen_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;

    // new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(0,1); new_rotm.z.x = new_angular(0,2);
    // new_rotm.x.y = new_angular(1,0); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(1,2);
    // new_rotm.x.z = new_angular(2,0); new_rotm.y.z = new_angular(2,1); new_rotm.z.z = new_angular(2,2);
    new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
    new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
    new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
    robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角

    cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;



    while (item < 10000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        update_robot_state();
        get_tcp_force();
        get_eef_pose();  //更新机器人状态

        eef_pos_d = eef_pos; //期望的位置不断更新，始终保持是当前状态，期望的位姿保持不变
        // 
        item = item + 1;

        // 导纳控制的范畴
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量

        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
        // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);

        //fixed rotation
        new_rotm_eef = eef_rotm; 

        get_new_link6_pose(new_linear_eef, eef_rotm_d);
        // // new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(0,1); new_rotm.z.x = new_angular(0,2);
        // // new_rotm.x.y = new_angular(1,0); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(1,2);
        // // new_rotm.x.z = new_angular(2,0); new_rotm.y.z = new_angular(2,1); new_rotm.z.z = new_angular(2,2);
        // // robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
        // // cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;

        // eigen_rpy = new_angular.eulerAngles(2,1,0); // 转欧拉角
        // new_rpy.rx = eigen_rpy[2]; new_rpy.ry = eigen_rpy[1]; new_rpy.rz = eigen_rpy[0]; 
        
        // cout<< new_pos.rpy.rx << new_pos.rpy.ry << new_pos.rpy.rz << endl;
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        cout<<"new_trans" << new_pos.tran.x << " " << new_pos.tran.y << " "<< new_pos.tran.z << " "<< endl;
        cout <<"new_rpy" << (new_pos.rpy.rx / PI) * 180 << "  " << (new_pos.rpy.ry / PI) * 180<< "  " << (new_pos.rpy.rz / PI) * 180<<"  " << endl; //new_rpy.rx不受导纳控制输出的影响，一开始就写死了

        // robot.linear_move(&new_pos, ABS, true, 5);
        robot.servo_p(&new_pos, ABS, loop_rate);
        // std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "linear search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }

}

void RobotAdmittanceControl::screw_assembly_search(){

    robot.servo_move_enable(true);
    //直线搜索***********************************************************************************
    wish_force << 0, 0, -5, 0, 0, 0;  //期望力
    selection_vector<<0, 0, 1, 0, 0, 0; //选择向量

    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    int item = 0;

    while (item < 1000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();  //更新机器人状态

        item = item + 1;

        eef_pos_d = eef_pos; //期望的位置不断更新，始终保持是当前状态，期望的位姿保持不变
        eef_rotm_d = eef_rotm;

        if (tcp_force[2] > 2){
            cout << "-----------linear search stopped------------" << endl;
            sleep(1);
            break;
        }

        cout << tcp_force[1] << endl;
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        linear_disp_clipped = linear_disp_clipped;  // 增加搜索的偏执
        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
        // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);

        //fixed rotation
        new_rotm_eef = eef_rotm; 

        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
        new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
        new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
        robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角

        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        cout<<"new_trans" << new_pos.tran.x << " " << new_pos.tran.y << " "<< new_pos.tran.z << " "<< endl;
        cout <<"new_rpy" << (new_pos.rpy.rx / PI) * 180 << "  " << (new_pos.rpy.ry / PI) * 180<< "  " << (new_pos.rpy.rz / PI) * 180<<"  " << endl; //new_rpy.rx不受导纳控制输出的影响，一开始就写死了
        robot.servo_p(&new_pos, ABS, loop_rate);
        // std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "linear search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }

    //曲线搜索***********************************************************************************
    cout << "-----------spiral search started------------" << endl;
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    wish_force << 0, 0, 0, 0, 0, 0;  //期望力
    selection_vector<<0, 0, 1, 0, 0, 0; //选择向量，表示只控制y轴
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    Eigen::Vector3d traj; //轨迹的向量表示
    item = 0;
    double a = 0;
    double b = 0.3/(2*PI); //毫米单位
    double theta = 0;

    while (item < 1200)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        // 生成平面螺旋曲线
        theta = (2*PI / 100) + theta; //螺旋线的角度
        double radius = a + b * theta;
        double x = radius * cos(theta) * 0.001;
        double y = radius * sin(theta) * 0.001; // 此处缩放到米为单位，以便于与导纳控制输出对其
        double z = 0;
        traj << x, y, z;

        item = item + 1;

        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        if (tcp_force[2] < 0.5){
            cout << "force" <<  tcp_force[2] << endl;
            cout << "spiral search stoped" <<endl;
            break;
        }
        cout << "force" <<  tcp_force[2] << endl;
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        linear_disp_clipped = linear_disp_clipped + traj;
        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。

        //fixed rotation
        new_rotm_eef = eef_rotm;

        //no-fixed rotation
        // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;

        cout<<"new_trans" << new_pos.tran.x << " " << new_pos.tran.y << " "<< new_pos.tran.z << " "<< endl;
        cout <<"new_rpy" << (new_pos.rpy.rx / PI) * 180 << "  " << (new_pos.rpy.ry / PI) * 180<< "  " << (new_pos.rpy.rz / PI) * 180<<"  " << endl;

        robot.servo_p(&new_pos, ABS, loop_rate);
        // std::this_thread::sleep_for(std::chrono::milliseconds(50)); 

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "spiral search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }

    //姿态搜索***********************************************************************************
    cout << "-----------pose search------------" << endl;
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    wish_force << 0, 0, -2, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量，表示只控制y轴
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

     // ruled-based rotation
    int N = 6;
    double theta = 0 * PI / 180;
    std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(N, theta);
    eef_rotm_d =  eef_rotm * rotationMatrices[0]; //期望的位置是当前位置，期望的位姿是经过旋转变换以后的位姿

    get_new_link6_pose(eef_pos_d, eef_rotm_d); // 转化期望的位姿到link6
    new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
    new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
    new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
    robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角

    cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;

    while (item < 5000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        item = item + 1;

        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
  
        cout << "force" <<  tcp_force[2] << endl;
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        linear_disp_clipped = linear_disp_clipped;
        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。

        //fixed rotation
        new_rotm_eef = eef_rotm;

        //no-fixed rotation
        // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);
        // std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "insertion item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }
}

void RobotAdmittanceControl::start(){
    // excution_loop();
    update_robot_state();
    get_world_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;
    // double init_z = eef_pos_d[2];
 
    
    int item = 0;
    while (item < 400000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        // eef_pos_d = eef_pos;
        // eef_rotm_d = eef_rotm;
        cout << "eef_d" << eef_pos_d << endl;
 
        item = item + 1;
        // eef_pos_d(1) -= 0.0000; //y方向1mm移动
        excution_calculation_loop();
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }
    reset();
    
    // excution_thread = std::thread(&RobotAdmittanceControl::excution_loop, this);
    // sensor_thread = std::thread(&RobotAdmittanceControl::calculation_loop, this);

    // for(int step=0; step<10000; step++){
    //     auto start_time = std::chrono::high_resolution_clock::now();
    //     cart.tran.x = 0; cart.tran.y = 1; cart.tran.z = 0;
    //     cart.rpy.rx = 0; cart.rpy.ry = 0; cart.rpy.rz = 0;
    //     // robot.linear_move(&cart, INCR, TRUE, 1);
    //     robot.servo_p(&cart, INCR);
    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
    //     cout<<  "the number is:"<< step << "    "<<"excution time is"<< duration.count()<<"ms"<<endl;
    // }

}

void RobotAdmittanceControl::go_to_pose(){
    CartesianPose goal_pose;
    goal_pose.tran.x = -58.14; goal_pose.tran.y = 509.42; goal_pose.tran.z = 195;
    goal_pose.rpy.rx = (180 * PI) / 180; goal_pose.rpy.ry = (0* PI) / 180; goal_pose.rpy.rz = (-60 * PI) / 180;
    robot.servo_move_enable(false);
    robot.linear_move(&goal_pose, ABS, TRUE, 12);

    update_robot_state();
    get_eef_pose();
    int N = 6;
    double theta = -3 * PI / 180;
    std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(N, theta);
    // cout << rotationMatrices[1] << endl;
    eef_rotm =  eef_rotm * rotationMatrices[0];
    cout << eef_rotm << endl;
    get_new_link6_pose(eef_pos, eef_rotm);
    eigen_rpy = new_angular.eulerAngles(2,1,0);
    cout <<"rpy" << eigen_rpy << endl;
    new_rpy.rx = eigen_rpy[2]; new_rpy.ry = eigen_rpy[1]; new_rpy.rz = eigen_rpy[0]; 

    // new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(0,1); new_rotm.z.x = new_angular(0,2);
    // new_rotm.x.y = new_angular(1,0); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(1,2);
    // new_rotm.x.z = new_angular(2,0); new_rotm.y.z = new_angular(2,1); new_rotm.z.z = new_angular(2,2);
    // robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角

    cout << "new_linear" << new_linear << endl;
    cout << new_rpy.rx << " "<<new_rpy.ry << " "<<new_rpy.rz << " "<<endl;
    goal_pose.tran.x = new_linear[0] * 1000; goal_pose.tran.y = new_linear[1]* 1000; goal_pose.tran.z = new_linear[2]* 1000;
    goal_pose.rpy.rx = eigen_rpy[2]; goal_pose.rpy.ry = eigen_rpy[1]; goal_pose.rpy.rz = eigen_rpy[0];
    // cout << goal_pose << endl;
    robot.linear_move(&goal_pose, ABS, TRUE, 12);
    sleep(3);
    cout << "the robot is at the goal pose" << endl;

}

void RobotAdmittanceControl::reset(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    cout<< "reset the robot"<<endl;
    JointValue right_joint_pos = { -PI/3, PI/3, 2*PI/3, PI/2, -PI/2, PI/2 };
    robot.joint_move(&right_joint_pos, ABS, true, 0.1);
    // JointValue left_joint_pos = { 0, PI/2, PI / 2, PI , PI / 2, (135 * PI) / 180 };
    // robot.joint_move(&left_joint_pos, ABS, true, 0.2);
    cout<< "the robot was resetted"<<endl;
}

void RobotAdmittanceControl::test(){
    // ef.width_reduce_or_increase_full(-1);
    // ef.width_recovery();

}

int main(int argc, char** argv){

    RobotAdmittanceControl robot_control;
    robot_control.ros_init(argc, argv);
    // robot_control.reset();
    robot_control.update_robot_state();
    // robot_control.get_world_force();
    robot_control.get_eef_pose();

    
    // robot_control.screw_assembly_search();
    bool running = true;
    char input;

    while (running) {
        std::cout << "Enter 1 with initialize, 2 without, 3 screwing or any other key to exit:" << std::endl;
        std::cin >> input;
        switch(input) {
            case '1':

                std::cout << "screwing" << std::endl;
                robot_control.go_to_pose();
                robot_control.tcp_admittance_run();
                break;

            case '2':

                std::cout << "insertion for the right tip" << std::endl;
                robot_control.tcp_admittance_run();
                break;
            
            case '3':

                std::cout << "insertion for the right tip" << std::endl;
                robot_control.screw_assembly_search();
                break;

            default:
                // 程序结束
                std::cout << "task stoped" << std::endl;
                running = false; // 设置循环为不运行状态，以退出
                break;

        }
    }


    return 0;
}

