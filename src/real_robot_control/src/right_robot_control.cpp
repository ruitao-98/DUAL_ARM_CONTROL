#include <chrono>
#include "real_robot_control/right_robot_control.h"
#include "real_robot_control/screwing_tool.h"

using namespace std;
using namespace Eigen;
endeffector ef;
// typedef actionlib::SimpleActionClient<real_robot_control::screwAction> Client;

RobotAdmittanceControl::RobotAdmittanceControl()
    : nh(std::make_shared<ros::NodeHandle>()),
      client(*nh, "screwactions", true) { 
        
    // Initialize the client member variable
    for_pub = nh->advertise<real_robot_control::force_pub>("robot_force", 10);
    pos_pub = nh->advertise<real_robot_control::robot_pos_pub>("robot_pos", 10);
    for_pos_pub = nh->advertise<real_robot_control::force_pos_pub>("for_pos", 10);
    listener_sub = nh->subscribe("orientation_phi", 10, &RobotAdmittanceControl::do_orien, this);
    
    // Wait for the action server to start
    client.waitForServer();

    selection_vector.resize(6);
    selection_vector<<1, 1, 1, 0, 0, 0;
    // 质量，刚度，阻尼
    adm_m.resize(6);
    adm_k.resize(6);
    adm_d.resize(6);
    wish_force.resize(6);
    wish_force << 0 ,0, -5, 0, 0, 0;

    adm_m << 3, 3, 4, 2, 2, 2;
    adm_k << 700.0, 700.0, 1300.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 4 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3.5 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[1] = 3.5 * sqrt(adm_m[1] * adm_k[1]);
    adm_d[2] = 2.8 * sqrt(adm_m[2] * adm_k[2]);
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
    robot.servo_move_use_carte_NLF(50, 200, 800, 30, 60, 100);
    // robot.servo_speed_foresight(15, 0.03);
    robot.servo_move_enable(TRUE);
    robot.set_torque_sensor_mode(1);
    robot.set_compliant_type(1, 0);
    sleep(1);
    robot.set_compliant_type(0,0);

    object_length << 0, 0, 0; 

    eef_offset_basic << -0.0785, 0, 0.1169;
    eef_offset_to_sensor_basic << -0.0785, 0, 0.0774;

    eef_offset = eef_offset_basic + object_length;
    cout << "eef_offset" << eef_offset << endl;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + object_length;

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
void RobotAdmittanceControl::do_orien(const real_robot_control::orientation_pub::ConstPtr& orien_p){
    adjust_phi = orien_p->phi;
    phi_changed = 1;
}

void RobotAdmittanceControl::done_cb(const actionlib::SimpleClientGoalState &state, const real_robot_control::screwResultConstPtr &result){
    if (state.state_ == state.SUCCEEDED)
    {
        screw_execute_result = result->result;
        ROS_INFO("screw_execute_result:%d",result->result);
    } else {
        ROS_INFO("failed！");
    }

}
//服务已经激活
void RobotAdmittanceControl::active_cb(){
    ROS_INFO("activated....");
}
//处理连续反馈
void RobotAdmittanceControl::feedback_cb(const real_robot_control::screwFeedbackConstPtr &feedback){
    screw_execute_status = feedback->screw_status; //0表示没有运行， 1表示正在运行
    ROS_INFO("screw_execute_status:%d",screw_execute_status);
    // ROS_INFO("洗涤进度为:%d%s", feedback->progress_bar, "%");
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


// void RobotAdmittanceControl::ros_init(int argc, char** argv){
//     ros::init(argc, argv, "real_robot_control");

//     nh = std::make_shared<ros::NodeHandle>();
//     for_pub = nh->advertise<real_robot_control::force_pub>("robot_force", 10);
//     // client = nh->serviceClient<real_robot_control::screwsrv>("screwservice");
//     Client client("screwactions",true);
//     //等待服务启动
//     client.waitForServer();
// }

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



}

void RobotAdmittanceControl::tcp_admittance_control(){
 
        
    // 使用Eigen数组操作进行clip
    clipped_tcp_force = tcp_force.array().min(upper).max(lower);
    clipped_tcp_force = wish_force + clipped_tcp_force;
    e.head<3>() = eef_rotm.transpose() * (eef_pos - eef_pos_d); //将基坐标系下的位置偏移转化为tcp坐标系下的偏移，后续所有计算都是在当前时刻的tcp坐标系下，计算下一时刻的数值
    Eigen::Matrix3d e_rotm = (eef_rotm * eef_rotm_d.transpose());

    Eigen::AngleAxisd angle_axis(e_rotm);
    // 获取旋转向量（旋转轴 * 旋转角度）
    Eigen::Vector3d rotation_vector = angle_axis.angle() * angle_axis.axis();
    e.tail<3>() = rotation_vector;
    e_dot = eef_vel;
    Eigen::VectorXd MA = clipped_tcp_force - adm_k.cwiseProduct(e) - adm_d.cwiseProduct(e_dot);
    Eigen::VectorXd adm_acc = MA.cwiseQuotient(adm_m);
    Eigen::VectorXd adm_vel = eef_vel + adm_acc * T;
    linear_disp = adm_vel.head(3) * T;
    angular_disp = eef_rotm * adm_vel.tail(3) * T; 
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
        // robot.servo_p(&new_pos, ABS, loop_rate);
        std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "linear search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }

}
void RobotAdmittanceControl::screw_assembly_directly(){

    int input;
    std::cout << "Enter 1 for continue" << std::endl;
    std::cin >> input;

    robot.servo_move_enable(true);
    //直线搜索***********************************************************************************
    wish_force << 0, 0, -4, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量

    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    int item = 0;
    real_robot_control::screwGoal goal;
    //实验测试用，用完注释
    // goal.num = 5;
    // client.sendGoal(goal,
    //             std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
    //             std::bind(&RobotAdmittanceControl::active_cb, this),
    //             std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  

    while (item < 7000)
    {   
        auto start_time = std::chrono::high_resolution_clock::now();
        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();  //更新机器人状态

        item = item + 1;

        eef_pos_d = eef_pos; //期望的位置不断更新，始终保持是当前状态，期望的位姿保持不变
        eef_rotm_d = eef_rotm;

        if (tcp_force[2] > 3){
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

        f.X = tcp_force[0];
        f.Y = tcp_force[1];
        f.Z = tcp_force[2];
        f.MX = tcp_force[3];
        f.MY = tcp_force[4];
        f.MZ = tcp_force[5];
        for_pub.publish(f);

        p.X = eef_pos[0];
        p.Y = eef_pos[1];
        p.Z = eef_pos[2];
        p.theta = 0;
        pos_pub.publish(p);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "linear search item" << item << " excution time is"<< duration.count()<<"ms" << endl;


    }
    std::cout << "Enter 1 for continue" << std::endl;
    std::cin >> input;
    
 
    //姿态搜索***********************************************************************************
    cout << "-----------screwing------------" << endl;
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    wish_force << 0, 0, -4, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量，表示只控制y轴
    adm_k << 700.0, 700.0, 1300.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 4 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3.5 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[1] = 3.5 * sqrt(adm_m[1] * adm_k[1]);
    adm_d[2] = 3.5 * sqrt(adm_m[2] * adm_k[2]);
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;
    Eigen::Vector3d init_height = {0.0, 0.0, 0.0};
    Eigen::Vector3d end_height = {0.0, 0.0, 0.0};
    int flag = 1; //初始化，一开始是搜索状态
    screw_execute_status = 2; //初始化，肯定是未运行的；
    new_rpy.rx = current_rpy.rx; 
    new_rpy.ry = current_rpy.ry;
    screw_execute_result = 9; //不需要对其进行初始化，一开始他没有结果
    

    int N = 8;
    int phi_index = 0;
    int theta_index = 1;
    double distance_threhold = 0.8;  // 单位 mm 
    while (true){
        // cout << "init_height - end_height = " << init_height - end_height << endl;
        // cout << "init_height - end_height. transpose = " << eef_rotm.transpose() * (init_height - end_height)  << endl;
        Eigen::Vector3d delta_height = eef_rotm.transpose() * (init_height - end_height);
        cout << "delta_height[2] = " << delta_height[2] << endl;
        // 判断是否要执行下一个期望位姿
        if (screw_execute_result == 2){ 
            cout << "装配完成，退出" << endl;
            // 装配完成
            flag = 3;
            break;
        }

        else if ((screw_execute_result == 0) && (delta_height[2]>(distance_threhold /1000))){
            cout << "flag = 0, 对准成功，进入下一步装配" << endl;
            flag = 0;
            // 对准成功，进入下一步装配，完全旋拧
        }

        else if (screw_execute_result == 1){
            cout << "装配完成，退出" << endl;
            // 装配完成
            flag = 3;
            // 对准成功，进入下一步装配，完全旋拧
            break;
        }


        item = 0;
        screw_execute_status = 2; //每一次搜索，都把状态置为2，表示还没开始运行
        cout << "开始运行" << endl;
        update_robot_state();
        get_eef_pose();
        eef_pos_d = eef_pos;
        while (item < 50000)
        {  
            if (screw_execute_status == 2)
            {   // 判断是否达到了期望位姿，如果达到了，并且此时执行器不是在运行的状态，就发布信息让执行器转动，一次循环只发送一次
                // item_flag = 1; //表示已经开始执行了
                cout << "\r" <<" start to send message " << flush;

                update_robot_state();
                get_eef_pose(); 
                if (flag == 0){
                    goal.num = 0; // 直接运行后续装配过程
                }
                else if(flag == 1){
                    goal.num = 1; // 运行第一阶段
                }
                client.sendGoal(goal,
                                std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                                std::bind(&RobotAdmittanceControl::active_cb, this),
                                std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  
            }
            // ros::spinOnce();
            if (screw_execute_status == 0){ //screw_execute_status = 0 表示运行结束了
                // 执行器运行结束了，可以切换了
                cout << " screw tool excution finish! break! " << endl;
                end_height = eef_pos; //记录结束的位置
                break;
            }

            if ((screw_execute_status == 1) && (item == 120)){
                init_height = eef_pos; //记录一下一开始的位置
                // temp_flag = 0;
                cout << " temp_flag = 0" << endl;
            }
                
            auto start_time = std::chrono::high_resolution_clock::now();
            eef_pos_d[2] = eef_pos[2];
            eef_rotm_d = eef_rotm;

            item = item + 1;

            // 导纳控制的范畴
            update_robot_state();
            get_tcp_force();
            get_eef_pose();

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
            new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
            // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
            robot.servo_p(&new_pos, ABS, loop_rate);
            std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

            f.X = tcp_force[0];
            f.Y = tcp_force[1];
            f.Z = tcp_force[2];
            f.MX = tcp_force[3];
            f.MY = tcp_force[4];
            f.MZ = tcp_force[5];
            for_pub.publish(f);

            p.X = eef_pos[0];
            p.Y = eef_pos[1];
            p.Z = eef_pos[2];
            p.theta = 0;
            pos_pub.publish(p);

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
            if ( item % 500 == 0 ){
                cout << "insertion item" << item << " excution time is"<< duration.count()<<"ms" << endl;
            }
            ros::spinOnce();
        }
    }
    

    // 机器人回退
    item = 0;
    while (item < 300) //100mm
    {  
        item = item + 1;
        update_robot_state();
        get_eef_pose();
        // linear_disp<< -0.002, 0, 0;
        linear_disp<< 0, 0, 0.001;
        new_linear_eef = eef_pos + eef_rotm * linear_disp;
        new_rotm_eef = eef_rotm;
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, 2);
        cout << "\r" << "final item = " << item << flush;
    }

    sleep(1);
    if(flag == 3){
        goal.num = 4; // 旋拧口回转，结束
    }

    client.sendGoal(goal,
                    std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&RobotAdmittanceControl::active_cb, this),
                    std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  
    
    while (ros::ok()){
        ros::spinOnce(); //处理一下上面的回调函数
        if (screw_execute_result == 4){
            break;
        }
    }
}
void RobotAdmittanceControl::screw_assembly_search(){

    char choice_tcp;

    adm_m << 3, 3, 3, 2, 2, 2;
    adm_k << 700.0, 700.0, 1200.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 4 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3.5 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[1] = 3.5 * sqrt(adm_m[1] * adm_k[1]);
    // adm_d[2] = 3.5 * sqrt(adm_m[1] * adm_k[1]);
    adm_d[2] = 2.8 * sqrt(adm_m[2] * adm_k[2]);

    std::cout << "Enter 1-3 to select the tcp:" << std::endl;
    std::cout << "1--> M12 六角头 || 2--> 3分螺母 || 3--> 3分三通" << std::endl;
    std::cin >> choice_tcp;
        
    switch(choice_tcp) {

        case '1':
            
            object_length << 0, 0, 0.023;   //M12 六角头螺丝
            break; 

        case '2':
            object_length << 0, 0, 0.016;   //3分螺母
            break; 
        
        case '3':
            
            object_length << 0, 0, 0.021;  //三通，3分
            break; 
        
        case '4':
            
            object_length << 0, 0, 0.006;  //水管
            break; 
        
        case '5':
            
            object_length << 0, 0, 0.046;  //圆形螺丝
            break; 

        case '6':
            object_length << 0, 0, 0.027;   //3分螺母 加长
            break; 
            
        default:
            // 程序结束
            std::cout << "task stoped" << std::endl;
        }

    eef_offset = eef_offset_basic + object_length;
    cout << "eef_offset" << eef_offset << endl;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + object_length; //更新tcp

    int input;
    std::cout << "Enter 1 for continue" << std::endl;
    std::cin >> input;

    robot.servo_move_enable(true);
    //直线搜索***********************************************************************************
    wish_force << 0, 0, -4, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量


    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    int item = 0;

    while (item < 7000)
    {   
        auto start_time = std::chrono::high_resolution_clock::now();
        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();  //更新机器人状态

        item = item + 1;

        eef_pos_d = eef_pos; //期望的位置不断更新，始终保持是当前状态，期望的位姿保持不变
        eef_rotm_d = eef_rotm;

        if (tcp_force[2] > 3){
            cout << "-----------linear search stopped------------" << endl;
            sleep(1);
            break;
        }

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
        // cout<<"new_trans" << new_pos.tran.x << " " << new_pos.tran.y << " "<< new_pos.tran.z << " "<< endl;
        // cout <<"new_rpy" << (new_pos.rpy.rx / PI) * 180 << "  " << (new_pos.rpy.ry / PI) * 180<< "  " << (new_pos.rpy.rz / PI) * 180<<"  " << endl; //new_rpy.rx不受导纳控制输出的影响，一开始就写死了
        robot.servo_p(&new_pos, ABS, loop_rate);
        // std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

        f.X = tcp_force[0];
        f.Y = tcp_force[1];
        f.Z = tcp_force[2];
        f.MX = tcp_force[3];
        f.MY = tcp_force[4];
        f.MZ = tcp_force[5];
        for_pub.publish(f);

        p.X = eef_pos[0];
        p.Y = eef_pos[1];
        p.Z = eef_pos[2];
        p.theta = 0;
        pos_pub.publish(p);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        if (item % 130 == 0){
            cout << "linear search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
        }
    }
    std::cout << "Enter 1 for continue" << std::endl;
    std::cin >> input;
    //**********************************曲线搜索*************************************************
    cout << "-----------spiral search started------------" << endl;
    
    update_robot_state();
    get_tcp_force();
    get_eef_pose();


    wish_force << 0, 0, -5, 0, 0, 0;  //期望力
    selection_vector<<0, 0, 1, 0, 0, 0; //选择向量，表示只控制z轴

    Eigen::Matrix3d init_eef_rotm = eef_rotm;
    Eigen::Vector3d init_eef_pos = eef_pos;

    Eigen::Vector3d traj; //轨迹的向量表示
    item = 0;
    double d = 0.2; //最大间隙
    double a = d / 2;
    double b = d / (2*PI); //毫米单位
    double h = d / 2.6;
    double theta = 0;
    double radius = a + b * theta;

    std::array<int, 42> point_numbers = {17, 32, 49, 65, 82, 98, 114, 131, 147, 163, 
                                        180, 196, 212, 229, 245, 261, 278, 294, 310, 327, 
                                        343, 359, 376, 392, 408, 425, 441, 458, 473, 
                                        490, 507, 522, 540, 555, 572, 588, 604, 621,
                                         637, 654, 669, 686}; //h=d/2.6的情况下用python脚本离线生成，36圈
    int circle_count = 0;

    phi_changed = 0; //初始化

    int temp_item = 0;
    int record_item;
    while (item < 7000)
    {   
        if (phi_changed){
            double theta_2 = 0.8 * PI / 180;
            int point_num = point_numbers[circle_count];
            std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(point_num, theta_2);
            if (adjust_phi>(point_num/2)){  //calculateRotationMatrices从-x处出发，逆时针，而机器人运动是从+x出发，逆时针，所以要进行变换
                adjust_phi = adjust_phi - (point_num/2);
            }
            else{
                adjust_phi = adjust_phi + (point_num/2);
            }
            eef_rotm_d =  eef_rotm * rotationMatrices[adjust_phi]; //期望的位置是当前位置，期望的位姿是经过旋转变换以后的位姿，
            eef_rotm_d_modified = eef_rotm_d;
            get_new_link6_pose(eef_pos_d, eef_rotm_d); // 转化期望的位姿到link6
            new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
            new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
            new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
            robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
            cout << "---------------phi_changed---------------------" << endl;
            cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;
            phi_changed = 0;
        }
        auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        item = item + 1;

        // 判断机器人状态
        update_robot_state();
        get_tcp_force();
        get_eef_pose();

        // p.X = eef_pos[0];
        // p.Y = eef_pos[1];
        // p.Z = eef_pos[2];
        // p.X = x * 1000;
        // p.Y = y * 1000;
        // p.Z = 0;
        // p.theta = theta;
        // pos_pub.publish(p);

        if ((abs(new_rpy.rx - current_rpy.rx)>0.003) || (abs(new_rpy.ry - current_rpy.ry)>0.003)) {
            int local_item = 0;
            while (true){
                local_item = local_item + 1;
                update_robot_state();
                get_tcp_force();
                get_eef_pose();

                eef_pos_d = eef_pos;
                eef_rotm_d = eef_rotm;
                tcp_admittance_control();
                linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
                angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
                //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
                linear_disp_clipped = linear_disp_clipped + traj;
                new_linear_eef = init_eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
                //fixed rotation
                new_rotm_eef = eef_rotm;
                //no-fixed rotation
                get_new_link6_pose(new_linear_eef, new_rotm_eef);
                new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
                new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz; //修改后的角度
                if (local_item == 1){
                    linear_disp_clipped << 0, 0, 0.001;
                    new_linear_eef = eef_pos + eef_rotm_d_modified * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
                    get_new_link6_pose(new_linear_eef, new_rotm_eef);
                    new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
                    robot.servo_p(&new_pos, ABS, 1);
                    cout << tcp_force[2] <<" sleep "<< endl;
                    sleep(1);
                    cout << "jumped" << endl;
                    // new_pos.tran.z = new_linear[2] * 1000 + 0.1;
                    // robot.servo_p(&new_pos, ABS, 1);
                }
                else{
                    robot.servo_p(&new_pos, ABS, 1);
                }
                // 
                if ((tcp_force[2] > 2) && (local_item>220)){
                    // sleep(1);
                    break;
                }
            }
        }

        else{
            // 生成平面螺旋曲线
            
            // theta = (2 * PI / point_num) + theta; //螺旋线的角度
            if (item == 0){
                radius = a + b * theta;
            }
            else{
                theta = theta + 2 * asin(h / (2 * radius)); //螺旋线的角度
                radius = a + b * theta;
            }

            double x = radius * cos(theta) * 0.001;
            double y = radius * sin(theta) * 0.001; // 此处缩放到米为单位，以便于与导纳控制输出对其
            double z = 0;
            traj << x, y, z;
            if (theta >= 2 * PI * (circle_count + 1)){
                circle_count = circle_count + 1;
                cout << "circle_count = " << circle_count << endl;
            }


            if (tcp_force[2] < 0.7){
                cout << "force" <<  tcp_force[2] << endl;
                cout << "spiral search stoped" <<endl;  //实际实验时进行调试
                break;
            }
            if ((abs(tcp_force[0]) > 6) || (abs(tcp_force[1]) > 6)){
                cout << "x_force" <<  tcp_force[0] << ", y_force" <<  tcp_force[1]<< endl;
                cout << "spiral search stoped" <<endl;  //实际实验时进行调试
                break;
            }
            fp.FX =  tcp_force[0];
            fp.FY =  tcp_force[1];
            fp.FZ =  tcp_force[2];
            fp.X = x*1000;  //发布毫米
            fp.Y = y*1000;
            fp.Z = z*1000;
            fp.theta = theta;
            for_pos_pub.publish(fp);
        }

        // cout << "force" <<  tcp_force[2] << endl;
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        linear_disp_clipped = linear_disp_clipped + traj;
        new_linear_eef = init_eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
        
        // x_robot_vec.push_back(new_linear_eef[0]);
        // y_robot_vec.push_back(new_linear_eef[1]);
        //fixed rotation
        new_rotm_eef = eef_rotm;

        //no-fixed rotation
        // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;

        // cout<<"new_trans" << new_pos.tran.x << " " << new_pos.tran.y << " "<< new_pos.tran.z << " "<< endl;
        // cout <<"new_rpy" << (new_pos.rpy.rx / PI) * 180 << "  " << (new_pos.rpy.ry / PI) * 180<< "  " << (new_pos.rpy.rz / PI) * 180<<"  " << endl;

        robot.servo_p(&new_pos, ABS, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); 

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        if (item % 130 == 0){
            cout << "spiral search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
        }
        ros::spinOnce();
    }



    std::cout << "Enter 1 for continue" << std::endl;
    std::cin >> input;
    //姿态搜索***********************************************************************************
    cout << "-----------pose search------------" << endl;
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    wish_force << 0, 0, -4, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量，表示只控制y轴
    adm_k << 700.0, 700.0, 1300.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 4 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3.5 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[1] = 3.5 * sqrt(adm_m[1] * adm_k[1]);
    adm_d[2] = 3.5 * sqrt(adm_m[2] * adm_k[2]);
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;
    Eigen::Vector3d init_height = {0.0, 0.0, 0.0};
    Eigen::Vector3d end_height = {0.0, 0.0, 0.0};
    int flag = 1; //初始化，一开始是搜索状态
    screw_execute_status = 2; //初始化，肯定是未运行的；
    new_rpy.rx = current_rpy.rx; 
    new_rpy.ry = current_rpy.ry;
    screw_execute_result = 9; //不需要对其进行初始化，一开始他没有结果
    real_robot_control::screwGoal goal;

    int N = 6;
    int phi_index = 0;
    int theta_index = 1;
    double distance_threhold = 1;  // 单位 mm 
    while (true){
        // cout << "init_height - end_height = " << init_height - end_height << endl;
        // cout << "init_height - end_height. transpose = " << eef_rotm.transpose() * (init_height - end_height)  << endl;
        Eigen::Vector3d delta_height = eef_rotm.transpose() * (init_height - end_height);
        // 判断是否要执行下一个期望位姿
        if (screw_execute_result == 2){ 
            cout << "装配完成，退出" << endl;
            // 装配完成
            flag = 3;
            break;
        }

        else if ((screw_execute_result == 0) && (delta_height[2]>(distance_threhold /1000))){
            cout << "delta_height[2] = " << delta_height[2] << endl;
            cout << "flag = 0, 对准成功，进入下一步装配" << endl;
            flag = 0;
            // 对准成功，进入下一步装配，完全旋拧
        }
        else if ((screw_execute_result == 0) && (delta_height[2]<(distance_threhold /1000))){
            cout << "delta_height[2] = " << delta_height[2] << endl;
            cout << "flag = 1, 对准失败，但是也没有卡住，进入下一个搜索点" << endl;
            flag = 1;

            cout << "[********]theta_index=" << theta_index << ", phi_index="<< phi_index << endl;
            cout << "[********]screw_execute_result = " << screw_execute_result<< endl;

            if (theta_index<3){
                if (phi_index==N){
                    phi_index = 0; //phi 循环了一轮以后，重新重置phi
                    theta_index++; //更新theta
                }
            }
            else if (theta_index == 3){
                break;
            }
            
            // 对准失败，但是也没有卡住，进入下一个搜索点
            // ruled-based rotation 更新期望姿态
            double theta_2 = theta_index * 1 * PI / 180;
            std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(N, theta_2);
            eef_rotm_d =  eef_rotm * rotationMatrices[phi_index]; //期望的位置是当前位置，期望的位姿是经过旋转变换以后的位姿

            get_new_link6_pose(eef_pos_d, eef_rotm_d); // 转化期望的位姿到link6
            new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
            new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
            new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
            robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
            cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;
            //*********************
            phi_index++;
        }
        else if (screw_execute_result == 1){
            cout << "flag = 2, 卡住了，回退，进入下一个搜索" << endl;
            flag = 2;
            // 卡住了，回退，进入下一个搜索
        }

        item = 0;
        screw_execute_status = 2; //每一次搜索，都把状态置为2，表示还没开始运行
        // int temp_flag = 1; //记录一开始高度的标志位
        cout << "开始运行" << endl;
        update_robot_state();
        get_eef_pose();
        eef_pos_d = eef_pos;
        while (item < 5000)
        {  
            if ((abs(new_rpy.rx - current_rpy.rx)<0.001) && (abs(new_rpy.ry - current_rpy.ry)<0.001) && (screw_execute_status == 2))
            {   // 判断是否达到了期望位姿，如果达到了，并且此时执行器不是在运行的状态，就发布信息让执行器转动，一次循环只发送一次
                // item_flag = 1; //表示已经开始执行了
                cout << "\r" <<" start to send message " << flush;

                update_robot_state();
                get_eef_pose(); 
                if (flag == 0){
                    goal.num = 0; // 直接运行后续装配过程
                }
                else if(flag == 1){
                    goal.num = 1; // 运行第一阶段，接着搜索
                }
                else if(flag == 2){
                    goal.num = 2; // 出现错误，回退
                }
                client.sendGoal(goal,
                                std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                                std::bind(&RobotAdmittanceControl::active_cb, this),
                                std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  
            }

            if (screw_execute_status == 0){ //screw_execute_status = 0 表示运行结束了
                // 执行器运行结束了，可以切换了
                cout << " screw tool excution finish! break! " << endl;
                end_height = eef_pos; //记录结束的位置
                break;
            }

            if ((screw_execute_status == 1) && (item == 180)){
                init_height = eef_pos; //记录一下一开始的位置
                // temp_flag = 0;
                cout << " temp_flag = 0" << endl;
            }
                
            auto start_time = std::chrono::high_resolution_clock::now();
            eef_pos_d[2] = eef_pos[2];
            eef_rotm_d = eef_rotm;

            item = item + 1;

            // 导纳控制的范畴
            update_robot_state();
            get_tcp_force();
            get_eef_pose();

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
            new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
            // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
            robot.servo_p(&new_pos, ABS, loop_rate);
            std::this_thread::sleep_for(std::chrono::milliseconds(8)); 

            f.X = tcp_force[0];
            f.Y = tcp_force[1];
            f.Z = tcp_force[2];
            f.MX = tcp_force[3];
            f.MY = tcp_force[4];
            f.MZ = tcp_force[5];
            for_pub.publish(f);

            p.X = eef_pos[0];
            p.Y = eef_pos[1];
            p.Z = eef_pos[2];
            p.theta = 0;
            pos_pub.publish(p);

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
            if ( item % 500 == 0 ){
                cout << "insertion item" << item << " excution time is"<< duration.count()<<"ms" << endl;
            }
            ros::spinOnce();
        }
    }
    

    if(flag == 3){
        goal.num = 3; // 打开夹爪，结束
    }

    client.sendGoal(goal,
                    std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&RobotAdmittanceControl::active_cb, this),
                    std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  
    
    while (ros::ok()){
        ros::spinOnce(); //处理一下上面的回调函数
        if (screw_execute_result == 3){
            break;
        }
    }
    
    // 机器人回退
    item = 0;
    while (item < 300) //100mm
    {  
        item = item + 1;
        update_robot_state();
        get_eef_pose();
        // linear_disp<< -0.002, 0, 0;
        linear_disp<< 0, 0, 0.001;
        new_linear_eef = eef_pos + eef_rotm * linear_disp;
        new_rotm_eef = eef_rotm;
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, 4);
        cout << "\r" << "final item = " << item << flush;
    }

    sleep(1);
    if(flag == 3){
        goal.num = 4; // 旋拧口回转，结束
    }

    client.sendGoal(goal,
                    std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                    std::bind(&RobotAdmittanceControl::active_cb, this),
                    std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  
    
    while (ros::ok()){
        ros::spinOnce(); //处理一下上面的回调函数
        if (screw_execute_result == 4){
            break;
        }
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
    char input;

    std::cout << "Enter 1-3 to select the goal position:" << std::endl;
    std::cin >> input;
    
    // switch(input) { //单臂实验标定结果

    //     case '1':
    //         goal_pose.tran.x = -152.571; goal_pose.tran.y = 342.299; goal_pose.tran.z = 264.209;
    //         goal_pose.rpy.rx = (180 * PI) / 180; goal_pose.rpy.ry = (0 * PI) / 180; goal_pose.rpy.rz = (-10 * PI) / 180; //m12
    //         robot.servo_move_enable(false);
    //         robot.linear_move(&goal_pose, ABS, TRUE, 18);
    //         break; // 添加break语句

    //     default:
    //         // 程序结束
    //         std::cout << "task stoped" << std::endl;
    //     }

    switch(input) {  //双臂实验标定结果

        case '1':
            goal_pose.tran.x = -76.022; goal_pose.tran.y = 429.777; goal_pose.tran.z = 195.261;
            goal_pose.rpy.rx = (176.795 * PI) / 180; goal_pose.rpy.ry = (0.178 * PI) / 180; goal_pose.rpy.rz = (-0.168 * PI) / 180; //m12
            robot.servo_move_enable(false);
            robot.linear_move(&goal_pose, ABS, TRUE, 18);
            break; // 添加break语句

        case '2':
            goal_pose.tran.x = -72.949; goal_pose.tran.y = 442.044; goal_pose.tran.z = 192.941;
            goal_pose.rpy.rx = (175.927 * PI) / 180; goal_pose.rpy.ry = (1.348 * PI) / 180; goal_pose.rpy.rz = (-0.303 * PI) / 180;
            robot.servo_move_enable(false);
            robot.linear_move(&goal_pose, ABS, TRUE, 18);
            break; // 添加break语句
        
        case '3':
            goal_pose.tran.x = 235.13; goal_pose.tran.y = 406.882; goal_pose.tran.z = 182;
            goal_pose.rpy.rx = (177.815 * PI) / 180; goal_pose.rpy.ry = (1.249 * PI) / 180; goal_pose.rpy.rz = (-53.967 * PI) / 180;
            robot.servo_move_enable(false);
            robot.linear_move(&goal_pose, ABS, TRUE, 18);
            break; // 添加break语句
        
        case '4':
            goal_pose.tran.x = -110.169; goal_pose.tran.y = 415.724; goal_pose.tran.z = 283.141;
            goal_pose.rpy.rx = (174.870 * PI) / 180; goal_pose.rpy.ry = (30.874 * PI) / 180; goal_pose.rpy.rz = (-73.671 * PI) / 180; //m12
            robot.servo_move_enable(false);
            robot.linear_move(&goal_pose, ABS, TRUE, 18);
            break; // 交接水管
        
        case '5':
            goal_pose.tran.x = -72.817; goal_pose.tran.y = 343.633; goal_pose.tran.z = 213.059;
            goal_pose.rpy.rx = (180.0 * PI) / 180; goal_pose.rpy.ry = (0 * PI) / 180; goal_pose.rpy.rz = (-60 * PI) / 180; //m12
            robot.servo_move_enable(false);
            robot.linear_move(&goal_pose, ABS, TRUE, 18);
            break; // 配合其他工具宁螺丝
        
        default:
            // 程序结束
            std::cout << "task stoped" << std::endl;
        }


    update_robot_state();
    get_eef_pose();
    // int N = 6;
    // double theta = -3 * PI / 180;
    // std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(N, theta);
    // // cout << rotationMatrices[1] << endl;
    // eef_rotm =  eef_rotm * rotationMatrices[0];
    // cout << eef_rotm << endl;
    // get_new_link6_pose(eef_pos, eef_rotm);
    // eigen_rpy = new_angular.eulerAngles(2,1,0);
    // cout <<"rpy" << eigen_rpy << endl;
    // new_rpy.rx = eigen_rpy[2]; new_rpy.ry = eigen_rpy[1]; new_rpy.rz = eigen_rpy[0]; 
    // robot.servo_move_enable(true);
    eef_pos_d = eef_pos;

    double theta_2 =  2 * PI / 180;
    std::vector<Eigen::Matrix3d> rotationMatrices = calculateRotationMatrices(4, theta_2);
    eef_rotm_d =  eef_rotm * rotationMatrices[2]; //期望的位置是当前位置，期望的位姿是经过旋转变换以后的位姿

    get_new_link6_pose(eef_pos_d, eef_rotm_d); // 转化期望的位姿到link6
    new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
    new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
    new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
    robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
    cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;

    // new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(0,1); new_rotm.z.x = new_angular(0,2);
    // new_rotm.x.y = new_angular(1,0); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(1,2);
    // new_rotm.x.z = new_angular(2,0); new_rotm.y.z = new_angular(2,1); new_rotm.z.z = new_angular(2,2);
    // robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角

    // cout << "new_linear" << new_linear << endl;
    // cout << new_rpy.rx << " "<<new_rpy.ry << " "<<new_rpy.rz << " "<<endl;
    goal_pose.tran.x = new_linear[0] * 1000; goal_pose.tran.y = new_linear[1]* 1000; goal_pose.tran.z = new_linear[2]* 1000;
    goal_pose.rpy.rx = new_rpy.rx; goal_pose.rpy.ry = new_rpy.ry; goal_pose.rpy.rz = new_rpy.rz;
    // // cout << goal_pose << endl;
    // robot.linear_move(&goal_pose, ABS, TRUE, 12);
    // robot.servo_p(&goal_pose, ABS, 2);
    // sleep(3);
    cout << "the robot is at the goal pose" << endl;

}

void RobotAdmittanceControl::reset(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    cout<< "reset the robot"<<endl;
    // JointValue right_joint_pos = { -PI/3, PI/3, 2*PI/3, PI/2, -PI/2, PI/2 };  //标准实验，双臂协作时的零点
    // robot.joint_move(&right_joint_pos, ABS, true, 0.15);

    JointValue left_joint_pos = { -60.283 * PI / 180, 74.928 * PI / 180, 132.499 * PI / 180, 62.573 * PI / 180, -90 * PI / 180, 39.717 * PI / 180 }; //只做单臂实验的领零点
    robot.joint_move(&left_joint_pos, ABS, true, 0.1);
    cout<< "the robot was resetted"<<endl;
}

void RobotAdmittanceControl::test(){
    // ef.width_reduce_or_increase_full(-1);
    // ef.width_recovery();

}

int main(int argc, char** argv){
    ros::init(argc, argv, "real_robot_control");
    cout << "ros init" << endl;

    RobotAdmittanceControl robot_control;
    // robot_control.ros_init(argc, argv);
    // robot_control.reset();
    robot_control.update_robot_state();
    // robot_control.get_world_force();
    robot_control.get_eef_pose();

    
    // robot_control.screw_assembly_search();
    bool running = true;
    char input;

    while (running) {
        std::cout << "1--> test program || 2--> go to reset pose " << std::endl;
        std::cout << "3--> go to goal pose || 4--> screw action " << std::endl;
        std::cin >> input;
        switch(input) {
            case '1':

                std::cout << "test program" << std::endl;
                robot_control.go_to_pose();
                robot_control.tcp_admittance_run();
                break;

            case '2':

                std::cout << "go to reset pose" << std::endl;
                // robot_control.tcp_admittance_run();
                robot_control.reset();
                break;

            case '3':

                std::cout << "go to goal pose" << std::endl;
                robot_control.go_to_pose();
                break;
            
            case '4':

                std::cout << "screw action" << std::endl;
                robot_control.screw_assembly_search();
                break;

            case '5':

                std::cout << "screw action" << std::endl;
                robot_control.screw_assembly_directly();
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

