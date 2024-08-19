#include <chrono>
#include "real_robot_control/left_robot_control.h"
#include <random>

using namespace std;
using namespace Eigen;

RobotAdmittanceControl::RobotAdmittanceControl():
    nh(std::make_shared<ros::NodeHandle>()),
    client(*nh, "screwactions", true){
    
    for_pub = nh->advertise<real_robot_control::force_pub>("robot_force", 10);
    
    client.waitForServer();

    selection_vector.resize(6);
    selection_vector<<1, 1, 1, 0, 0, 0;
    // 质量，刚度，阻尼
    adm_m.resize(6);
    adm_k.resize(6);
    adm_d.resize(6);
    wish_force.resize(6);
    wish_force << 0 ,0, 0, 0, 0, 0;


    adm_m << 3, 3, 4, 0.5, 0.5, 1.0;
    adm_k << 600.0, 600.0, 1200.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 3 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[1] = 3 * sqrt(adm_m[1] * adm_k[1]);
    // adm_d[2] = 300;
    cout << "k" << adm_k[0] <<endl;
    cout << "d" << adm_d[0]<<endl;

    // 初始化机器人
    // robot.login_in("192.168.3.200"); 
    // robot.power_on();
    // robot.enable_robot();
    // robot.set_tool_id(0);
    // // reset();
    // // go_to_pose();
    // // robot.servo_speed_foresight(15, 0.03);
    // robot.servo_move_use_carte_NLF(50, 200, 800, 30, 60, 100);
    // robot.servo_move_enable(TRUE);
    // robot.set_torque_sensor_mode(1);
    // robot.set_compliant_type(1, 0);
    // sleep(1);
    // robot.set_compliant_type(0,0);

    eef_offset << 0, 0, 0.1955 + 0.037;
    eef_offset_rotm = Eigen::AngleAxisd(PI / 4, Eigen::Vector3d::UnitZ());

    eef_offset_to_sensor << 0, 0, 0.146 + 0.037;
    eef_offset_rotm_to_sensor = Eigen::AngleAxisd(PI / 4, Eigen::Vector3d::UnitZ());

}

RobotAdmittanceControl::~RobotAdmittanceControl(){
    // if (excution_thread.joinable()) {
    //     excution_thread.join();
    // }
    // if (sensor_thread.joinable()) {
    //     sensor_thread.join();
    // }
}

void RobotAdmittanceControl::get_eef_pose(){
    // 位置赋值
    link6_pos[0] = status.cartesiantran_position[0]/1000;
    link6_pos[1] = status.cartesiantran_position[1]/1000;
    link6_pos[2] = status.cartesiantran_position[2]/1000; //转化为米单位
    // 姿态欧拉角赋值
    current_rpy.rx = status.cartesiantran_position[3];
    current_rpy.ry = status.cartesiantran_position[4];
    current_rpy.rz = status.cartesiantran_position[5];
    // cout << link6_pos[0] << link6_pos[1] << link6_pos[2] << endl;
    // cout << current_rpy.rx << ","<< current_rpy.ry << ","<< current_rpy.rz << endl;
    // 欧拉角转旋转矩阵，赋值
    // robot.rpy_to_rot_matrix(&current_rpy, &current_rotm);
    // link6_rotm << current_rotm.x.x, current_rotm.y.x, current_rotm.z.x,
    //               current_rotm.x.y, current_rotm.y.y, current_rotm.z.y,
    //               current_rotm.x.z, current_rotm.y.z, current_rotm.z.z;
    link6_rotm = Eigen::AngleAxisd(current_rpy.rz, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(current_rpy.ry, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(current_rpy.rx, Eigen::Vector3d::UnitX());
    eef_rotm =  link6_rotm * eef_offset_rotm;
    Eigen::Vector3d euler_angles = eef_rotm.eulerAngles(2, 1, 0); // rz ry rx
    eef_pos = link6_pos + eef_rotm * (eef_offset_rotm.transpose() * eef_offset);
}
 
void RobotAdmittanceControl::get_new_link6_pose(const Eigen::Vector3d& new_linear_eef, const Eigen::Matrix3d& new_angular_eef){
        // left_link6_pos = left_eef_pos - left_eef_rotm @ (self.left_eef_offset_rotm.T @ self.left_eef_offset)
        // left_link6_rotm = left_eef_rotm @ self.left_eef_offset_rotm.T
        new_linear = new_linear_eef - new_angular_eef * (eef_offset_rotm.transpose() * eef_offset);
        new_angular = new_angular_eef * eef_offset_rotm.transpose();
}

void RobotAdmittanceControl::updata_rotation(const Eigen::Matrix3d& current_rotm, const Eigen::Vector3d& angluar_disp, Eigen::Matrix3d& new_orientation){

    Eigen::AngleAxisd delta_rotation(angluar_disp.norm(), angluar_disp.normalized());
    // 更新当前旋转矩阵
    new_orientation = delta_rotation.toRotationMatrix() *  current_rotm;
    // cout<< current_rotm <<endl;
    // cout<< new_orientation <<endl;
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
    robot.rpy_to_rot_matrix(&current_rpy, &current_rotm);
    link6_rotm << current_rotm.x.x, current_rotm.y.x, current_rotm.z.x,
                  current_rotm.x.y, current_rotm.y.y, current_rotm.z.y,
                  current_rotm.x.z, current_rotm.y.z, current_rotm.z.z;
    
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

    tcp_force.head<3>() = eef_offset_rotm_to_sensor.transpose() * local_force.head<3>();
    tcp_force.tail<3>() = -eef_offset_to_sensor.cross(tcp_force.head<3>()) + eef_offset_rotm_to_sensor.transpose() * local_force.tail<3>();

    f.X = tcp_force[0];
    f.Y = tcp_force[1];
    f.Z = tcp_force[2];
    f.MX = tcp_force[3];
    f.MY = tcp_force[4];
    f.MZ = tcp_force[5];
    for_pub.publish(f);
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

void RobotAdmittanceControl::tcp_admittance_control(){
 
        // 使用Eigen数组操作进行clip
        clipped_tcp_force = tcp_force.array().min(upper).max(lower);
        clipped_tcp_force = wish_force + clipped_tcp_force;
        e.head<3>() = eef_rotm.transpose() * (eef_pos - eef_pos_d); //将基坐标系下的位置偏移转化为tcp坐标系下的偏移，后续所有计算都是在当前时刻的tcp坐标系下，计算下一时刻的数值
        Eigen::Matrix3d e_rotm = eef_rotm_d.transpose() * eef_rotm;

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
    // get_world_force();
    get_tcp_force();
    get_eef_pose();
    tcp_admittance_control();
    
    linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
    angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01);
    new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped;
    //fixed rotation
    new_rotm_eef = eef_rotm;

    //no-fixed rotation

    // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);

    get_new_link6_pose(new_linear_eef, new_rotm_eef);
    // Eigen::Vector3d eigen_rpy = new_angular.eulerAngles(2,1,0);
    // cout << eigen_rpy << endl;
    // new_rpy.rx = eigen_rpy[2]; new_rpy.ry = eigen_rpy[1]; new_rpy.rz = eigen_rpy[0]; 
    // new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(0,1); new_rotm.z.x = new_angular(0,2);
    // new_rotm.x.y = new_angular(1,0); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(1,2);
    // new_rotm.x.z = new_angular(2,0); new_rotm.y.z = new_angular(2,1); new_rotm.z.z = new_angular(2,2);
    // robot.rot_matrix_to_rpy(&new_rotm, &new_rpy);

    new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
    // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
    new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
    robot.servo_p(&new_pos, ABS, loop_rate);
}

void RobotAdmittanceControl::start(){
    // excution_loop();
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;
    // double init_z = eef_pos_d[2];
 
    
    int item = 0;
    while (item < 1000000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;
        // cout << "eef_d" << eef_pos_d << endl;
        // if (eef_pos_d[2] > init_z - 0.005){
        //     eef_pos_d[2] = eef_pos_d[2] - 0.0001;
            
        // }
        // else{
        //     eef_pos_d[2] = init_z - 0.009;
        // }
        
        item = item + 1;
        // eef_pos_d(1) -= 0.0000; //y方向1mm移动
        excution_calculation_loop();
        // sleep(0.1);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        cout << "item" << item << " excution time is"<< duration.count()<<"ms" << endl;
    }
    // reset();

}

void RobotAdmittanceControl::go_to_pose(){
    CartesianPose goal_pose;
    goal_pose.tran.x = -89.698; goal_pose.tran.y = 446.831; goal_pose.tran.z = 180;
    goal_pose.rpy.rx = (180 * PI) / 180; goal_pose.rpy.ry = (0* PI) / 180; goal_pose.rpy.rz = (-60 * PI) / 180;

    robot.linear_move(&goal_pose, ABS, TRUE, 12);
    sleep(6);
    cout << "the robot is at the goal pose" << endl;

}

void RobotAdmittanceControl::reset(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    cout<< "reset the robot"<<endl;
    JointValue left_joint_pos = { PI / 3, PI/2, 3 * PI / 4, 3 * PI / 4 , PI / 2, (135 * PI) / 180 };
    robot.joint_move(&left_joint_pos, ABS, true, 0.2);
    cout<< "the robot was resetted"<<endl;
}

void RobotAdmittanceControl::pick_up(){
    robot.servo_move_enable(TRUE);
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量
    wish_force << 0, -5, 0, 0, 0, 0;  //期望力
    adm_m << 3.5, 3, 3.5, 0.5, 0.5, 1.0;
    adm_k << 1200.0, 1100.0, 1200.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 3.5 * sqrt(adm_m[i] * adm_k[i]);
    }

    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    Eigen::Vector3d traj; //轨迹的向量表示
    Eigen::Vector3d init_eef_pos; 
    Eigen::Vector3d e_eef_pos; 
    init_eef_pos = eef_pos;
    

    int item = 0;
    while (item < 1000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;
        e_eef_pos = eef_rotm.transpose() * (eef_pos - init_eef_pos);

        if (abs(e_eef_pos[1]) > 0.02){
            cout << e_eef_pos[1] << endl;
            break;
        }
   
        item = item + 1;

        // excution_calculation_loop();

        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
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

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);

        if (item % 100 == 0){
        cout << "item" << item << " excution time is"<< duration.count()<<"ms" << endl;}
    }
    robot.servo_move_enable(false);
}
void RobotAdmittanceControl::back_to_middle(){
    int item = 0;
    update_robot_state();
    get_eef_pose();
    Eigen::Vector3d init_eef_pos; 
    Eigen::Vector3d e_eef_pos; 
    init_eef_pos = eef_pos;
    robot.servo_move_enable(true);
    while (item < 3000) //100mm
    {  
        if (item % 500 == 0){
             cout << "serving: " << item << endl;
        }
        item = item + 1;
        update_robot_state();
        get_eef_pose();

        e_eef_pos = eef_rotm.transpose() * (eef_pos - init_eef_pos);

        if (abs(e_eef_pos[2]) > 0.15){
            cout << e_eef_pos[2] << endl;
            cout << "\r" << "final item = " << item << endl;
            break;
        }
        linear_disp<< 0, 0, -0.002;
        new_linear_eef = eef_pos + eef_rotm * linear_disp;
        new_rotm_eef = eef_rotm;
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);
    }
    robot.servo_move_enable(false);
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
}

void RobotAdmittanceControl::pure_passive_model(){

    robot.servo_move_enable(TRUE);
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量
    wish_force << 0, 0, 0, 0, 0, 0;  //期望力
    adm_m << 3, 3, 3, 0.5, 0.5, 1.0;
    adm_k << 900.0, 900.0, 900.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 2.5 * sqrt(adm_m[i] * adm_k[i]);
    }

    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;
    // double x = 0; double y = 0; double z = 0;
    // Eigen::Vector3d traj; //轨迹的向量表示
    // Eigen::Vector3d init_eef_pos; 
    // Eigen::Vector3d e_eef_pos; 
    // init_eef_pos = eef_pos;

    screw_execute_result = 2;
    screw_execute_status = 0;
    
    int item = 0;
    while (item < 20000)
    {   auto start_time = std::chrono::high_resolution_clock::now();

        if ((screw_execute_result == 2) && (screw_execute_status == 0)){
            if (item % 200 == 0){
                 cout << "初次执行" << endl;
            }
           
            goal.num = 5;
            client.sendGoal(goal,
                            std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&RobotAdmittanceControl::active_cb, this),
                            std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));  
        }
        else if ((screw_execute_result == 0) && (screw_execute_status == 0)){
            cout << "执行成功" << endl;
            break;  
        }
        else if ((screw_execute_result == 1) && (screw_execute_status == 0)){
            
            cout << "执行成功失败，搜索" << endl;
            goal.num = 6;
            client.sendGoal(goal,
                            std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&RobotAdmittanceControl::active_cb, this),
                            std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 

            while (true){
                ros::spinOnce(); //等待打开程序执行完成
                if (screw_execute_status == 0){
                    cout << "screw_execute_status == 0, break" << endl;
                    break;
                }
            }
            // 使用随机设备生成随机数种子
            std::random_device rd;
            std::mt19937 gen(rd()); // Mersenne Twister 随机数生成器
            std::uniform_real_distribution<> dis(-3.0, 3.0); // 生成 0.0 到 1.0 之间的浮点数

            // 生成一个随机数
            double x = dis(gen) / 1000;
            double y = dis(gen) / 1000;
            double z = dis(gen) / 1000;

            Eigen::Vector3d search_distance;
            search_distance << x,y,z;

            update_robot_state(); //robot move
            get_eef_pose();
            new_linear_eef = eef_pos + eef_rotm * search_distance;
            new_rotm_eef = eef_rotm;
            get_new_link6_pose(new_linear_eef, new_rotm_eef);
            new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
            new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
            robot.servo_p(&new_pos, ABS, loop_rate);
        }

        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;
        // e_eef_pos = eef_rotm.transpose() * (eef_pos - init_eef_pos);

        // if (abs(e_eef_pos[1]) > 0.02){
        //     cout << e_eef_pos[1] << endl;
        //     break;
        // }
   
        item = item + 1;

        // excution_calculation_loop();

        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
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

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);

        if (item % 100 == 0){
        cout << "item" << item << " excution time is"<< duration.count()<<"ms" << endl;}

        ros::spinOnce();
    }
    robot.servo_move_enable(false);
}

void RobotAdmittanceControl::spiral_search(){

    robot.servo_move_enable(true);
    adm_m << 3, 4, 3, 0.5, 0.5, 1.0;
    adm_k << 600.0, 1100.0, 600.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 3 * sqrt(adm_m[i] * adm_k[i]);
    }
    //直线搜索***********************************************************************************
    wish_force << 0, 5, 0, 0, 0, 0;  //期望力
    selection_vector<<0, 1, 0, 0, 0, 0; //选择向量，表示只控制y轴

    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    int item = 0;

    while (item < 1500)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        item = item + 1;

        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        if (tcp_force[1] < -2){
            
            cout << "-----------linear search stopped------------" << endl;
            sleep(1);
            break;
        }
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。

        //fixed rotation
        new_rotm_eef = eef_rotm;

        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        if (item % 100 == 0){
            cout << "linear search item" << item << " excution time is"<< duration.count()<<"ms" << endl;
        }
        
    }

    //曲线搜索***********************************************************************************
    cout << "-----------spiral search started------------" << endl;
    update_robot_state();
    get_tcp_force();
    get_eef_pose();

    wish_force << 0, 5, 0, 0, 0, 0;  //期望力
    selection_vector<<0, 1, 0, 0, 0, 0; //选择向量，表示只控制y轴
    // eef_pos_d = eef_pos;
    // eef_rotm_d = eef_rotm;
    Eigen::Matrix3d init_eef_rotm = eef_rotm;
    Eigen::Vector3d init_eef_pos = eef_pos;

    Eigen::Vector3d traj; //轨迹的向量表示
    item = 0;
    double a = 0;
    double b = 0.05/(2*PI); //毫米单位
    double theta = 0;

    while (item < 2800)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        // 生成平面螺旋曲线
        theta = (2*PI / 800) * 0.005 * item + theta; //螺旋线的角度
        double radius = a + b * theta;
        double x = radius * cos(theta) * 0.001;
        double y = 0;
        double z = radius * sin(theta) * 0.001;
        traj << x, y, z;

        item = item + 1;

        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        if (tcp_force[1] > -1){
            cout << "force" <<  tcp_force[1] << endl;
            cout << "spiral search stoped" <<endl;
            break;
        }
        if ((abs(tcp_force[0]) > 4) || (abs(tcp_force[2]) > 4)){
            cout << "x_force" <<  tcp_force[0] << ", y_force" <<  tcp_force[1]<< endl;
            cout << "spiral search stoped" <<endl;  //实际实验时进行调试
            break;
        }
        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        linear_disp_clipped = linear_disp_clipped + traj;
        new_linear_eef = init_eef_pos + init_eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。

        //fixed rotation
        new_rotm_eef = eef_rotm;

        //no-fixed rotation
        // updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);
        

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        if (item % 100 == 0){
        cout << "spiral search item" << item << " excution time is"<< duration.count()<<"ms" << endl;}
    }

    //直线插入***********************************************************************************
        cout << "-----------insertion started------------" << endl;
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    wish_force << 0, 7, 0, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量，表示只控制y轴
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    while (item < 1500)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        item = item + 1;

        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        if (tcp_force[1] < -8){
            cout << tcp_force[1] << endl;
            cout << "insert stoped" <<endl;
            break;
        }
        cout << "force" <<  tcp_force[1] << endl;
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
        robot.servo_p(&new_pos, ABS, loop_rate);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        if (item % 100 == 0){
        cout << "insertion item" << item << " excution time is"<< duration.count()<<"ms" << endl;}
    }
    robot.servo_move_enable(false);
}

void RobotAdmittanceControl::plug_out(){
    robot.servo_move_enable(TRUE);
    // excution_loop();
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    adm_m << 3, 4, 3, 0.5, 0.5, 1.0;
    adm_k << 1100.0, 1100.0, 1100.0, 10.0, 10.0, 10.0;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 3 * sqrt(adm_m[i] * adm_k[i]);
    }
    wish_force << 0, -10, 0, 0, 0, 0;  //期望力
    selection_vector<<1, 1, 1, 0, 0, 0; //选择向量，表示只控制y轴
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    Eigen::Vector3d traj; //轨迹的向量表示
    Eigen::Vector3d init_eef_pos; 
    Eigen::Vector3d e_eef_pos;
    int item = 0;
    init_eef_pos = eef_pos;
    // double x = 0; double y = 0; double z = 0;

    while (item < 2000)
    {   auto start_time = std::chrono::high_resolution_clock::now();

        // 导纳控制的范畴
        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm_d;

        e_eef_pos = eef_rotm.transpose() * (eef_pos - init_eef_pos);
        // cout << e_eef_pos[1] << endl;
        if (abs(e_eef_pos[1]) > 0.018){
            cout << e_eef_pos[1] << endl;
            break;
        }
        
        // 生成直线轨迹
        // if (y > -0.015){
        //     x = 0;
        //     y = -0.0001 + y;  //米单位
        //     z = 0;
        //     traj << x, y, z;
        //     eef_rotm_d = eef_rotm;
        //     eef_pos_d = init_eef_pos + eef_rotm * traj;
        // }

        item = item + 1;

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


        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
        if (item % 100 == 0){
        cout << "plug out item" << item << " excution time is"<< duration.count()<<"ms" << endl;}
    }
    robot.servo_move_enable(false);
}

void RobotAdmittanceControl::move_to_target(int choice){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();
    if (choice == 0){
        new_pos.tran.x = -314.096; new_pos.tran.y = -303.394 + 28 * 2; new_pos.tran.z = 45.632; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
    }
    else if (choice == 1)
    {
        new_pos.tran.x = -314.096; new_pos.tran.y = -303.394 + 28; new_pos.tran.z = 45.632; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
    }
    else if (choice == 2)
    {
        new_pos.tran.x = -314.096; new_pos.tran.y = -303.394; new_pos.tran.z = 45.632; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
    }
    else if (choice == 3)
    {
        new_pos.tran.x = -314.096; new_pos.tran.y = -303.394 - 28; new_pos.tran.z = 45.632; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
    }
    else if (choice == 4)
    {
        new_pos.tran.x = -314.096; new_pos.tran.y = -303.394 - 28 * 2; new_pos.tran.z = 45.632; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
    }
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 10);
    cout<< "move to the picking position"<<endl;

}

void RobotAdmittanceControl::move_to_left_insert(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -209.502; new_pos.tran.y = -547.163; new_pos.tran.z = 273.835; //示教获得
    new_pos.rpy.rx = -90.012 * PI / 180; new_pos.rpy.ry = 44.959 * PI / 180; new_pos.rpy.rz = -149.972 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 10);
    cout<< "move to the picking position"<<endl;

}

void RobotAdmittanceControl::move_to_right_insert(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -221.423; new_pos.tran.y = -554.023; new_pos.tran.z = 275; //示教获得
    new_pos.rpy.rx = 90.45 * PI / 180; new_pos.rpy.ry = -46.074 * PI / 180; new_pos.rpy.rz = 29.705 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 10);
    cout<< "move to the picking position"<<endl;
}

void RobotAdmittanceControl::move_to_left_pick(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -204.283; new_pos.tran.y = -544.799; new_pos.tran.z = 273.835; //示教获得
    new_pos.rpy.rx = -90.012 * PI / 180; new_pos.rpy.ry = 44.959 * PI / 180; new_pos.rpy.rz = -149.972 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 8);
    cout<< "move to the picking position"<<endl;
}

void RobotAdmittanceControl::move_to_right_pick(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -230.277; new_pos.tran.y = -560.198; new_pos.tran.z = 274.958; //示教获得
    new_pos.rpy.rx = 90.45 * PI / 180; new_pos.rpy.ry = -46.074 * PI / 180; new_pos.rpy.rz = 29.705 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 8);

    cout<< "move to the picking position"<<endl;
}


void RobotAdmittanceControl::move_to_left_middle(){
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -278.145; new_pos.tran.y = -453.809; new_pos.tran.z = 273.852; //示教获得
    new_pos.rpy.rx = -90.012 * PI / 180; new_pos.rpy.ry = 44.959 * PI / 180; new_pos.rpy.rz = -149.972 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 10);
    cout<< "move to the picking position"<<endl;
}

void RobotAdmittanceControl::move_to_right_middle(){

    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -277.834; new_pos.tran.y = -454.074; new_pos.tran.z = 275.626; //示教获得
    new_pos.rpy.rx = 90.45 * PI / 180; new_pos.rpy.ry = -46.074 * PI / 180; new_pos.rpy.rz = 29.705 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 10);
    cout<< "move to the picking position"<<endl;
}

void RobotAdmittanceControl::move_to_recycle(){

    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -357.641; new_pos.tran.y = -301.328; new_pos.tran.z = 49.42; //示教获得
    new_pos.rpy.rx = -89.277 * PI / 180; new_pos.rpy.ry = -44.838 * PI / 180; new_pos.rpy.rz = 90.503 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 6);
    cout<< "move to the picking position"<<endl;
}

