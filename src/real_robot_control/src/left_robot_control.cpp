#include <chrono>
#include "real_robot_control/left_robot_control.h"


#include <random>

using namespace std;
using namespace Eigen;

RobotAdmittanceControl::RobotAdmittanceControl():
    nh(std::make_shared<ros::NodeHandle>()),
    client(*nh, "screwactions", true){
    
    for_pub = nh->advertise<real_robot_control::force_pub>("robot_force", 10);
    joint_states_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
    for_pos_pub = nh->advertise<real_robot_control::force_pos_pub>("for_pos", 10);

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
    object_length << 0, 0, 0;   //3分螺柱
    object_rotm = Eigen::Matrix3d::Identity();

    eef_offset_basic << 0, 0, 0.1955 + 0.037;
    eef_offset_rotm_basic = Eigen::AngleAxisd(PI / 4, Eigen::Vector3d::UnitZ());

    eef_offset_to_sensor_basic << 0, 0, 0.146 + 0.037;
    eef_offset_rotm_to_sensor_basic = Eigen::AngleAxisd(PI / 4, Eigen::Vector3d::UnitZ());

    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm;

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length;



}

RobotAdmittanceControl::~RobotAdmittanceControl(){

}

void RobotAdmittanceControl::joint_states_callback(ros::Publisher joint_states_pub)
{
    sensor_msgs::JointState joint_position;

    RobotStatus left_robotstatus;
    robot.get_robot_status(&left_robotstatus);

    JointValue joint_pos = { -PI / 3, PI / 3, PI * 2 / 3, PI / 2, -PI / 2,  PI / 2 }; //右臂位置，写死了

    for (int i = 0; i < 6; i++)
    {
        joint_position.position.push_back(left_robotstatus.joint_position[i]);
        int j = i + 1;
        joint_position.name.push_back("l_j" + to_string(j));
    }
    for (int i = 0; i < 6; i++)
    {
        joint_position.position.push_back(joint_pos.jVal[i]);
        int j = i + 1;
        joint_position.name.push_back("r_j" + to_string(j));
    }
    joint_position.position.push_back(0);
    joint_position.name.push_back("l_p1");
    joint_position.position.push_back(0);
    joint_position.name.push_back("l_p2");
    joint_position.position.push_back(0);
    joint_position.name.push_back("r_c1");
    joint_position.position.push_back(0);
    joint_position.name.push_back("r_p1");
    joint_position.position.push_back(0);
    joint_position.name.push_back("r_p2");
    joint_position.header.stamp = ros::Time::now();
    
    joint_states_pub.publish(joint_position);
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

    link6_rotm = Eigen::AngleAxisd(current_rpy.rz, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(current_rpy.ry, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(current_rpy.rx, Eigen::Vector3d::UnitX());
    eef_rotm =  link6_rotm * eef_offset_rotm;

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
        angular_disp = eef_rotm * adm_vel.tail(3) * T; //将变化转到机器人tcp坐标系下
        linear_disp = selection_vector.head<3>().cwiseProduct(linear_disp);
        angular_disp = selection_vector.tail<3>().cwiseProduct(angular_disp);
        eef_vel = adm_vel;
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
    object_length << 0, 0, 0;   //3分螺柱
    object_rotm = Eigen::Matrix3d::Identity();

    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm;

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length;

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
    object_length << 0, 0, 0;   //3分螺柱
    object_rotm = Eigen::Matrix3d::Identity();

    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm;

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length;
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

        if (abs(e_eef_pos[2]) > 0.07){
            cout << e_eef_pos[2] << endl;
            cout << "\r" << "final item = " << item << endl;
            break;
        }
        linear_disp<< 0, 0, -0.004;
        new_linear_eef = eef_pos + eef_rotm * linear_disp;
        new_rotm_eef = eef_rotm;
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);
    }
    robot.servo_move_enable(false);
    joint_states_callback(joint_states_pub); //更新一下机器人状态
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

bool RobotAdmittanceControl::isInTabuList(int x, int y, int z) {
    for (const auto& item : tabuList) {
        if (std::get<0>(item) == x && std::get<1>(item) == y && std::get<2>(item) == z) {
            cout << "is in tabulist " << endl;
            return true; // 找到匹配的三元组
        }
    }
    return false; // 未找到
    // std::tuple<int, int, int> position = std::make_tuple(x, y, z);
    // return tabuList.find(position) != tabuList.end();
}

void RobotAdmittanceControl::printTabuList() const{
        std::cout << "Tabu List: ";
        for (const auto& item : tabuList) {
            std::cout << "(" << std::get<0>(item) << ", "
                      << std::get<1>(item) << ", "
                      << std::get<2>(item) << ") ";
        }
        std::cout << std::endl;
    }

// 将当前位置加入禁忌表
void RobotAdmittanceControl::addToTabuList(int x, int y, int z) {
    tabuList.emplace_back(x, y, z);
    // std::tuple<int, int, int> position = std::make_tuple(x, y, z);
    // tabuList.insert(position);
    std::cout << "Position (" << x << ", " << y << ", " << z << ") added to tabu list." << std::endl;
}

int RobotAdmittanceControl::random_pure_passive_model(){

    // 获取 ROS 参数
    std::string output_dir;
    std::string filename;
    if (!nh->getParam("log_output_dir", output_dir)) {
        ROS_ERROR("Failed to get 'log_output_dir' parameter, using default.");
        output_dir = "/tmp";  // 默认值
    }
    if (!nh->getParam("log_filename", filename)) {
        ROS_ERROR("Failed to get 'log_filename' parameter, using default.");
        filename = "default_log";
    }
    // 构造完整的日志文件路径
    std::string log_file_path = output_dir + "/" + filename + ".txt";
    std::ofstream logFile(log_file_path, std::ios::app);

    robot.servo_move_enable(TRUE);
    selection_vector<<1, 1, 1, 1, 1, 1; //选择向量
    wish_force << 0, 0, 0, 0, 0, 0;  //期望力
    
    object_rotm = Eigen::Matrix3d::Identity();
    // object_length << 0, -0.032, 0; //m8 长螺丝
    // object_length << 0, -0.019, 0; //螺柱
    // object_length << 0, -0.006, 0; //螺柱
    object_length << 0, 0, 0; //三通和螺母
    // object_length << 0.008, 0, 0; //两通

    int x_z = 1; //1:在xz平面内搜索，0：在yz平面内搜索
    int single = 1; //1为只在z方向搜索，0，在平面内搜索
    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm; 

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length; //更新工具坐标系下的旋转和平移坐标系

    cout << eef_offset << endl;
    cout << eef_offset_rotm << endl;
    // sleep(2);

    adm_m << 3.5, 3.2, 3.2, 0.5, 0.5, 0.5;
    adm_k << 1000.0, 1000.0, 1000.0, 0.7, 0.7, 0.7;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 3.5 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3.5 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[3] = 2 * sqrt(adm_m[3] * adm_k[3]);
    adm_d[4] = 2 * sqrt(adm_m[4] * adm_k[4]);
    adm_d[5] = 2 * sqrt(adm_m[5] * adm_k[5]);
    update_robot_state();
    get_tcp_force();
    get_eef_pose();
    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    Eigen::Vector3d init_eef_pos;
    Eigen::Vector3d start_init_eef_pos;
    Eigen::Vector3d e_eef_pos;
    Eigen::Vector3d expected_xyz;

    init_eef_pos = eef_pos;
    start_init_eef_pos = eef_pos;

    screw_execute_result = 2; //2
    screw_execute_status = 0;
    
    int item = 0;
    int try_time = 0;
    int max_times = 15;
    double x = 0; 
    double y = 0;
    double z = 0;
    int intX; int intY; int intZ;
    int present_width;

    while (item < 26000)
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
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            ros::spinOnce(); //处理回调信息，更改screw_execute_status
        }
        else if ((screw_execute_result == 0) && (screw_execute_status == 0)){
            cout << "达到目标宽度，执行成功" << endl;
            ros::param::get("present_width", present_width); //更新最新的宽度
            e_eef_pos = eef_rotm.inverse() * (eef_pos - init_eef_pos);
            x = e_eef_pos[0] * 1000;
            y = e_eef_pos[1] * 1000;
            z = e_eef_pos[2] * 1000;
            cout << "try_time =" << try_time - 1 << 
            " final real excuted xyz:" << x << ", " << y << ", " << z << endl; //记录的是上一次的实际到达位置
            cout<< "end width = " << present_width << endl;

            logFile << "try_time =" << try_time - 1 << " final real excuted xyz:" << x << ", " << y << ", " << z << endl;
            logFile << "end width = " << present_width << endl;
            break;
        }
        else if ((screw_execute_result == 1) && (screw_execute_status == 0)){
            
            cout << "执行失败，搜索" << endl;

            if (try_time == max_times ){
                cout << "搜索失败，重新尝试" << endl;
                ros::param::get("present_width", present_width); //更新最新的宽度
                e_eef_pos = eef_rotm.inverse() * (eef_pos - init_eef_pos);
                x = e_eef_pos[0] * 1000;
                y = e_eef_pos[1] * 1000;
                z = e_eef_pos[2] * 1000;

                cout << "try_time =" << try_time - 1 << 
                " final real excuted xyz:" << x << ", " << y << ", " << z << endl; //记录的是上一次的实际到达位置
                cout<< "end width = " << present_width << endl;
                logFile << "try_time =" << try_time - 1 << 
                " final real excuted xyz:" << x << ", " << y << ", " << z << endl;
                logFile << "end width = " << present_width << endl;

                goal.num = 3; //打开旋拧口
                client.sendGoal(goal,
                                std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                                std::bind(&RobotAdmittanceControl::active_cb, this),
                                std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                while (true){
                    ros::spinOnce(); //等待打开程序执行完成
                    if (screw_execute_status == 0){
                        cout << "screw_execute_status == 0, break" << endl;
                        break;
                    }
                }
                break;
            }
            // 使用随机设备生成随机数种子
            std::random_device rd;
            std::mt19937 gen(rd()); // Mersenne Twister 随机数生成器
            // 生成 1.0 到 3.0 之间的浮点数
            std::uniform_real_distribution<> dis_real(-5, 5);

            // 生成一个随机数
            x = dis_real(gen); 
            z = dis_real(gen);
            y = dis_real(gen);

            Eigen::Vector3d search_distance;
            update_robot_state(); 
            get_eef_pose();

            if (try_time == 0){ //第一次夹取后，还没有开始搜索
                init_eef_pos = eef_pos; //记录作为坐标系原点
                ros::param::get("present_width", present_width); //获取第一次夹取，初始条件下的宽度
                cout<< "start width = " << present_width << endl;
                e_eef_pos = eef_rotm.inverse() * (init_eef_pos - start_init_eef_pos);
                cout << "first_gripping_xyz =" << e_eef_pos[0] * 1000 << 
                ", " << e_eef_pos[1] * 1000 << ", " << e_eef_pos[2] * 1000 << endl;

                logFile << "start width = " << present_width << endl;
                e_eef_pos = eef_rotm.inverse() * (init_eef_pos - start_init_eef_pos);
                logFile << "first_gripping_xyz =" << e_eef_pos[0] * 1000 << 
                ", " << e_eef_pos[1] * 1000 << ", " << e_eef_pos[2] * 1000 << endl;
            }
            else{
                ros::param::get("present_width", present_width); //更新最新的宽度
                e_eef_pos = eef_rotm.inverse() * (eef_pos - init_eef_pos);

               cout << "##try_time =" << try_time - 1 << " new real excuted xyz:" 
                << e_eef_pos[0] * 1000 << ", " << e_eef_pos[1] * 1000 << ", " << e_eef_pos[2] * 1000 << 
                " width = " << present_width << endl; //记录的是上一次的实际到达位置
                logFile << "##try_time =" << try_time - 1 << " new real excuted xyz:" 
                << e_eef_pos[0] * 1000 << ", " << e_eef_pos[1] * 1000 << ", " << e_eef_pos[2] * 1000 << 
                " width = " << present_width << endl; 

                intX = static_cast<int>(expected_xyz[0]); //加入的是上次执行失败的搜索方向
                intY = static_cast<int>(expected_xyz[1]); 
                intZ = static_cast<int>(expected_xyz[2]);
                addToTabuList(intX, intY, intZ);
                // 为了保证禁忌表的可用性，必须将非搜索方向的状态手动设为0，因为机器人读取的可能会有误差，不能保证是0
            }
            logFile << "----------------------------------------" << std::endl;

            goal.num = 6;
            client.sendGoal(goal,
                            std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&RobotAdmittanceControl::active_cb, this),
                            std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));

            while (true){
                ros::spinOnce(); //等待打开程序执行完成
                if (screw_execute_status == 0){
                    cout << "screw_execute_status == 0, break" << endl;
                    break;}
            }

            if (single){
                    search_distance << 0, 0, z; }
            else{
                if(x_z){
                    search_distance << x, 0, z;
                }
                else{
                    search_distance << 0, y, z;
                }} 

            intX = static_cast<int>(std::round(search_distance[0])); //检查当前的位置是不是在禁忌表中
            intY = static_cast<int>(std::round(search_distance[1])); 
            intZ = static_cast<int>(std::round(search_distance[2]));

            if (isInTabuList(intX, intY, intZ)){
                while(isInTabuList(intX, intY, intZ) || abs(x) > 5 || abs(y) > 5 || abs(z) > 5){
                    if (single){
                        // 归一化方向向量
                        x = 0; y = 0;
                        z = dis_real(gen);
                        search_distance << 0, 0, z;
                    }
                    else{
                        if(x_z){
                            x = dis_real(gen); y = 0;
                            z = dis_real(gen);
                            search_distance << x, 0, z;
    
                        }
                        else{
                            x = 0;
                            y = dis_real(gen);
                            z = dis_real(gen);
                            search_distance << 0, y, z;
                        }
                    }
                intX = static_cast<int>(std::round(search_distance[0])); //检查当前的位置是不是在禁忌表中
                intY = static_cast<int>(std::round(search_distance[1])); 
                intZ = static_cast<int>(std::round(search_distance[2]));
                }
            }

            expected_xyz << search_distance[0], search_distance[1], search_distance[2]; //mm为单位

            std::cout << "##try_time" << try_time << " ## expected x y z" << search_distance[0] << " " 
            << search_distance[1] << " " << search_distance[2] << std::endl;
            logFile << "##try_time" << try_time << " ## expected x y z" << search_distance[0] << " " 
            << search_distance[1] << " " << search_distance[2] << std::endl;

            search_distance = search_distance / 1000; //缩放到1000

            update_robot_state(); //robot move
            get_eef_pose();
            new_linear_eef = init_eef_pos + eef_rotm * search_distance;
            new_rotm_eef = eef_rotm;
            get_new_link6_pose(new_linear_eef, new_rotm_eef);
            new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
            new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;

            robot.servo_p(&new_pos, ABS, 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));

            try_time = try_time + 1;
        }

        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        fp.X = eef_pos[0];
        fp.Y = eef_pos[1];
        fp.Z = eef_pos[2];
        fp.theta = 0;

        new_eef_rotm.x.x = eef_rotm(0,0); new_eef_rotm.y.x = eef_rotm(1,0); new_eef_rotm.z.x = eef_rotm(2,0);
        new_eef_rotm.x.y = eef_rotm(0,1); new_eef_rotm.y.y = eef_rotm(1,1); new_eef_rotm.z.y = eef_rotm(2,1);
        new_eef_rotm.x.z = eef_rotm(0,2); new_eef_rotm.y.z = eef_rotm(1,2); new_eef_rotm.z.z = eef_rotm(2,2);
        robot.rot_matrix_to_rpy(&new_eef_rotm, &new_eef_rpy); //转欧拉角

        fp.FX = new_eef_rpy.rx;
        fp.FY = new_eef_rpy.ry;
        fp.FZ = new_eef_rpy.rz;
 
        for_pos_pub.publish(fp);
        
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;

        item = item + 1;

        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
        
        //fixed rotation
        // new_rotm_eef = eef_rotm;
        
        //no-fixed rotation
        updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
        new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
        new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
        robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
        // cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;

        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);

        if (item % 200 == 0){
        cout << "item" << item << " excution time is"<< duration.count()<<"ms" << endl;}

        ros::spinOnce();
    }
    robot.servo_move_enable(false);
    // 用于handover 测试，最终打开旋拧口
    // goal.num = 3; //打开旋拧口
    // client.sendGoal(goal,
    //                 std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
    //                 std::bind(&RobotAdmittanceControl::active_cb, this),
    //                 std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // while (true){
    //     ros::spinOnce(); //等待打开程序执行完成
    //     if (screw_execute_status == 0){
    //         cout << "screw_execute_status == 0, break" << endl;
    //         break;
    //     }
    // }
    logFile.close();
    if ((screw_execute_result == 1) && (try_time == max_times)){
        return 1;
        }

    else{
        return 0;
    }
}

int RobotAdmittanceControl::pure_passive_model(){

      // 获取 ROS 参数
    std::string output_dir;
    std::string filename;
    if (!nh->getParam("log_output_dir", output_dir)) {
        ROS_ERROR("Failed to get 'log_output_dir' parameter, using default.");
        output_dir = "/tmp";  // 默认值
    }
    if (!nh->getParam("log_filename", filename)) {
        ROS_ERROR("Failed to get 'log_filename' parameter, using default.");
        filename = "default_log";
    }
    // 构造完整的日志文件路径
    std::string log_file_path = output_dir + "/" + filename + ".txt";
    std::ofstream logFile(log_file_path, std::ios::app);

    robot.servo_move_enable(TRUE);
    selection_vector<<1, 1, 1, 1, 1, 1; //选择向量
    wish_force << 0, 0, 0, 0, 0, 0;  //期望力
    /*
        //获取当前工件末端tcp
    int T_00; int T_01; int T_02; int T_03;
    int T_10; int T_11; int T_12; int T_13;
    int T_20; int T_21; int T_22; int T_23;

    ros::param::get("T_s_g_00", T_00); ros::param::get("T_s_g_01", T_01); ros::param::get("T_s_g_02", T_02); ros::param::get("T_s_g_03", T_03);
    ros::param::get("T_s_g_10", T_10); ros::param::get("T_s_g_11", T_11); ros::param::get("T_s_g_12", T_12); ros::param::get("T_s_g_13", T_13);
    ros::param::get("T_s_g_20", T_20); ros::param::get("T_s_g_21", T_21); ros::param::get("T_s_g_22", T_22); ros::param::get("T_s_g_23", T_23);

    Eigen::Matrix4d T_sg;
    T_sg << T_00, T_01, T_02, T_03,
            T_10, T_11, T_12, T_13,
            T_20, T_21, T_22, T_23,
            0, 0, 0, 1;
    T_sg = T_sg.inverse(); //一开始是旋拧坐标系下的抓取点表示，求逆以后才是抓取点下的旋拧点

    object_rotm = T_sg.block<3,3>(0, 0);
    object_length = T_sg.col(3).head<3>();
    */


    object_rotm = Eigen::Matrix3d::Identity();
    // object_length << 0, -0.032, 0; //m8 长螺丝
    // object_length << 0, -0.019, 0; //螺柱
    // object_length << -0.006, 0, 0; //堵头
    object_length << 0, 0, 0; //三通和螺母
    // object_length << 0.012, 0, 0; //m12螺栓
    // object_length << 0.008, 0, 0; //两通

    int x_z = 1; //1:在xz平面内搜索，0：在yz平面内搜索
    int single = 1; //1为只在z方向搜索，0，在平面内搜索

    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm; 

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length; //更新工具坐标系下的旋转和平移坐标系

    adm_m << 3.5, 3.2, 3.2, 0.5, 0.5, 0.5;
    adm_k << 1000.0, 1000.0, 1000.0, 0.5, 0.5, 0.5;
    for (Eigen::Index i = 0; i < adm_m.size(); ++i) {
        adm_d[i] = 3.5 * sqrt(adm_m[i] * adm_k[i]);
    }
    adm_d[0] = 3.5 * sqrt(adm_m[0] * adm_k[0]);
    adm_d[3] = 2 * sqrt(adm_m[3] * adm_k[3]);
    adm_d[4] = 2 * sqrt(adm_m[4] * adm_k[4]);
    adm_d[5] = 2 * sqrt(adm_m[5] * adm_k[5]);
    update_robot_state();
    get_tcp_force();
    get_eef_pose();

    eef_pos_d = eef_pos;
    eef_rotm_d = eef_rotm;

    Eigen::Vector3d init_eef_pos;
    Eigen::Vector3d start_init_eef_pos;
    init_eef_pos = eef_pos;
    start_init_eef_pos = eef_pos;

    screw_execute_result = 2; //2
    screw_execute_status = 0;
    
    int item = 0;
    int try_time = 0;
    int max_times = 15;
    int present_width;
    int last_width;
    double stepSize = 1.6; //sqrt(2)
    if (single){
        stepSize = 1.2; //1
    }
    double gradX = 0, gradZ = 0;
    double prevX = 0, prevZ = 0;
    Eigen::Vector3d expected_xyz; //相对于初始位置的搜索向量
    double directionX, directionZ;
    // double predirectionX, predirectionZ;
    double x = 0; 
    double y = 0;
    double z = 0;
    int intX; int intY; int intZ;
    Eigen::Vector3d e_eef_pos;
    while (item < 26000)
    {   auto start_time = std::chrono::high_resolution_clock::now();
        if ((screw_execute_result == 2) && (screw_execute_status == 0)){
            if (item % 200 == 0){
                 cout << "关闭旋拧口" << endl;}

            goal.num = 5; //夹取旋拧口
            client.sendGoal(goal,
                            std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&RobotAdmittanceControl::active_cb, this),
                            std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1));
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            ros::spinOnce(); //处理回调信息，更改screw_execute_status
        }
        else if ((screw_execute_result == 0) && (screw_execute_status == 0)){
            cout << "达到目标宽度，执行成功" << endl;
            ros::param::get("present_width", present_width); //更新最新的宽度
            e_eef_pos = eef_rotm.inverse() * (eef_pos - init_eef_pos);
                x = e_eef_pos[0] * 1000;
                y = e_eef_pos[1] * 1000;
                z = e_eef_pos[2] * 1000;
                cout << "try_time =" << try_time - 1 << 
                " final real excuted xyz:" << x << ", " << y << ", " << z << endl; //记录的是上一次的实际到达位置
                cout<< "end width = " << present_width << endl;

                logFile << "try_time = " << try_time - 1 <<  //保存到log
                " final real excuted xyz:" << x << ", " << y << ", " << z << endl;
                logFile << "end width = " << present_width << endl;
            break;

        }
        else if ((screw_execute_result == 1) && (screw_execute_status == 0)){
            
            cout << "没有达到目标宽度，继续搜索" << endl;

            //这是程序保护终止条件，防止过度搜索
            if (try_time == max_times ){
                cout << "已经达到最大搜索次数，搜索失败，退出" << endl;
                cout << "try_time =" << try_time - 1 << 
                " final real excuted xyz:" << x << ", " << y << ", " << z << endl; //记录的是上一次的实际到达位置
                cout<< "end width = " << present_width << endl;
                logFile << "try_time =" << try_time - 1 << 
                " [failed] final real excuted xyz:" << x << ", " << y << ", " << z << endl;
                logFile << "end width = " << present_width << endl;

                goal.num = 3; //完全打开旋拧口
                client.sendGoal(goal,
                                std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                                std::bind(&RobotAdmittanceControl::active_cb, this),
                                std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                while (true){
                    ros::spinOnce(); //等待打开程序执行完成
                    if (screw_execute_status == 0){
                        cout << "screw_execute_status == 0, break" << endl;
                        break;
                    }
                }
                break;
            }

            // 这里生成随机数 *************************
            // 使用随机设备生成随机数种子
            std::random_device rd;
            std::mt19937 gen(rd()); // Mersenne Twister 随机数生成器
            // 生成 0 到 1.0 之间的浮点数
            std::uniform_real_distribution<> dis_real(0, 1);
            std::uniform_real_distribution<> dis_real5(-5.0, 5.0); // 生成 -5 到 5.0 之间的浮点数
            //  // 生成 -1.0 或 1.0 的浮点数
            std::uniform_int_distribution<> dis_int(0, 1);
            double sign_x = dis_int(gen) == 0 ? 1.0 : -1.0;
            double sign_z = dis_int(gen) == 0 ? 1.0 : -1.0;
            // 生成一个随机数
            update_robot_state(); 
            get_eef_pose();
            
            //此处用于获取状态，保存状态，特指宽度数据
            if (try_time == 0){ //第一次夹取后，还没有开始搜索

                init_eef_pos = eef_pos; //记录作为坐标系原点
                ros::param::get("present_width", present_width); //获取第一次夹取，初始条件下的宽度
                cout<< "start width = " << present_width << endl;
                e_eef_pos = eef_rotm.inverse() * (init_eef_pos - start_init_eef_pos);
                cout << "first_gripping_xyz =" << e_eef_pos[0] * 1000 << 
                ", " << e_eef_pos[1] * 1000 << ", " << e_eef_pos[2] * 1000 << endl;

                logFile << "start width = " << present_width << endl;
                logFile << "first_gripping_xyz =" << e_eef_pos[0] * 1000 << 
                ", " << e_eef_pos[1] * 1000 << ", " << e_eef_pos[2] * 1000 << endl;
            }
            else{
                last_width = present_width; //将上一次的宽度保留,这是一个负数，越小越好
                ros::param::get("present_width", present_width); //更新最新的宽度
                // 更新当前位置，本来x,y,z可以由上一次计算后的x,y，z直接获得，但由于进行的导纳控制，导致实际位置与理论位置有差别，
                // 因此，在每一轮中，都使用当前的实际位置与上一刻的实际位置进行计算梯度
                e_eef_pos = eef_rotm.inverse() * (eef_pos - init_eef_pos);
                x = e_eef_pos[0] * 1000;
                y = e_eef_pos[1] * 1000;
                z = e_eef_pos[2] * 1000;
                cout << "try_time =" << try_time - 1 << " new real excuted xyz:" << x << ", " << y << ", " << z << " width = " << present_width << endl; //记录的是上一次的实际到达位置
                logFile << "try_time =" << try_time - 1 << " new real excuted xyz:" << x << ", " << y << ", " << z << " width = " << present_width << endl;
                // 为了保证禁忌表的可用性，必须将非搜索方向的状态手动设为0，因为机器人读取的可能会有误差，不能保证是0
                if (single){
                    x = 0;
                    y = 0;
                }
                else{
                    if(x_z){
                        y = 0;

                    }
                    else{
                        x = 0;
                    }
                }
                intX = static_cast<int>(std::round(expected_xyz[0]));
                intY = static_cast<int>(std::round(expected_xyz[1])); //四舍五入
                intZ = static_cast<int>(std::round(expected_xyz[2]));
                addToTabuList(intX, intY, intZ); //能再次进入该循环的，当前的位置必然是失败的位置，加入禁忌表
            }
            logFile << "----------------------------------------" << std::endl;
            //既然要继续搜索，那么应该首先打开旋拧口
            goal.num = 6; //打开旋拧口4mm
            client.sendGoal(goal,
                            std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&RobotAdmittanceControl::active_cb, this),
                            std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));

            while (true){
                ros::spinOnce(); //等待打开旋拧口程序执行完成
                if (screw_execute_status == 0){
                    cout << "screw_execute_status == 0, break" << endl;
                    break;
                }
            }
            /////////////////////////////////////////////////

            Eigen::Vector3d search_distance; //相对于初始位置的搜索向量

            if (try_time == 0){ //第一步，使用随机梯度进行更新
                cout << "try_time=0 的第一次, 随机搜索" << endl;
                //计算随机方向
                directionX = (dis_real(gen) * sign_x ); //(-1, 1) 之间的随机数
                directionZ = (dis_real(gen) * sign_z );

                if (single){
                    // 归一化方向向量
                    prevZ = z; //先记录初始位置
                    prevX = x;

                    double magnitude = std::sqrt(directionZ * directionZ);
                    directionX = 0; //都应该赋值
                    directionZ /= magnitude;
                    z = 0 - stepSize * directionZ;
                    search_distance << 0, 0, z; //理论下一步期望位置
                }

                else{
                    double magnitude = std::sqrt(directionX * directionX + directionZ * directionZ);
                    directionX /= magnitude;
                    directionZ /= magnitude;

                    if(x_z){
                        prevX = x; //初始位置，都是0
                        prevZ = z;
                        x = 0 -  stepSize * directionX;
                        z = 0 -  stepSize * directionZ;
                        search_distance << x, 0, z;
                    }
                    else{
                        prevX = y;
                        prevZ = z;
                        y = 0 - stepSize * directionX;
                        z = 0 - stepSize * directionZ;
                        search_distance << 0, y, z;
                    }
                } 
            }

            else { //从第二步开始，使用前一步与当前的指标差异进行更新
                if (single){
                        // 从第二步开始，利用前后评价指标的差值更新梯度
                    if(abs(present_width - last_width) > 200){ //宽度没变化，梯度为0，延续上一时刻的方向，此时directionZ还没有被更新
                        cout << "grident was calculated, not random" <<endl;
                        gradZ = (present_width - last_width) / (z - prevZ);  // 数值梯度的 y 分量
                        // 计算梯度的大小（模长）
                        double gradMagnitude = std::sqrt(gradZ * gradZ);
                        // 归一化梯度向量，保持移动距离相同
                        directionX = 0; //都应该赋值
                        directionZ = gradZ/gradMagnitude;
                    }
                    prevZ = z;
                    // 更新位置，沿负梯度方向移动，保持固定步长
                    z = prevZ - stepSize * directionZ;
                    search_distance << 0, 0, z;
                    
                }

                else{
                    if(x_z){
                        // 从第二步开始，利用前后评价指标的差值更新梯度，适用于x z 情况
                        if(abs(present_width - last_width) > 200){ //宽度没变化，梯度为0，延续上一时刻的方向，此时directionZ还没有被更新, 宽度有变化才更新梯度
                            cout << "grident was calculated, not random" <<endl;
                            gradX = (present_width - last_width) / (x - prevX);  // 数值梯度的 x 分量
                            gradZ = (present_width - last_width) / (z - prevZ);  // 数值梯度的 y 分量
                            // 计算梯度的大小（模长）
                            double gradMagnitude = std::sqrt(gradX * gradX + gradZ * gradZ);

                            // 归一化梯度向量，保持移动距离相同
                            directionX = gradX/gradMagnitude;
                            directionZ = gradZ/gradMagnitude;
                        }
                        prevX = x;
                        prevZ = z;
                        // 更新位置，沿负梯度方向移动，保持固定步长
                        x = prevX - stepSize * directionX;
                        z = prevZ - stepSize * directionZ;
                        search_distance << x, 0, z;

                    }
                    else{
                        if(abs(present_width - last_width) > 200){
                            cout << "grident was calculated, not random" <<endl;
                            // 从第二步开始，利用前后评价指标的差值更新梯度，适用于y z 情况
                            gradX = (present_width - last_width) / (y - prevX);  // 数值梯度的 x 分量
                            gradZ = (present_width - last_width) / (z - prevZ);  // 数值梯度的 y 分量
                            // 计算梯度的大小（模长）
                            double gradMagnitude = std::sqrt(gradX * gradX + gradZ * gradZ);
                            // 归一化梯度向量，保持移动距离相同
                            directionX = gradX/gradMagnitude;
                            directionZ = gradZ/gradMagnitude;
                        }

                        prevX = y;
                        prevZ = z; //先记录上一次
                        // 更新位置，沿负梯度方向移动，保持固定步长
                        y = prevX - stepSize * directionX;
                        z = prevZ - stepSize * directionZ;
                        search_distance << 0, y, z;
                    }
                }
            }
            
            intX = static_cast<int>(std::round(x));
            intY = static_cast<int>(std::round(y)); //四舍五入
            intZ = static_cast<int>(std::round(z));
            
            int local_try_time = 0;

            // 检查在不在禁忌表或者超过边界，如果在，重复随机搜索，如果被禁忌表包围，则随机跳跃，直到跳出禁忌表
            while (local_try_time < 50) { //如果在重新生成随机搜索
                if ((isInTabuList(intX, intY, intZ)) || abs(x) > 5 || abs(y) > 5 || abs(z) > 5){
                    
                    if (local_try_time < 1){
                        cout << "local_searching ..." << endl;
                        
                        logFile << "local_searching ..." << endl;
                        logFile << "[expected xyz before tabulist:]" << x << ", " << y << ", "<< z << endl;
                    }
                    
                    sign_x = dis_int(gen) == 0 ? 1.0 : -1.0;
                    sign_z = dis_int(gen) == 0 ? 1.0 : -1.0;
                    // 生成一个随机数
                    directionX = (dis_real(gen) * sign_x ); //(-1, 1) 之间的随机数
                    directionZ = (dis_real(gen) * sign_z );

                    if (single){
                            // 归一化方向向量
                            double magnitude = std::sqrt(directionZ * directionZ);
                            directionZ /= magnitude;
                            x = 0;
                            y = 0;
                            z = prevZ - stepSize * directionZ;
                            search_distance << 0, 0, z;
                        }
                    else{
                        double magnitude = std::sqrt(directionX * directionX + directionZ * directionZ);
                        directionX /= magnitude;
                        directionZ /= magnitude;
                        if(x_z){
                            x = prevX - stepSize * directionX;
                            y = 0;
                            z = prevZ - stepSize * directionZ;
                            search_distance << x, 0, z;
    
                        }
                        else{
                            x = 0;
                            y = prevX - stepSize * directionX;
                            z = prevZ - stepSize * directionZ;
                            search_distance << 0, y, z;
                        }
                    }
                    intX = static_cast<int>(std::round(x));
                    intY = static_cast<int>(std::round(y)); //四舍五入
                    intZ = static_cast<int>(std::round(z));
                    local_try_time = local_try_time + 1;

                }
                else{
                    break;
                }
            }
            //跳跃搜索
            cout << "medium xyz = " << x << ", " << y << ", " << z << endl;

            if ((isInTabuList(intX, intY, intZ)) || abs(x) > 5 || abs(y) > 5 || abs(z) > 5){
                cout << "jump search" << endl;
                logFile << "jump search" << endl;
                while( (isInTabuList(intX, intY, intZ)) || abs(x) > 5 || abs(y) > 5 || abs(z) > 5){
                    if (single){
                        // 归一化方向向量
                        x = 0; 
                        y = 0;
                        z = dis_real5(gen);
                        search_distance << 0, 0, z;
                        cout << "jump single" << search_distance[0] << ", " <<search_distance[1] << ", " <<search_distance[2] << endl; 
                    }
                    else{
                        if(x_z){
                            x = dis_real5(gen); 
                            y = 0;
                            z = dis_real5(gen);
                            search_distance << x, 0, z;
                            cout << "jump xz = " << x << search_distance[0] << ", " <<search_distance[1] << ", " <<search_distance[2] << endl; 
                        }
                        else{
                            x = 0;
                            y = dis_real5(gen);
                            z = dis_real5(gen);
                            search_distance << 0, y, z;
                            cout << "jump yz = " << search_distance[0] << ", " <<search_distance[1] << ", " <<search_distance[2] << endl; 
                        }
                    }
                    intX = static_cast<int>(std::round(x));
                    intY = static_cast<int>(std::round(y)); //四舍五入
                    intZ = static_cast<int>(std::round(z));
                }
            }

            expected_xyz << search_distance[0], search_distance[1], search_distance[2]; //mm为单位

            // add more information, using printing method to record!
            cout << " grad  " << - directionX << ", " << - directionZ << endl;
            cout << "try_time = "<< try_time << " new expected x y z = " << search_distance[0] << ", " <<search_distance[1] << ", " <<search_distance[2] << endl;
            logFile << " grad  " << - directionX << ", " << - directionZ << endl;
            logFile << "try_time = "<< try_time << " new expected x y z = " << search_distance[0] << ", " <<search_distance[1] << ", " <<search_distance[2] << endl;

            search_distance = search_distance / 1000; //缩放到m
            update_robot_state(); //robot move
            get_eef_pose();

            new_linear_eef = init_eef_pos + eef_rotm * search_distance;
            new_rotm_eef = eef_rotm;
            get_new_link6_pose(new_linear_eef, new_rotm_eef);
            new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
            new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;

            robot.servo_p(&new_pos, ABS, 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));

            try_time = try_time + 1;
        }

        update_robot_state();
        get_tcp_force();
        get_eef_pose();
        fp.X = eef_pos[0];
        fp.Y = eef_pos[1];
        fp.Z = eef_pos[2];
        fp.theta = 0;

        new_eef_rotm.x.x = eef_rotm(0,0); new_eef_rotm.y.x = eef_rotm(1,0); new_eef_rotm.z.x = eef_rotm(2,0);
        new_eef_rotm.x.y = eef_rotm(0,1); new_eef_rotm.y.y = eef_rotm(1,1); new_eef_rotm.z.y = eef_rotm(2,1);
        new_eef_rotm.x.z = eef_rotm(0,2); new_eef_rotm.y.z = eef_rotm(1,2); new_eef_rotm.z.z = eef_rotm(2,2);
        robot.rot_matrix_to_rpy(&new_eef_rotm, &new_eef_rpy); //转欧拉角

        fp.FX = new_eef_rpy.rx;
        fp.FY = new_eef_rpy.ry;
        fp.FZ = new_eef_rpy.rz;
        // cout << fp.FX * 180 / PI << " " << fp.FY * 180 / PI << " " << fp.FZ * 180 / PI << " " << endl;
        // fp.X = tcp_force[3];
        // fp.Y = tcp_force[4];
        // fp.Z = tcp_force[5];
        // fp.theta = 0;
        for_pos_pub.publish(fp);
        
        eef_pos_d = eef_pos;
        eef_rotm_d = eef_rotm;


        item = item + 1;

        tcp_admittance_control();
        
        linear_disp_clipped = linear_disp.cwiseMin(0.01).cwiseMax(-0.01);
        angluer_disp_clipped = angular_disp.cwiseMin(0.01).cwiseMax(-0.01); //此处获取了在tcp坐标系下机器人末端的位移偏量
        //我们需要在此处对其进行修改，上述偏量经过选择向量的修改只剩z方向的偏移了，我们再加上x,y方向的偏移。
        new_linear_eef = eef_pos + eef_rotm * linear_disp_clipped; //最后将总偏移量再加到原始的tcp坐标上面去。
        
        //fixed rotation
        // new_rotm_eef = eef_rotm;
        
        //no-fixed rotation
        updata_rotation(eef_rotm, angluer_disp_clipped, new_rotm_eef);
        get_new_link6_pose(new_linear_eef, new_rotm_eef);
        new_pos.tran.x = new_linear[0] * 1000; new_pos.tran.y = new_linear[1] * 1000; new_pos.tran.z = new_linear[2] * 1000;
        // new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;
        new_rotm.x.x = new_angular(0,0); new_rotm.y.x = new_angular(1,0); new_rotm.z.x = new_angular(2,0);
        new_rotm.x.y = new_angular(0,1); new_rotm.y.y = new_angular(1,1); new_rotm.z.y = new_angular(2,1);
        new_rotm.x.z = new_angular(0,2); new_rotm.y.z = new_angular(1,2); new_rotm.z.z = new_angular(2,2);
        robot.rot_matrix_to_rpy(&new_rotm, &new_rpy); //转欧拉角
        // cout <<"rot_rpy" << (new_rpy.rx / PI) * 180 << "  " << (new_rpy.ry / PI) * 180<< "  " << (new_rpy.rz / PI) * 180 <<endl;
        new_pos.rpy.rx = new_rpy.rx; new_pos.rpy.ry = new_rpy.ry; new_pos.rpy.rz = new_rpy.rz;

        // new_pos.rpy.rx = current_rpy.rx; new_pos.rpy.ry = current_rpy.ry; new_pos.rpy.rz = current_rpy.rz;
        robot.servo_p(&new_pos, ABS, loop_rate);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);

        if (item % 600 == 0){
        cout << "item" << item << " excution time is"<< duration.count()<<"ms" << endl;}

        ros::spinOnce();
    }
    robot.servo_move_enable(false);

    // 用于handover 测试，最终打开旋拧口
    // goal.num = 3; //打开旋拧口
    // client.sendGoal(goal,
    //                 std::bind(&RobotAdmittanceControl::done_cb, this, std::placeholders::_1, std::placeholders::_2),
    //                 std::bind(&RobotAdmittanceControl::active_cb, this),
    //                 std::bind(&RobotAdmittanceControl::feedback_cb, this, std::placeholders::_1)); 
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // while (true){
    //     ros::spinOnce(); //等待打开程序执行完成
    //     if (screw_execute_status == 0){
    //         cout << "screw_execute_status == 0, break" << endl;
    //         break;
    //     }
    // }
    // cout << "try_time = " << try_time << endl;
    logFile.close();
    std::cout << "Log saved to log.txt" << std::endl;
    
    if ((screw_execute_result == 1) && (try_time == max_times)){
        return 1;
        }

    else{
        return 0;
    }


}

void RobotAdmittanceControl::spiral_search(){
    object_length << 0, 0, 0;   //3分螺柱
    object_rotm = Eigen::Matrix3d::Identity();

    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm;

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length;

    robot.servo_move_enable(true);
    adm_m << 3, 4, 3, 0.5, 0.5, 1.0;
    adm_k << 1000.0, 1100.0, 1000.0, 10.0, 10.0, 10.0;
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
        theta = (2*PI / 800) * 0.008 * item + theta; //螺旋线的角度
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
        if ((abs(tcp_force[0]) > 5) || (abs(tcp_force[2]) > 5)){
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
    wish_force << 0, 12, 0, 0, 0, 0;  //期望力
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
        if (tcp_force[1] < -11){
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
    object_length << 0, 0, 0;   //3分螺柱
    object_rotm = Eigen::Matrix3d::Identity();

    eef_offset_rotm = eef_offset_rotm_basic * object_rotm;
    eef_offset_rotm_to_sensor = eef_offset_rotm_to_sensor_basic * object_rotm;

    eef_offset = eef_offset_basic + eef_offset_rotm_basic * object_length;
    eef_offset_to_sensor = eef_offset_to_sensor_basic + eef_offset_rotm_basic * object_length;
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
    wish_force << 0, -15, 0, 0, 0, 0;  //期望力
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
    robot.linear_move(&new_pos, ABS, true, 20);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态

}

void RobotAdmittanceControl::move_to_left_insert(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -206.784; new_pos.tran.y = -545.427; new_pos.tran.z = 273.495; //示教获得
    new_pos.rpy.rx = -90.012 * PI / 180; new_pos.rpy.ry = 45 * PI / 180; new_pos.rpy.rz = -150.395 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 25);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态

}

void RobotAdmittanceControl::move_to_right_insert(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -223.851; new_pos.tran.y = -554.023; new_pos.tran.z = 274.8; //示教获得
    new_pos.rpy.rx = 90.324 * PI / 180; new_pos.rpy.ry = -46.231 * PI / 180; new_pos.rpy.rz = 28.993 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 25);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态
}

void RobotAdmittanceControl::move_to_left_pick(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -191.895; new_pos.tran.y = -538.196; new_pos.tran.z = 273.835; //示教获得
    new_pos.rpy.rx = -90.012 * PI / 180; new_pos.rpy.ry = 44.2 * PI / 180; new_pos.rpy.rz = -153.56 * PI / 180;
    
    robot.servo_move_enable(false);
    robot.get_joint_position(&cur_joint_pos);
    robot.kine_inverse(&cur_joint_pos, &new_pos, &next_joint_pos);
    robot.joint_move(&next_joint_pos, ABS, TRUE, 0.2);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态
}

void RobotAdmittanceControl::move_to_right_pick(){
    // -np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 1 / 4, -np.pi / 2, np.pi / 2
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -230.277; new_pos.tran.y = -560.198; new_pos.tran.z = 274.958; //示教获得
    new_pos.rpy.rx = 90.45 * PI / 180; new_pos.rpy.ry = -46.074 * PI / 180; new_pos.rpy.rz = 29.705 * PI / 180;
    
    robot.servo_move_enable(false);

    robot.get_joint_position(&cur_joint_pos);
    robot.kine_inverse(&cur_joint_pos, &new_pos, &next_joint_pos);
    robot.joint_move(&next_joint_pos, ABS, TRUE, 0.2);

    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态
}


void RobotAdmittanceControl::move_to_left_middle(){
    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -278.145; new_pos.tran.y = -453.809; new_pos.tran.z = 273.852; //示教获得
    new_pos.rpy.rx = -90.012 * PI / 180; new_pos.rpy.ry = 44.959 * PI / 180; new_pos.rpy.rz = -149.972 * PI / 180;
    
    robot.servo_move_enable(false);

    robot.get_joint_position(&cur_joint_pos);
    robot.kine_inverse(&cur_joint_pos, &new_pos, &next_joint_pos);
    robot.joint_move(&next_joint_pos, ABS, TRUE, 0.2);

    // robot.linear_move(&new_pos, ABS, true, 20);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态
}

void RobotAdmittanceControl::move_to_right_middle(){

    update_robot_state();
    get_eef_pose();

    new_pos.tran.x = -277.834; new_pos.tran.y = -454.074; new_pos.tran.z = 275.626; //示教获得
    new_pos.rpy.rx = 90.45 * PI / 180; new_pos.rpy.ry = -46.074 * PI / 180; new_pos.rpy.rz = 29.705 * PI / 180;
    
    robot.servo_move_enable(false);

    robot.get_joint_position(&cur_joint_pos);
    robot.kine_inverse(&cur_joint_pos, &new_pos, &next_joint_pos);
    robot.joint_move(&next_joint_pos, ABS, TRUE, 0.2);

    // robot.linear_move(&new_pos, ABS, true, 20);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态
}

void RobotAdmittanceControl::move_to_recycle(int choice, int int_value){

    update_robot_state();
    get_eef_pose();

    if (int_value == 3){ //left
        if (choice == 0){
        new_pos.tran.x = -313.341; new_pos.tran.y = -302.973 + 28 * 2; new_pos.tran.z = 54; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 1)
        {
            new_pos.tran.x = -313.341; new_pos.tran.y = -302.973 + 28; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 2)
        {
            new_pos.tran.x = -313.341; new_pos.tran.y = -302.973; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 3)
        {
            new_pos.tran.x = -313.341; new_pos.tran.y = -302.973 - 28; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 4)
        {
            new_pos.tran.x = -313.341; new_pos.tran.y = -302.973 - 28 * 2; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
    }
    if (int_value == 4){ //right
        if (choice == 0){
        new_pos.tran.x = -312.717; new_pos.tran.y = -302.886 + 28 * 2; new_pos.tran.z = 54; //示教获得
        new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 1)
        {
            new_pos.tran.x = -312.717; new_pos.tran.y = -302.886 + 28; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 2)
        {
            new_pos.tran.x = -312.717; new_pos.tran.y = -302.886; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 3)
        {
            new_pos.tran.x = -312.717; new_pos.tran.y = -302.886 - 28; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
        else if (choice == 4)
        {
            new_pos.tran.x = -312.717; new_pos.tran.y = -302.886 - 28 * 2; new_pos.tran.z = 54; //示教获得
            new_pos.rpy.rx = -90 * PI / 180; new_pos.rpy.ry = -45 * PI / 180; new_pos.rpy.rz = -90 * PI / 180;
        }
    }
 
    robot.servo_move_enable(false);
    robot.linear_move(&new_pos, ABS, true, 20);
    cout<< "move to the picking position"<<endl;
    joint_states_callback(joint_states_pub); //更新一下机器人状态
}

